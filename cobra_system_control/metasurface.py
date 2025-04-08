"""
file: metasurface.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file defines the characteristics of the LM10 LCM used in M30 systems.

It also provides a driver for the FPGA LCM Controller
peripheral.
"""
from pathlib import Path
from typing import Tuple

import numpy as np

from cobra_system_control.device import Device
from cobra_system_control import remote

from cobra_system_control.values_utilities import OptionValue
from cobra_system_control.functional_utilities import wait_for_true


LM10_LCM_ORDERS = (*range(1, 463), *(0,) * (512 - 462))


class LM10OrderOv(OptionValue):
    """A class to define the LCM orders in the SPI Flash
    for a LM10 LCM
    """
    OPTIONS = LM10_LCM_ORDERS
    # Special order is 463//2+1 = 232

    @property
    def field(self) -> int:
        """The table index in spi flash
        """
        # pylint: disable-next=no-member
        return self.OPTIONS.index(self.value)

    @property
    def offset(self) -> int:
        """The memory offset in spi flash
        """
        return self.field * 0x400

    @classmethod
    def nonzero_sorted_orders(cls) -> Tuple[int, ...]:
        """Returns sequence of steering orders with 0 removed
        """
        temp = np.asarray(cls.OPTIONS)
        # Some order lists have more than one zero so we need to
        # exclude them all
        return tuple(sorted(temp[np.flatnonzero(temp)]))

    @classmethod
    def from_field(cls, field) -> "OrderOv":
        # pylint: disable-next=no-member
        # pylint: disable-next=unsubscriptable-object
        return cls(cls.OPTIONS[field])


@remote.register_for_serialization
class LcmAssembly:
    """A class to define the configuration of the LCM installed
    into the system.

    Defaults to LM10
    """
    def __init__(self):
        self.order_ov = LM10OrderOv
        self.orders = self.order_ov.OPTIONS
        self.rail_pitch_nm = 300.0
        self.num_rails = 463
        self.tile_size = self.num_rails * self.rail_pitch_nm * 1e-9
        self.lambda_nm = 910
        self.prism_index = 1.8784087
        self.prism_phi_in_deg = 33.94807

        lcm_bin_path = Path(
            Path(__file__).parent, 'resources',
            'lm10_voltage_patterns'
        ).with_suffix('.bin').absolute()
        assert lcm_bin_path.exists()
        self.lcm_bin_path = str(lcm_bin_path)

    def order_to_angle(self, m: int) -> float:
        """Calculates the farfield steering angle for a given steering order.

        Args:
            m: positive integer steering order

        Returns:
            air_phi_out_deg: steering angle out of the system, in degrees
        """
        m = abs(m)
        if m == 0:
            return np.nan
        else:
            prism_phi_out_rad = np.arcsin(
                m * (self.lambda_nm * 1.0e-9 / self.prism_index)
                / self.tile_size
                - np.sin(np.deg2rad(self.prism_phi_in_deg)))
            air_phi_out_rad = np.arcsin(
                np.sin(prism_phi_out_rad) * self.prism_index)
            air_phi_out_deg = np.rad2deg(air_phi_out_rad)
            return air_phi_out_deg

    def angle_to_order(self, air_phi_out_deg: float) -> int:
        """Calculates the closest steering order for a given farfield steering
        angle.

        Args:
            air_phi_out_deg: steering angle out of the system, in degrees

        Returns:
            m: positive integer steering order
        """
        air_phi_out_rad = np.deg2rad(air_phi_out_deg)
        prism_phi_out_rad = np.arcsin(
            np.sin(air_phi_out_rad) / self.prism_index)
        m_float = (
            (self.tile_size
             / (self.lambda_nm * 1.0e-9 / self.prism_index))
            * (np.sin(prism_phi_out_rad) + np.sin(
                np.deg2rad(self.prism_phi_in_deg)))
        )
        m = int(round(m_float))
        return m


class LcmBuff(Device):
    """A class to interface with the FPGA LCM RAM buffer
    peripheral.
    """
    def __init__(
            self, bus: 'I2CBus', device_addr: int,
            memmap_periph: 'MemoryMapPeriph',
    ):
        super().__init__(bus, device_addr, 2, 1, memmap_periph,
                         data_bigendian=False, addr_bigendian=True)
        self.max_tables = 512
        self.bytes_per_table = 1024


class LcmController(Device):
    """A class to interface with the FPGA LCM Controller peripheral.
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph',
                 buff: LcmBuff, ffun: 'FpgaFieldFuncs',
                 scan_device: Device):
        super().__init__(bus, device_addr, 2, 1,
                         memmap_periph,
                         addr_bigendian=True, data_bigendian=False)
        self.buff = buff
        self.ffun = ffun
        self.scan = scan_device
        self._debug_cxt_count = 0
        self._backdoor_cxt_count = 0

    def setup(self):
        # set default operation parameters
        self.write_fields(
            reset_code=0x00,
            pol_finish_ovr=0,
            tp1_done_high=0,
            n_steps=170,
            rst_pw=4,
            tx_wait=7,
            pol_toggle=0,  # 20230126 turned off to support cont. mode
            settle_tc=9,  # 20230127 set the 9V pattern apply TP1 to very short
            pattern_sync_mode='pulse',
            tcon_enable=0,
            pol_invert=1,
            gpio_ito_select=0,
        )

    def disable(self):
        """Runs proper disable sequence for this class"""
        self.write_fields(tcon_enable=0)
        wait_for_true(lambda: self.read_fields(
            'tcon_state', use_mnemonic=True) in ['idle', 'done'],
                      n_tries=5, interval_s=0.1,
                      timeout_msg='could not disable tcon')
