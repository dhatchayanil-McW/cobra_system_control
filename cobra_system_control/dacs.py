"""
file: dacs.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provide drivers for all the DACs present on the
Compute Platform (NCB) and the M30 Sensor Head.

The IC specific classes (MchpDacMCP48FVB12, TiDac6578)
are used by the higher-level *Dac classes to adjust
voltage rails to their correct levels.
"""
from typing import Union

import numpy as np
import Pyro5.api

from cobra_system_control.device import Device
from cobra_system_control.cobra_log import log


class DacDevice(Device):
    """A base class to define the DAC interfaces. Inherits
    from Device to enable I2C communication.
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 addr_bytes: int, data_bytes: int,
                 memmap_periph: 'MemoryMapPeriph',
                 addr_bigendian: bool, data_bigendian: bool):
        super().__init__(bus, device_addr, addr_bytes, data_bytes,
                         memmap_periph, True, True)

    def dac_config(self):
        raise NotImplementedError

    def dac_write(self, cmd_idx: int, ch_idx: Union[str, int], payload: int):
        raise NotImplementedError

    def dac_read(self, ch_idx: Union[str, int]):
        raise NotImplementedError

    def set_gain(self, gain: float, ch_idx: int):
        raise NotImplementedError


class MchpDacMCP48FVB12(DacDevice):
    """A class to control a Microchip DAC MCP48FVB12, a 10-bit DAC

    Output at power up is 1.65V

    In M25, VDD is set to 3.3V and the internal reference of 1.22V is used
    For the ITO DAC, the gain should be set to 1, for a range of 0V to 2.44V
    For the CI and VLDA DAC, the gain should be set to 2 for a range of
    0V to 3.3V. The range is limited by VDD otherwise would be 0V to 4.88V.
    The voltage will clamp at 3.3V.
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph',
                 full_scale_pre_gain: float, gain: int,
                 cselect: int):
        super().__init__(bus, device_addr, 2, 1, memmap_periph, True, True)
        self.memmap_periph = memmap_periph
        self.dac_full_scale_pre_gain = full_scale_pre_gain
        self.dac_full_scale = None
        self.gain = gain
        self.dac_settle_us = 9  # (7.8 + 1)  # nominally 7.8 us, plus margin
        self.dac_bits = 10
        self.int_callable = np.int16
        self.bit_shift = 0
        self.tolerance = 0.04
        self.cselect = cselect

    def setup(self):
        self.dac_config()

    def dac_config(self):
        # Set the output to zero
        self.dac_write(None, 0, 0)
        self.dac_write(None, 1, 0)
        # configure VREF for both channels
        self.dac_write(None, 'vref', 0x5)
        vref = self.dac_read('vref')
        if vref != 0x5:
            raise RuntimeError("Failed to configure MCP48FVB12 VREF.")
        # Disable 2x gain for both channels on setup.
        # Changed in the enclosing class setup.

        self.dac_write(None, 'gain', 0b00 << 8)
        rd_g = self.dac_read('gain')
        if rd_g >> 8 & 0b11 != 0:
            raise RuntimeError("Failed to configure MCP48FVB12 gain to 1x.")

    def set_gain(self, gain: int, ch_idx: int):
        if gain not in [1, 2]:
            raise ValueError(f'Only gains of [1,2] valid for MchpDacMCP48FVB12. set {gain}')
        # gain for ch0 is at bit 8 and ch1 is at bit 9
        self.dac_write(None, 'gain', (gain//2) << (ch_idx+8))

        rd_g = self.dac_read('gain')
        and_rd_g = rd_g >> (ch_idx+8) & 0b1
        if and_rd_g != (gain//2):
            raise RuntimeError("Failed to configure MCP48FVB12 gain.")
        self.dac_full_scale = self.dac_full_scale_pre_gain * gain

    def dac_get_addr(self, ch_idx: Union[str, int]):
        channel_addr = {
            0: 0x0,  # DAC0 register
            1: 0x1,  # DAC1 register
            'vref': 0x8,  # VREF register
            'pwrdwn': 0x9,  # Power-down register
            'gain': 0xa,    # Gain and status
        }
        try:
            return channel_addr[ch_idx]
        except KeyError as exc:
            raise KeyError(f'Invalid dac channel addr ({ch_idx})') from exc

    def check_payload_size(self, payload: int):
        if not 0 <= payload < 2 ** 10:
            log.error('DAC data size must be 0 <= data < 2 ** 10')
            raise ValueError(f'Invalid dac_spi data size ({payload})')

    def dac_send_cmd_and_block(self, spi_control_word):
        cnt_mod = 2**self.memmap_periph.fields['spi_cnt'].size
        cnt_before = self.read_fields('spi_cnt')
        self.write_fields(spi_control_word=spi_control_word)

        # Check that the transaction happened
        while True:
            cnt_after = self.read_fields('spi_cnt')
            done = ((cnt_after - cnt_before) % cnt_mod) == 1
            if done:
                break

    def dac_read_resp_and_check(self):
        rdata = self.read_fields('spi_rx_data')
        cmderr = (rdata >> 16) & 0b1
        if cmderr == 0:
            raise ValueError('MCP48FVB12 DAC command failure')
        ret = rdata & 0x3ff
        return ret

    def dac_write(self, cmd_idx: None, ch_idx: Union[str, int], payload: int):
        """Writes to Microchip DAC

        The frequency is min(100/(clk_div + 1), 50) MHz.

        The cmd_idx arg is there to match the interface of the Octal DAC
        """
        self.check_payload_size(payload)
        val = 0
        # Select clock rate to
        val |= 9 << 24
        # Set chip select
        val |= self.cselect << 28
        # build the frame separately for returning
        frame = 0
        # Set to write as defined in the datasheet
        frame |= 0b000 << 16
        # Pick the channel address
        frame |= self.dac_get_addr(ch_idx) << 19
        # Or in the data
        frame |= payload
        val |= frame
        self.dac_send_cmd_and_block(val)
        self.dac_read_resp_and_check()

    def dac_read(self, ch_idx: Union[str, int]):
        """Reads from Microchip DAC
        """
        val = 0
        # Select clock rate to
        val |= 9 << 24
        # Set chip select
        val |= self.cselect << 28
        # build the frame separately for returning
        frame = 0
        # Set to read as defined in the datasheet
        frame |= 0b110 << 16
        # Pick the channel address
        frame |= self.dac_get_addr(ch_idx) << 19
        val |= frame
        self.dac_send_cmd_and_block(val)
        rdata = self.dac_read_resp_and_check()
        return rdata


class TiDac6578(DacDevice):
    """A class to control a TI DAC6578 IC, a 10-bit Dac
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph',  # not needed but keeping interface the same
                 full_scale_pre_gain: float,
                 gain: int = 1):
        super().__init__(bus, device_addr, 1, 2, None, True, True)
        self.dac_full_scale_pre_gain = full_scale_pre_gain
        self.gain = gain  # should be 1
        self.dac_full_scale = full_scale_pre_gain * gain

        self.dac_settle_us = 8  # (7 + 1)  # nominally 7 us, plus margin
        self.dac_bits = 10
        self.int_callable = np.int16
        self.bit_shift = 6
        self.cmd_shift = 4
        self.tolerance = 0.0125

    def connect(self):
        pass

    def setup(self):
        self.dac_config()

    def dac_config(self):
        pass

    def get_channel_addr(self, ch_idx: int):
        if isinstance(ch_idx, int):
            if ch_idx not in range(8):
                raise ValueError(f'Invalid dac_iic ch_idx ({ch_idx})')
            else:
                return ch_idx
        else:
            raise TypeError(f'Invalid type {type(ch_idx)} '
                            f'for dac_iic ch_idx')

    def set_gain(self, gain: int, ch_idx: int):
        pass

    def get_write_command(self, cmd_idx: Union[str, int]):
        command_addr = {
            'write_register': 0x0,
            'update_register': 0x1,
            'write_update_global': 0x2,
            'write_update': 0x3,
            'shutdown': 0x4,
            'clear_register': 0x5,
            'write_ldac': 0x6,
            'reset': 0x7,
        }
        try:
            return command_addr[cmd_idx]
        except KeyError as exc:
            raise KeyError(f'Invalid dac cmd ({cmd_idx})') from exc

    def get_read_command(self, cmd_idx: Union[str, int]):
        command_addr = {
            'read_input_reg': 0x0,
            'read_reg': 0x1,
            'read_pw_dwn_reg': 0x4,
            'read_clear_reg': 0x5,
            'read_ldac_reg': 0x6,
        }
        try:
            return command_addr[cmd_idx]
        except KeyError as exc:
            raise KeyError(f'Invalid dac cmd ({cmd_idx})') from exc

    def check_payload_size(self, payload: int):
        """Check payload size for DAC6578 (10-bit)

        payload sent to DAC is 16 bits, 6 highest bits are DNC
        so shift by 6 to get true payload
        """
        payload >>= self.bit_shift
        if not (0 <= payload < (2 ** self.dac_bits)):
            raise ValueError(f'Invalid dac6578 data size ({payload})')

    def dac_write(self, cmd_idx: str, ch_idx: int, payload: int):
        """Write to TI Octal DAC6578 (10-bit)
        Information for communication found at
            https://www.ti.com/product/DAC6578
        """
        self.check_payload_size(payload)

        cmd_value = self.get_write_command(cmd_idx)
        shifted_cmd_value = cmd_value << self.cmd_shift

        chan_value = self.get_channel_addr(ch_idx)

        # Produce 8 bit message to specify channel and action
        addr_access_byte = shifted_cmd_value | chan_value
        self.i2c.write(addr_access_byte, payload)

    def dac_read(self, ch_idx: int):
        """Write to TI Octal DAC6578 (10-bit)
               Information for communication found at
                   https://www.ti.com/product/DAC6578
        """
        cmd_value = 0x1
        shifted_cmd_value = cmd_value << self.cmd_shift

        chan_value = self.get_channel_addr(ch_idx)

        # Produce 8 bit message to specify channel and action
        addr_access_byte = shifted_cmd_value | chan_value
        return self.i2c.read(int(addr_access_byte))


class VDac:
    """Base class to define functionality for
    DACs to convert voltages to DAC fields.
    """
    def __init__(
            self, dac_device: DacDevice, channel_idx: int,
            enable_ctrl: Union[
                None, 'LcmController',
                'CpuGpio', 'DummyDevice'],
    ):
        self.dac = dac_device
        self.chan_idx = channel_idx
        self.slope = None
        self.offset = None
        self.enable_ctrl = enable_ctrl

    def connect(self):
        self.dac.connect()
        if self.enable_ctrl is not None:
            self.enable_ctrl.connect()

    def setup(self):
        self.dac.setup()
        self.dac.set_gain(self.dac.gain, self.chan_idx)
        if self.enable_ctrl is not None:
            self.enable_ctrl.setup()

    def field_unshifted_from_voltage(self, vmain: float) -> int:
        """Gets the raw register value from a given voltage
        """
        field = self.dac.int_callable((self.slope * vmain + self.offset)
                                      / self.dac.dac_full_scale
                                      * 2**self.dac.dac_bits)
        return np.clip(field, 0, 2**self.dac.dac_bits - 1)

    def field_from_voltage(self, vmain) -> int:
        """Gets the shifted field register from a given voltage
        """
        return self.field_unshifted_from_voltage(vmain) << self.dac.bit_shift

    def set_voltage(self, vset: float):
        """Sets the DAC output voltage that controls VMAIN voltage of sensor head with.
        Check VmainBv class for conversion from DAC output to VMAIN Voltage
        """
        self.dac.dac_write('write_update', self.chan_idx,
                           self.field_from_voltage(vset))

    def voltage_from_field(self, field_val: int) -> float:
        return (((field_val >> self.dac.bit_shift)
                 / 2**self.dac.dac_bits
                 * self.dac.dac_full_scale
                 - self.offset)
                / self.slope)

    def get_field(self) -> int:
        """Returns vmain field
        """
        return self.dac.dac_read(self.chan_idx)

    def get_voltage(self) -> float:
        """Returns the DAC output voltage that controls VMAIN voltage
        of sensor head
        """
        return self.voltage_from_field(self.get_field())

    def disconnect(self):
        self.dac.disconnect()

    def apply_settings(self):
        pass

    def enable(self):
        pass

    def disable(self):
        pass

    def raw_set_zero_voltage(self):
        """Uses direct dac write to do a backdoor
        setting of the DAC to zero volts
        """
        self.dac.dac_write('write_update', self.chan_idx, 0)

    def raw_set_max_dac_voltage(self):
        self.dac.dac_write(
            'write_update', self.chan_idx, self.dac.dac_bits << self.dac.bit_shift)


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class SensorVDac(VDac):
    """Class to control the DAC on the NCB that sets
    the output of the 3.3V regulator

    Based on the chosen resistors, the calculated range is
    3.3V DAC = 2.75V Regulator output
    0V DAC = 3.95V Regulator output
    """
    def __init__(self, dac_device: DacDevice, channel_idx, board_type,
                 enable_ctrl: Union['CpuGpio', 'DummyDevice']):
        super().__init__(dac_device, channel_idx, enable_ctrl)
        self.board_type = board_type
        if self.board_type == "nxp":
            # 3.3 = m (2.75) + b
            # 0 = m (3.95) + b
            self.slope = -2.75
            self.offset = 10.8625
        else:
            raise ValueError(f'slope and offset not defined for {board_type}')

    def enable(self):
        self.enable_ctrl.enable()

    def disable(self):
        self.enable_ctrl.disable()


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class LcmVDac(VDac):
    """A class to control the DAC to the set the LCM voltage rail.
    """
    def __init__(
            self, dac_device: DacDevice, channel_idx, board_type,
            cmb_enable_ctrl: Union['CpuGpio', 'DummyDevice'],
            fpga_enable_ctrl: 'LcmController',
    ):
        super().__init__(dac_device, channel_idx, None)
        self.cmb_enable_ctrl = cmb_enable_ctrl
        self.fpga_enable_ctrl = fpga_enable_ctrl
        self.board_type = board_type
        if self.board_type == "nxp":
            # 3.3 = m (12.36) + b
            # 0 = m (20.97) + b
            self.slope = -0.38327
            self.offset = 8.0373
        else:
            raise ValueError(f'slope and offset not defined for {board_type}')

    def connect(self):
        self.dac.connect()
        self.cmb_enable_ctrl.connect()
        self.fpga_enable_ctrl.connect()

    def setup(self):
        self.dac.setup()
        self.dac.set_gain(self.dac.gain, self.chan_idx)
        self.cmb_enable_ctrl.setup()
        self.fpga_enable_ctrl.setup()

    def enable(self):
        self.cmb_enable_ctrl.enable()
        self.fpga_enable_ctrl.write_fields(gpio_pwr_en=1)

    def disable(self):
        self.fpga_enable_ctrl.write_fields(gpio_pwr_en=0)
        self.cmb_enable_ctrl.disable()


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class ItoDac(VDac):
    """A class to control the DAC to set the ITO voltage.
    """
    def __init__(self, dac_device: DacDevice, channel_idx: int):
        super().__init__(dac_device, channel_idx, None)
        self.slope = 0.13556
        self.offset = 1.22

    def set_voltage(self, vset: float):
        """Sets the ITO voltage. Using a negative number reverses the polarity
        relative to the POL signal.

        POS_V = 1.5 * 9V_REF - 3.57 * DAC
        NEG_V = 0.5 * 9V_REF + 3.57 * DAC
        DAC adjustable from 0V to 2.44V

        """
        if not -9 <= vset <= 9:
            raise ValueError(f'Ito V of {vset} outside range of [-9, 9]')
        self.dac.dac_write(None, self.chan_idx, self.field_from_voltage(vset))
