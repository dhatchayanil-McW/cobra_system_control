"""
file: scan_control.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines the parameters that comprise a single
Scan Entry in the full Scan Table and set up the
FPGA peripheral to write the scan table to the
scan param RAM in the FPGA.

Included are helper functions to convert scan parameter
values to FPGA fields.

"""
from collections.abc import ItemsView
import dataclasses as dc
import inspect
import struct
from typing import Sequence, Tuple, Optional, TypeVar

import numpy as np
import pandas as pd

from cobra_system_control.device import Device
import cobra_system_control.exceptions as cobex
from cobra_system_control.itof import FrameSettings, TRIG_DELAY_US, LaserMgSync
from cobra_system_control import remote

from cobra_system_control.functional_utilities import wait_for_true
from cobra_system_control.values_utilities import BoundedValue, OptionValue
from cobra_system_control.validation_utilities import Register, BoundedNumber


SCAN_TABLE_SIZE = 512


@remote.register_for_serialization
class SnrThresholdBv(BoundedValue):
    """SNR Threshold is defined as a u9.3 fixed point value
    providing a max of 511.875. Experiments suggest a max
    SNR of ~600 from a reasonable scene. An SNR threshold
    of 20 provided an image with lots of points.
    """
    _NFRAC = 3
    LIMITS = (0, (2 ** 12 - 1) * 2**(-1 * _NFRAC))
    TOLERANCE = 0

    @property
    def field(self) -> int:
        # converts to SNR field
        return int(self.value / 2**(-1 * SnrThresholdBv._NFRAC))

    @classmethod
    def from_field(cls, field) -> 'SnrThresholdBv':
        # converts to SNR field
        return cls(field * 2**(-1 * SnrThresholdBv._NFRAC))


@remote.register_for_serialization
class BinningOv(OptionValue):
    """A class to define the options for pixel binning
    """
    OPTIONS = [1, 2, 4]

    @property
    def field(self) -> int:
        return self.value


class ScanEntry:
    """Aggregates all scan parameters involved in a scan table entry.

    There is no memory map yaml for these, so ``ScanEntry``
    defines the offset, position, and size below.
    ``Register`` handles validation.
    """

    DATA_WORDS = 16

    # address descriptor (memory maps maps address)
    roi_sel = Register(offset=0, position=6, size=9)

    # data descriptors (memory maps data)
    # blob4 = Register(offset=0, position=24, size=8)
    # blob1 = Register(offset=0, position=12, size=12)
    roi_id = Register(offset=0, position=0, size=12)

    # blob5
    virtual_sensor_bitmask = Register(offset=1, position=24, size=8)
    # blob3 = Register(offset=1, position=12, size=12)
    # blob2 = Register(offset=1, position=0, size=12)

    # blob6
    start_stop_flags = Register(offset=2, position=0, size=32)

    min_frm_length = Register(offset=3, position=16, size=16)

    laser_ci_hdr = Register(offset=4, position=12, size=12)
    laser_ci = Register(offset=4, position=0, size=12)

    npulse_group_f1 = Register(offset=5, position=20, size=12)
    npulse_group_f0 = Register(offset=5, position=8, size=12)
    mod_freq1_opt = Register(offset=5, position=4, size=3)
    mod_freq0_opt = Register(offset=5, position=0, size=3)

    sensor_mode = Register(offset=6, position=30, size=2)
    rwin0_l = Register(offset=6, position=21, size=9)
    rwin0_s = Register(offset=6, position=12, size=9)
    dpulse_group_f1 = Register(offset=6, position=6, size=6)
    dpulse_group_f0 = Register(offset=6, position=0, size=6)

    sync_laser_lvds_mg = Register(offset=7, position=24, size=6)
    inte_burst_length_f1 = Register(offset=7, position=12, size=12)
    inte_burst_length_f0 = Register(offset=7, position=0, size=12)

    inte_burst_length_f1_hdr = Register(offset=8, position=12, size=12)
    inte_burst_length_f0_hdr = Register(offset=8, position=0, size=12)

    steering_idx = Register(offset=9, position=16, size=9)
    pol_cnt_tc_1 = Register(offset=9, position=8, size=8)
    pol_cnt_tc_0 = Register(offset=9, position=0, size=8)

    tp1_period_1 = Register(offset=10, position=16, size=16)
    tp1_period_0 = Register(offset=10, position=0, size=16)

    ito_phase_tc_1 = Register(offset=11, position=16, size=16)
    ito_phase_tc_0 = Register(offset=11, position=0, size=16)

    ito_toggle_tc_1 = Register(offset=12, position=16, size=16)
    ito_toggle_tc_0 = Register(offset=12, position=0, size=16)

    scan_fetch_delay = Register(offset=13, position=16, size=16)
    scan_trigger_delay = Register(offset=13, position=0, size=16)

    def __init__(
            self, *,
            roi_sel: int,   # address descriptor (memory maps maps address)
            roi_id: int,
            virtual_sensor_bitmask: int,
            start_stop_flags: int,
            min_frm_length: int,
            laser_ci: int,
            laser_ci_hdr: int,
            npulse_group_f1: int, npulse_group_f0: int,
            mod_freq1_opt: int, mod_freq0_opt: int,
            sensor_mode: int,
            rwin0_l: int, rwin0_s: int,
            dpulse_group_f1: int, dpulse_group_f0: int,
            inte_burst_length_f1: int, inte_burst_length_f0: int,
            inte_burst_length_f1_hdr: int, inte_burst_length_f0_hdr: int,
            steering_idx: int,
            pol_cnt_tc_1: int, pol_cnt_tc_0: int,
            tp1_period_1: int, tp1_period_0: int,
            ito_phase_tc_1: int, ito_phase_tc_0: int,
            ito_toggle_tc_1: int, ito_toggle_tc_0: int,
            scan_fetch_delay: int, scan_trigger_delay: int,

            sync_laser_lvds_mg: int = LaserMgSync(1).field,
    ):

        self.roi_sel = roi_sel
        self.roi_id = roi_id
        self.virtual_sensor_bitmask = virtual_sensor_bitmask

        self.start_stop_flags = start_stop_flags

        self.steering_idx = steering_idx
        self.laser_ci = laser_ci
        self.sensor_mode = sensor_mode
        self.mod_freq0_opt = mod_freq0_opt
        self.mod_freq1_opt = mod_freq1_opt
        self.npulse_group_f0 = npulse_group_f0
        self.npulse_group_f1 = npulse_group_f1
        self.dpulse_group_f0 = dpulse_group_f0
        self.dpulse_group_f1 = dpulse_group_f1
        self.inte_burst_length_f0 = inte_burst_length_f0
        self.inte_burst_length_f1 = inte_burst_length_f1
        self.rwin0_s = rwin0_s
        self.rwin0_l = rwin0_l
        self.min_frm_length = min_frm_length

        self.tp1_period_1 = tp1_period_1
        self.tp1_period_0 = tp1_period_0
        self.pol_cnt_tc_1 = pol_cnt_tc_1
        self.pol_cnt_tc_0 = pol_cnt_tc_0
        self.ito_phase_tc_1 = ito_phase_tc_1
        self.ito_phase_tc_0 = ito_phase_tc_0
        self.ito_toggle_tc_1 = ito_toggle_tc_1
        self.ito_toggle_tc_0 = ito_toggle_tc_0
        self.scan_fetch_delay = scan_fetch_delay
        self.scan_trigger_delay = scan_trigger_delay

        self.sync_laser_lvds_mg = sync_laser_lvds_mg

        self.inte_burst_length_f0_hdr = inte_burst_length_f0_hdr
        self.inte_burst_length_f1_hdr = inte_burst_length_f1_hdr
        self.laser_ci_hdr = laser_ci_hdr

    @property
    def addr(self) -> int:
        """Returns the address for the first word of the given roi_sel
        """
        return self.roi_sel << ScanEntry.roi_sel.position

    @property
    def data_words(self) -> Sequence[int]:
        """Returns all 16 4-byte words for the given scan params entry
        """
        data_words = [0] * ScanEntry.DATA_WORDS  # 4 bytes each
        for name, param_def in ScanEntry.memmap():
            # skip roi_sel, it determines the addr, not the data
            if param_def == ScanEntry.roi_sel:
                continue
            val = getattr(self, name)
            data_words[param_def.offset] |= (val << param_def.position)

        return data_words

    @classmethod
    def memmap(cls) -> ItemsView:
        """Returns the mapping of {reg_name -> Register} for each register
        descriptor
        """
        return {n: p for n, p in cls.__dict__.items() if
                isinstance(p, Register)}.items()

    FM = TypeVar('PerVirtualSensorMetadata')

    @classmethod
    def build(cls, field_funcs: 'FpgaFieldFuncs',
              roi_sel: int,
              order: 'OrderOv',
              ci_v_field_unshifted: int,
              hdr_ci_v_field_unshifted: int,
              frame: FrameSettings,
              virtual_sensor_bitmask: int,
              start_stop_flags: int,
              binning: int,
              frame_rate: int,

              ito_freq_mult: int = None,
              tp1_period_us: Optional[Tuple[float]] = (None, None),
              pol_cnt: Optional[Tuple[int]] = (None, None),

              scan_fetch_delay: int = None, scan_trigger_delay: int = None,

    ) -> 'ScanEntry':
        """Create ``ScanEntry`` from basic building blocks

        Also chooses the itof delay and pulse widths based on the
        modulation frequency input.
        """
        ito_phase_fields, ito_toggle_fields, tp1_fields, pol_cnt_tc, scan_fetch_delay, scan_trigger_delay = (
            get_scan_time_constants(
                field_funcs, frame,
                binning, frame_rate,
                ito_freq_mult,
                tp1_period_us,
                pol_cnt,
                scan_fetch_delay, scan_trigger_delay,
            ))

        return cls(
            roi_sel=roi_sel,
            roi_id=order.value,  # possibly the roi number / sub scans
            steering_idx=order.field,

            virtual_sensor_bitmask=virtual_sensor_bitmask,

            start_stop_flags=start_stop_flags,
            laser_ci=ci_v_field_unshifted,
            laser_ci_hdr=hdr_ci_v_field_unshifted,

            sensor_mode=frame.pleco_mode.field,
            mod_freq0_opt=frame.mod_freq_int[0].field,
            mod_freq1_opt=frame.mod_freq_int[1].field,
            npulse_group_f0=frame.npulse[0].field,
            npulse_group_f1=frame.npulse[1].field,
            dpulse_group_f0=frame.dpulse[0].field,
            dpulse_group_f1=frame.dpulse[1].field,
            inte_burst_length_f0=frame.inte_burst_length[0],
            inte_burst_length_f1=frame.inte_burst_length[1],
            inte_burst_length_f0_hdr=frame.hdr_inte_burst_length[0],
            inte_burst_length_f1_hdr=frame.hdr_inte_burst_length[1],
            rwin0_s=frame.start_row.field,
            rwin0_l=frame.roi_rows.field,
            min_frm_length=frame.min_frm_length,

            pol_cnt_tc_1=pol_cnt_tc[1], pol_cnt_tc_0=pol_cnt_tc[0],
            tp1_period_1=tp1_fields[1], tp1_period_0=tp1_fields[0],

            ito_phase_tc_1=ito_phase_fields[1],
            ito_phase_tc_0=ito_phase_fields[0],
            ito_toggle_tc_1=ito_toggle_fields[1],
            ito_toggle_tc_0=ito_toggle_fields[0],
            scan_fetch_delay=scan_fetch_delay,
            scan_trigger_delay=scan_trigger_delay,
        )


def get_extra_scan_delay_margin_time_us(
        inte_time_s: Tuple[float, float]) -> int:
    """Calculates extra time to add to min_scan_delay_us
    when determining the time constants for the scan controller.
    This time was determined experimentally.

    This function assumes the typical usage of both modulation
    frequencies having the same integration time setting.
    """
    inte_time_us = int(inte_time_s[0] * 1e6)
    if inte_time_us < 10:
        return inte_time_us * -3.5 + 38
    else:
        return 0


def get_scan_time_constants(
        field_funcs: 'FpgaFieldFuncs',
        frame: FrameSettings,
        binning: int,
        frame_rate: int = None,
        ito_freq_mult: int = None,
        tp1_period_us: Tuple[float] = (None, None),
        pol_cnt: Tuple[int] = (None, None),
        scan_fetch_delay: int = None,
        scan_trigger_delay: int = None,
        ito_phase_deg: float = 180.0,
):
    """Calculates the timing parameters required by the scan controller from
    the frame settings, the binning value, and the desired frame rate.

    The frame_rate will be reduced to the maximum allowed by the other
    settings if its too high for the combination of integration
    time and binning level.

    Note: a binning=1 value (VGA) requires an extra slow-down to
          frame rate to keep RawToDepth processing from being overloaded.

    The field_funcs are used to translate values to fields written
    to the FPGA registers.

    For a 90deg ito phase shift, a >>1 is sufficient
    # Provides 90 deg phase shift
    ito_phase_fields = [x >> 1 for x in ito_toggle_fields]
    """
    # Can only specify one of frame_rate or other scan control arguments
    if frame_rate is not None and any([tp1_period_us[0], pol_cnt[0],
                                       tp1_period_us[1], pol_cnt[1]]):
        raise cobex.ScanPatternValueError(
            "Specifying both a frame_rate and tp1_period or pol_cnt "
            "will lead to unexpected results. Use either frame_rate "
            " or (tp1_period and/or pol_cnt) but not both")

    ff = field_funcs
    extra_scan_delay_us = get_extra_scan_delay_margin_time_us(frame.inte_time_s)
    # min_scan delay is the trigger delay + the time it takes for all
    # 6 subframes to complete
    min_scan_delay_us = (
        6 * frame.t_subframe_us + TRIG_DELAY_US + extra_scan_delay_us
    )

    if frame_rate is not None:
        frame_rate_us = 1 / frame_rate * 1e6
        # Only use the frame rate value if it doesn't violate the minimum timing
        if frame_rate_us < min_scan_delay_us:
            frame_rate_us = min_scan_delay_us
    else:
        frame_rate_us = min_scan_delay_us

    # TP1 Period 1 is based on the subframe timing unless full frame
    # Pol count adjusted for VGA
    if frame.roi_rows == 480:
        tp1p_us1 = tp1_period_us[1] or 500
        if binning == 1:
            pol_1 = pol_cnt[1] or 150
        else:
            pol_1 = pol_cnt[1] or 23
    elif frame.roi_rows in [6, 8, 20]:
        tp1p_us1 = tp1_period_us[1] or int(frame.t_subframe_us)
        if binning == 1:
            pol_1 = pol_cnt[1] or 18
        else:
            pol_1 = pol_cnt[1] or 3
    else:
        raise cobex.ScanPatternValueError(f'Roi rows {frame.roi_rows} not supported')

    # Get the right TP1 Period 0 from what's left
    # after the tp1_1 * pol_1 is taken up
    # make sure this is positive number
    slowed_roi_time_remain = max(1, frame_rate_us - (tp1p_us1 * pol_1 * 2))
    optimal_roi_time_remain = max(1, min_scan_delay_us - (tp1p_us1 * pol_1 * 2))
    if frame.roi_rows == 480:
        scan_trigger_increase = 0
    else:
        scan_trigger_increase = int(slowed_roi_time_remain
                                    - optimal_roi_time_remain)

    ito_freq_mult = ito_freq_mult or 0

    # Adjust pol_0 and tp1p_us0 to get the desired frame rate
    # Make sure that TP1 does not go out of bounds.
    pol_0 = pol_cnt[0] or 1
    while True:
        try:
            tp1p_us0 = (tp1_period_us[0]
                        or np.ceil(slowed_roi_time_remain / (pol_0 * 2)))
            _ = ff.getf_tp1_period(tp1p_us0, ito_freq_mult)
        except cobex.MemoryMapFieldValueError:
            # TP1 field is out of bounds so let's add a pol
            pol_0 += 1
            continue
        else:
            # Field is okay so we can beak out of the loop
            break

    tp1_fields = [ff.getf_tp1_period(x, ito_freq_mult)
                  for x in (tp1p_us0, tp1p_us1)]
    tp1_vals = [ff.getv_tp1_period(x, ito_freq_mult) for x in tp1_fields]

    roi_time_us = (tp1_vals[0] * pol_0 * 2) + (tp1_vals[1] * pol_1 * 2)

    # Make sure that our roi time is longer than scan delay
    # so we don't violate timing
    while roi_time_us < min_scan_delay_us:
        tp1_fields[0] += 1
        tp1_vals = [ff.getv_tp1_period(x, ito_freq_mult) for x in tp1_fields]
        roi_time_us = (tp1_vals[0] * pol_0 * 2) + (tp1_vals[1] * pol_1 * 2)

    if binning != 1:
        msg = f'roi time {roi_time_us:.0f}, fr {frame_rate_us:.0f}, min_scan_delay {min_scan_delay_us:.0f}'
        if frame.roi_rows == 480:
            # If full-frame, we can be slower as it is only for development.
            assert abs(roi_time_us - frame_rate_us) < 2000, msg
        else:
            # If not in VGA, is our frame rate close to what we wanted by less than 20us?
            assert abs(roi_time_us - frame_rate_us) < 20, msg

    # Toggle fields should match the TP1 fields closely,
    # minus 0.5us TP1 pulse width
    ito_toggle_fields = [ff.getf_ito_toggle_tc(x, ito_freq_mult)
                         for x in tp1_vals]

    # Phase shift the ITO edges
    ito_phase_fields = [int(((ito_phase_deg - 180) / 180) * x)
                        for x in ito_toggle_fields]

    # ITO phase needs to be less than the toggle value
    assert ito_phase_fields[0] < ito_toggle_fields[0]
    assert ito_phase_fields[1] < ito_toggle_fields[1]

    scan_trigger_delay = (
        scan_trigger_delay
        or (int(frame.t_subframe_us/2) + scan_trigger_increase)
    )
    # There is an oddity with the GTOF where the trigger and the param writes
    # cannot happen too close together. Fetch delay needs to be delayed from
    # the trigger
    scan_fetch_delay = (
        scan_fetch_delay
        or int(4.5 * frame.t_subframe_us + scan_trigger_delay)
    )

    pol_cnt_tc = (ff.getf_pol_cnt_tc(pol_0), ff.getf_pol_cnt_tc(pol_1))

    return (ito_phase_fields, ito_toggle_fields, tp1_fields,
            pol_cnt_tc, scan_fetch_delay, scan_trigger_delay)


@remote.register_for_serialization
@dc.dataclass(frozen=True)
class ScanTable:
    """A class to collect multiple Scan Entries into a
    scan table.
    """
    # this is the only __init__ arg
    scan_entries: Sequence[ScanEntry]

    # descriptors
    _first_ptr = BoundedNumber(0, 2 ** ScanEntry.roi_sel.size - 1)
    _last_ptr = BoundedNumber(0, 2 ** ScanEntry.roi_sel.size - 1)

    def __post_init__(self):
        # ensure scan entries are contiguous in memory to prevent scan
        # controller from looping over undefined scan states.
        sorted_roi_sel = list(sorted(e.roi_sel for e in self.scan_entries))
        contiguous_roi_sel = list(
            range(sorted_roi_sel[0], sorted_roi_sel[0] + len(sorted_roi_sel)))
        if contiguous_roi_sel != sorted_roi_sel:
            raise ValueError(
                'Scan table memory must be contiguous, but gap(s) were found.')

        # validate pointers
        object.__setattr__(self, '_first_ptr', sorted_roi_sel[0])
        object.__setattr__(self, '_last_ptr', sorted_roi_sel[-1])

    @property
    def valid_ptr_range(self) -> Tuple[int, int]:
        """Valid pointer range (inclusive)
        """
        return self._first_ptr, self._last_ptr

    @classmethod
    def build(cls,
              field_funcs: 'FpgaFieldFuncs',
              orders: Sequence['OrderOv'],
              ci_v_fields_unshifted: Sequence[int],
              hdr_ci_v_fields_unshifted: Sequence[int],
              frames: Sequence[FrameSettings],
              virtual_sensor_bitmask: Sequence[int],
              binning: Sequence['BinningOv'],
              frame_rate: Sequence['FrameRateOv'],
              start_stop_flags: Sequence[int],

              ito_freq_mult: int = None,
              pol_cnt: Optional[Tuple[int]] = (None, None),
              tp1_period_us: Optional[Tuple[float]] = (None, None),
              scan_fetch_delay: int = None, scan_trigger_delay: int = None,
    ) -> 'ScanTable':

        if len({len(orders), len(ci_v_fields_unshifted),
                len(hdr_ci_v_fields_unshifted),
                len(frames), len(virtual_sensor_bitmask)}) != 1:
            raise ValueError(
                'Lengths of orders, ci_v, hdr_ci_v, virtual_sensor_bitmask, '
                'and frames are unequal')

        return cls(tuple(ScanEntry.build(
            field_funcs=field_funcs,
            roi_sel=idx,
            order=o,
            ci_v_field_unshifted=ci,
            hdr_ci_v_field_unshifted=hdrci,
            frame=f,
            virtual_sensor_bitmask=fmask,
            start_stop_flags=ssf,
            binning=bnx.value,
            frame_rate=frx.value,
            ito_freq_mult=ito_freq_mult,
            tp1_period_us=tp1_period_us,
            pol_cnt=pol_cnt,
            scan_fetch_delay=scan_fetch_delay,
            scan_trigger_delay=scan_trigger_delay,
        ) for idx, (o, f, ci, fmask, ssf, bnx, frx, hdrci) in enumerate(
            zip(orders, frames, ci_v_fields_unshifted, virtual_sensor_bitmask,
                start_stop_flags,
                binning, frame_rate,
                hdr_ci_v_fields_unshifted,
                )
        )))

    def __len__(self) -> int:
        return len(self.scan_entries)

    def __getitem__(self, item) -> ScanEntry:
        return self.scan_entries[item]

    def to_dataframe(self):
        cols = inspect.getfullargspec(ScanEntry).kwonlyargs
        df = pd.DataFrame(columns=cols)
        for i, e in enumerate(self.scan_entries):
            data = {col: getattr(e, col) for col in cols}
            df.loc[i] = data
        return df

    def __str__(self) -> str:
        """Returns the scan table in str format for printing
        pandas to_string can return None
        """
        df = self.to_dataframe()
        df['start_stop_flags'] = df['start_stop_flags'].map('{:#034b}'.format)
        df['virtual_sensor_bitmask'] = df['virtual_sensor_bitmask'].map('{:#010b}'.format)
        out = self.to_dataframe().transpose().to_string(index=True)
        if out is not None:
            return str(out)
        else:
            return ""


class Scan(Device):
    """A class to interface with the FPGA Scan peripheral
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph'):
        super().__init__(bus, device_addr, 2, 1, memmap_periph,
                         addr_bigendian=True, data_bigendian=False)

    def setup(self):
        pass

    def stop(self):
        """Stops the scan controller
        """
        self.write_fields(reset='fifo_reset')
        self.write_fields(scan_halt=1)
        self.wait_for_scan_idle()

        self.write_fields(reset='all_reset')
        self.write_fields(scan_halt=0)

    def wait_for_scan_idle(self):
        wait_for_true(lambda: self.read_fields(
            'scan_is_idle') == 1,
                      n_tries=20, interval_s=0.05,
                      timeout_msg='scan controller did not exit gracefully')


class ScanParams(Device):
    """A class to interface with the FPGA Scan Params peripheral.
    Handles writing the scan table and maintaining the valid range of
    scan table pointers.
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph'):
        super().__init__(bus, device_addr, 2, 4,
                         memmap_periph,
                         addr_bigendian=True, data_bigendian=False)
        self.memmap_periph = memmap_periph
        self._scan_table: Optional['ScanTable'] = None

    @property
    def scan_table(self):
        return self._scan_table

    def write_scan_table(self, table: 'ScanTable'):
        """Packs and sends the scan params to the FPGA in a single I2C call
        per entry

        All scan table entries must be written simultaneously to ensure
        the table is contiguous in memory.

        Entries must represent a set of scan tables which are contiguous in
        memory (though not necessarily in order). E.g. a sequence of rois with
        roi_sel = [2, 1, 3] is allowed, but [2, 1, 4] is not allowed
        (missing 3)
        """
        # for speed, write 2 + 32 bytes at a time for each ROI
        for e in table.scan_entries:
            ba = bytearray([*struct.pack(
                self.i2c.addr_pack, e.addr | self.memmap_periph.addr_base)])
            for d in e.data_words:
                ba.extend(struct.pack(self.i2c.data_pack, d))
            self.i2c.bus.write(self.i2c.device_addr, ba)

        self._scan_table = table

    @property
    def valid_ptr_range(self) -> Optional[Tuple[int, int]]:
        """Valid pointer range (inclusive)

        Returns None if the scan table has not been written yet.

        The valid pointer range needs to reference the previous
        value of scan_ram_msb since it changed after you write
        a new scan table. This feature could be implemented
        a different way depending on how you want to control
        the scan table.
        """
        try:
            return self.scan_table.valid_ptr_range
        except AttributeError:
            return None

    def disconnect(self):
        """Reset the pointers when the system is disconnected.
        """
        self._scan_table = None
