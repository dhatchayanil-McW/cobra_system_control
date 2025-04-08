"""
file: itof.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file provides a class to communicate with the
GPixel GTOF0503 indirect time-of-flight sensor including
all initial configuration.
"""
import dataclasses as dc
import enum
import itertools
from textwrap import dedent
import time
from typing import Union, Tuple

import numpy as np

from cobra_system_control.device import Device
from cobra_system_control.cobra_log import log
from cobra_system_control.fpga_misc import FpgaDbg
from cobra_system_control.memory_map import PLECO_MEMORY_MAP
from cobra_system_control import remote

from cobra_system_control.values_utilities import OptionValue, BoundedValue
from cobra_system_control.validation_utilities import cast


N_TAPS = 3
N_ROWS = 480
N_COLS = 640

# Values derived from datasheet and measurement.
PRE_PROCESS_S = 10e-6
RESET_TIME_S = 5.28e-6
FOT1_S = 2.64e-6
FOT2_S = 15.84e-6
SMP_S = 8e-6
SUBFRAME_FIXED_TIME = RESET_TIME_S + FOT1_S + FOT2_S + SMP_S
SUBFRAME_TIME_MARGIN = 54e-6
READOUT_TIME_S_PER_ROW = 8.3e-6
FRAME_TIME_MARGIN = 23e-6

# email from Gpixel Applications Engineer on 8/13/2021
PLL2_MHZ = 1000
TCLK_PIX_MHZ = 100
TLINE_US = 1 / TCLK_PIX_MHZ * 264

# time that appears to be tacked on to each subframe on top of
# the end of the min_frm_length duration
SUBFRAME_DEAD_TIME_US = 5.25

# amount of time between rising edge of HW trigger and start of dummy bursts
TRIG_DELAY_US = 160.1

PLL2_FREQ_HZ = PLL2_MHZ * 1e6
MAX_LOCAL_DC = 0.5
LASER_PREHEAT = 0
DUMMY_PREHEAT = 1

# we always want DPULSE at max (63), and NPULSE follows from duty cycle = 50%
NPULSE = 63
DPULSE = 63
DLASER = 12
DUMMY_BURSTS = 12


class ModFreqIntOv(OptionValue):
    """Sets valid options for the mod freq divisor in MHz

    The modulation frequency is calculated as fcld_pll2 / ModFreqIntOv
    """
    OPTIONS = range(3, 11)

    @property
    def clk_freq_hz(self) -> float:
        return PLL2_FREQ_HZ / self.value

    @property
    def laser_freq_hz(self) -> float:
        return self.clk_freq_hz / 3

    @property
    def clk_period_s(self) -> float:
        return 1 / self.clk_freq_hz

    @property
    def field(self) -> int:
        return self.value - 3

    @classmethod
    def from_field(cls, field) -> 'ModFreqIntOv':
        return cls(field + 3)


class NPulseOv(OptionValue):
    """Sets valid options for the number of pulses laser pulses.

    Needs to be a multiple of N_TAPS
    """
    OPTIONS = range(0, 2 ** 12 - 1, N_TAPS)

    @property
    def field(self):
        # define this for consistency
        return self.value

    @property
    def fields(self) -> Tuple[int, int]:
        """Returns (lo, hi) npulse fields"""
        return self.value & 0xFF, (self.value >> 8) & 0xFF

    @property
    def lo(self) -> int:
        return self.fields[0]

    @property
    def hi(self) -> int:
        return self.fields[1]

    @classmethod
    def from_fields(cls, lo: int, hi: int):
        return NPulseOv(lo + (hi << 8))


class DPulseOv(OptionValue):
    """Sets valid options for the number of pulses.

    Needs to be a multiple of N_TAPS
    """
    OPTIONS = range(0, 2 ** 6 - 1, N_TAPS)

    @classmethod
    def max(cls) -> int:
        return cls.OPTIONS[-1]

    @property
    def field(self):
        return self.value


class DLaserOv(OptionValue):
    """Sets valid options for number of dlaser (non-laser) integration pulses
    """
    OPTIONS = range(0, 2 ** 6 - 1, N_TAPS)

    @property
    def field(self) -> int:
        return self.value


class PlecoMode(enum.Enum):
    """Pleco modes

    Tuple of (name, n_subframes, n_taps, n_freqs)
    """
    # VIDEO and SMFD are not currently supported
    # VIDEO = ('video', 1, 1, 1, 2)
    # SMFD = ('smfd', 3, 3, 1, 1)
    DMFD = ('dmfd', 6, 3, 2, 0)

    def __init__(self,
                 kind: str,
                 n_subframes: int,
                 n_taps: int,
                 n_freq: int,
                 field: int):
        self.kind = kind
        self.n_subframes = n_subframes
        self.n_taps = n_taps
        self.n_freq = n_freq
        self.field = field


class InteTimeSBv(BoundedValue):
    """Sets integration time between the bound limits.
    The lower limit is 1us set by the valid API values
        with a 1e12 floating point tolerance.
    The upper limit is 20us set by the valid API values
        (and to a value considered safe for laser reliability)
        with a 1e12 floating point tolerance.
    """
    MAX_LASER_INTE_SECONDS = 20e-6 + 1e-12
    LIMITS = (1e-6 - 1e-12, MAX_LASER_INTE_SECONDS)
    TOLERANCE = 0.05


class StartRow(OptionValue):
    """Valid starting rows"""
    OPTIONS = range(N_ROWS)

    @property
    def field(self) -> int:
        return self.value + 4


class RoiRows(OptionValue):
    """Valid ROI heights"""
    OPTIONS = (6, 8, 20, N_ROWS)

    @property
    def field(self) -> int:
        return self.value + 4


class FovRowsOv(OptionValue):
    """Valid number of rows in an FOV
    """
    OPTIONS = range(1, N_ROWS+1)

    @property
    def field(self) -> int:
        return self.value


class DelayNsBv(BoundedValue):
    """MG / Laser delays"""
    MAX_COARSE = 9
    MAX_FINE = 9
    GATE_COARSE_NS = 0.650
    GATE_FINE_NS = 0.170
    LIMITS = (0, GATE_COARSE_NS * MAX_COARSE + GATE_FINE_NS * MAX_FINE)
    TOLERANCE = GATE_FINE_NS

    @property
    def fields(self) -> (int, int):
        """Returns (coarse, fine) fields"""
        field_combos = DelayNsBv.delay_combinations()
        valid_delays = np.array([
            DelayNsBv.from_fields(*f).value
            for f in field_combos
        ])
        return field_combos[np.argmin(abs(valid_delays - self.value))]

    @classmethod
    def from_fields(cls, coarse, fine):
        return cls(coarse * DelayNsBv.GATE_COARSE_NS
                   + fine * DelayNsBv.GATE_FINE_NS)

    @classmethod
    def max(cls):
        return cls.from_fields(DelayNsBv.MAX_COARSE,
                               DelayNsBv.MAX_FINE)

    @classmethod
    def delay_combinations(cls) -> Tuple[Tuple[int, int], ...]:
        """Computes all possible combinations of delays using the coarse and
        fine registers. Returning field values sorted in pairs which result
         in increasing delays"""
        field_combinations = tuple(
            itertools.product(range(0, cls.MAX_COARSE + 1),
                              range(0, cls.MAX_FINE + 1)))
        delays = (c * cls.GATE_COARSE_NS + f * cls.GATE_FINE_NS
                  for c, f in field_combinations)
        sorted_delay_fields = tuple(
            delay_field for _, delay_field
            in sorted(zip(delays, field_combinations), key=lambda x: x[0]))

        return sorted_delay_fields


class LaserMgSync(OptionValue):
    """Valid options for syncing the laser pulse with MG
    """
    OPTIONS = (0, 1, 2)

    @property
    def field(self) -> int:
        return (self.value + 1) * 21

    @classmethod
    def from_field(cls, field) -> 'LaserMgSync':
        return cls(field // 21 - 1)


class NumFramesOv(OptionValue):
    """Valid options for number of frames"""
    OPTIONS = range(0, 2 ** 16 - 1)

    @property
    def fields(self) -> Tuple[int, int]:
        return self.value & 0xFF, (self.value >> 8) & 0xFF

    @classmethod
    def continuous(cls, mode: PlecoMode):
        return cls({PlecoMode.DMFD: 10922}[mode])


class PgaGainOv(OptionValue):
    """Valid options for PGA Gain"""
    OPTIONS = range(0, 2 ** 5 - 1)

    LIMITS = (1.0, 1.0 + 0.1 * (2 ** 5 - 1))
    TOLERANCE = 0.1

    @property
    def gain(self) -> float:
        return self.value * 0.1 + 1.0

    @property
    def field(self) -> int:
        return self.value

    @classmethod
    def from_gain(cls, gain: float):
        return cls(int((gain - 1.0) / 0.1))


@remote.register_for_serialization
@dc.dataclass
class FrameSettings:
    """A class to define the parameters that are needed to configure
    a single depth measurement.
    """
    start_row: StartRow
    roi_rows: RoiRows
    pleco_mode: PlecoMode
    mod_freq_int: Tuple[ModFreqIntOv, ModFreqIntOv]
    inte_time_s: Tuple[InteTimeSBv, InteTimeSBv]
    hdr_inte_time_s: Tuple[InteTimeSBv, InteTimeSBv]
    dummy_bursts: Tuple[int, int]
    dlaser: Tuple[DLaserOv, DLaserOv]
    n_frames_capt: NumFramesOv

    npulse: Tuple[NPulseOv, NPulseOv] = dc.field(init=False)
    dpulse: Tuple[DPulseOv, DPulseOv] = dc.field(init=False)
    inte_burst_length: Tuple[int, int] = dc.field(init=False)
    hdr_inte_burst_length: Tuple[int, int] = dc.field(init=False)

    def __init__(
            self,
            start_row: int,
            roi_rows: int = 8,
            pleco_mode: PlecoMode = PlecoMode.DMFD,
            mod_freq_int: Tuple[int, int] = (8, 7),
            inte_time_s: Union[float, Tuple[float, float]] = (
                14e-6, 14e-6),
            hdr_inte_time_s: Union[float, Tuple[float, float]] = (
                5e-6, 5e-6),
            dummy_bursts: Tuple[int, int] = (DUMMY_BURSTS, DUMMY_BURSTS),
            dlaser: Tuple[int, int] = (DLASER, DLASER),
            n_frames_capt: int = 1,
    ):

        self.start_row = cast(start_row, StartRow)
        self.roi_rows = cast(roi_rows, RoiRows)
        self.pleco_mode = pleco_mode
        self.dummy_bursts = dummy_bursts
        self.dlaser = cast(dlaser[0], DLaserOv), cast(dlaser[1], DLaserOv)

        if self.roi_rows.value > (N_ROWS - self.start_row):
            raise ValueError(
                f'Too many rois rows ({self.roi_rows} given '
                f'the starting row ({self.start_row}.)'
            )

        self.mod_freq_int = (cast(mod_freq_int[0], ModFreqIntOv),
                             cast(mod_freq_int[1], ModFreqIntOv))
        if self.mod_freq_int[0] - self.mod_freq_int[1] != 1:
            raise ValueError(
                f'The first modulation frequency integer must be exactly 1 '
                f'greater than the second. They are {self.mod_freq_int[0]} '
                f'and {self.mod_freq_int[1]}, respectively.'
            )

        if isinstance(inte_time_s, float):
            inte_time_s = (inte_time_s, inte_time_s)
        self.inte_time_s = (cast(inte_time_s[0], InteTimeSBv),
                            cast(inte_time_s[1], InteTimeSBv))

        if isinstance(hdr_inte_time_s, float):
            hdr_inte_time_s = (hdr_inte_time_s, hdr_inte_time_s)
        self.hdr_inte_time_s = (cast(hdr_inte_time_s[0], InteTimeSBv),
                                cast(hdr_inte_time_s[1], InteTimeSBv))

        # our current operating mode always maxes these out at 50% DC
        # we can revisit this when we experiment with different duty cycles
        if any(self.dlaser[i].value >= NPULSE for i in (0, 1)):
            raise ValueError(
                f'dlaser cannot be greater than or equal to the number '
                f'of laser-active npulses, but they are currently '
                f'{self.dlaser} and {NPULSE}, respectively')

        # dlaser is the number of cycles of npulse to not fire the laser,
        # but still integration
        # adjust npulse and dpulse here to maintain the duty cycle
        self.npulse = (NPulseOv(NPULSE + self.dlaser[0]),
                       NPulseOv(NPULSE + self.dlaser[1]))
        self.dpulse = (DPulseOv(DPULSE - self.dlaser[0]),
                       DPulseOv(DPULSE - self.dlaser[1]))

        self.inte_burst_length = (
            int(self.inte_time_s[0] / (
                    (self.npulse[0] - self.dlaser[0])
                    * self.mod_freq_int[0].clk_period_s)),
            int(self.inte_time_s[1] / (
                    (self.npulse[1] - self.dlaser[1])
                    * self.mod_freq_int[1].clk_period_s))
        )
        if any(burst_len <= 0 for burst_len in self.inte_burst_length):
            raise ValueError(
                f'register inte_burst_len must be greater than 0,'
                f'but it is currently {self.inte_burst_length}. '
                f'Please increase the integration time.'
                f'inte_time_s = {self.inte_time_s}, '
                f'npulse = {self.npulse}, '
                f'dlaser = {self.dlaser}, '
                f'dpulse = {self.dpulse}, '
                f'mod_freq_int clk period = {self.mod_freq_int[0].clk_period_s}, '
                f'{self.mod_freq_int[1].clk_period_s}'
            )

        self.hdr_inte_burst_length = (
            int(self.hdr_inte_time_s[0] / (
                    (self.npulse[0] - self.dlaser[0])
                    * self.mod_freq_int[0].clk_period_s)),
            int(self.hdr_inte_time_s[1] / (
                    (self.npulse[1] - self.dlaser[1])
                    * self.mod_freq_int[1].clk_period_s))
        )
        if any(burst_len <= 0 for burst_len in self.hdr_inte_burst_length):
            raise ValueError(
                f'register hdr_inte_burst_len must be greater than 0,'
                f'but it is currently {self.hdr_inte_burst_length}. '
                f'Please increase the HDR integration time.'
                f'inte_time_s = {self.hdr_inte_time_s}, '
            )

        self.n_frames_capt = cast(n_frames_capt, NumFramesOv)
        if n_frames_capt == 0:  # continuous mode
            self.n_frames_capt = NumFramesOv.continuous(self.pleco_mode)

    @property
    def metadata_size(self) -> int:
        """Total number of metadata elements, including empty elements."""
        return N_COLS * self.pleco_mode.n_taps

    @property
    def frame_size(self) -> int:
        """Total number of elements in a frame, including metadata."""
        data_size = (self.roi_rows * N_COLS *
                     self.pleco_mode.n_taps * self.pleco_mode.n_subframes)
        return data_size + self.metadata_size

    def comp_frame_time_us(self):
        """Computes the expected frame time, in microseconds.

        Time from frame start to frame start
        """
        return self.pleco_mode.n_subframes * self.t_subframe_us

    def comp_frame_rate_hz(self):
        return 1e6 / self.comp_frame_time_us()

    @property
    def t_subframe_us(self):
        """ Defined by He Ren 8/13/2021
        Joe observed fixed offset on 11/29/2021
        """
        return TLINE_US * self.min_frm_length + SUBFRAME_DEAD_TIME_US

    @property
    def inte_total_burst_length(self) -> Tuple[int, int]:
        return (self.inte_burst_length[0] + self.dummy_bursts[0],
                self.inte_burst_length[1] + self.dummy_bursts[1])

    @property
    def inte_state_us(self) -> float:
        longest_inte_us = max(self.inte_time_s) * 1e6
        duty_cycle = NPULSE / (NPULSE + DPULSE)
        return longest_inte_us / duty_cycle

    @property
    def fclk_mod_mhz(self) -> Tuple[float, float]:
        """ He Ren, 8/13/2021
        """
        return (PLL2_MHZ / self.mod_freq_int[0],
                PLL2_MHZ / self.mod_freq_int[1])

    @property
    def tclk_mod_us(self) -> Tuple[float, float]:
        """ He Ren, 8/13/2021
        """
        return 1 / self.fclk_mod_mhz[0], 1 / self.fclk_mod_mhz[1]

    # downstream frame settings
    @property
    def min_frm_length(self) -> int:
        """
        20210623 He feedback for next version of datasheet:
        min_sfrm_length cannot be less than SFRM_TH
        SFRM_TH = ceil((1+INTE_BURST_LENGTH_Fx + LASER_PREHEAT_Fx)
                       * (NPULSE_GROUP_Fx + 60)
                       * T_clk_mod / T_line) + 10 + 3 * NROW)

        Joe added 5 on 11/29/20 as this was the number of additional values
        necessary to operate in the linear regime, otherwise
        the subframe timing gets clamped to a minimum value and this messes up
        timing sync; the subframe duration would, in practice, be higher
        than anticipated in the event of clamping
        """
        min_frm_len = 5 + int(max(
            (np.ceil(
                (1 + self.inte_total_burst_length[f] + LASER_PREHEAT)
                * (self.npulse[f] + self.dpulse[f] + 60)
                * (self.tclk_mod_us[f] / TLINE_US))
             + 10 + 3 * self.roi_rows)
            for f in (0, 1)))
        return min_frm_len

    @property
    def sub_frm_line_num(self) -> int:
        """SUB_FRM_LINE_NUM = N_ROW - 2 - EBD_LINE
        These registers are set according to Pleco datasheet V0.2, pg. 68
        The examples seem to treat "EBD_LINE" as the requested number
        of embedded data lines, NOT the value of the register
        """
        return self.roi_rows - 0  # Does NOT work with - 2

    @property
    def rd_line_max(self) -> int:
        """ RD_LINE_MAX = SUB_FRM_LINE_NUM * SUBFRM_NUM
        + (SUBFRM_NUM-1) * 4 + EBD_LINE
        """
        return (self.sub_frm_line_num * self.pleco_mode.n_subframes
                + (self.pleco_mode.n_subframes - 1) * 4 + 0)

    @property
    def mipi_max_line(self) -> int:
        return self.sub_frm_line_num * self.pleco_mode.n_subframes

    @property
    def data_pix_num(self):
        return N_COLS * 3 // 8 - 1

    @property
    def dummy_time_us(self):
        return (self.mod_freq_int[0] * (NPULSE + DPULSE) * DUMMY_BURSTS / 1e3,
                self.mod_freq_int[1] * (NPULSE + DPULSE) * DUMMY_BURSTS / 1e3)

    def __str__(self):
        return dedent(f"""
        --- laser settings ---
        npulse: {self.npulse}
        dpulse: {self.dpulse}
        dlaser: {self.dlaser}
        bursts: {self.inte_burst_length}
        las freq 0: {self.mod_freq_int[0].clk_freq_hz / 3 / 1e6:.1f} [MHz]
        las freq 1: {self.mod_freq_int[1].clk_freq_hz / 3 / 1e6:.1f} [MHz]
        Frame time: {self.comp_frame_time_us() / 1e-3:.2f} [ms]
        Subframe Time: {self.t_subframe_us:.2f} [us]
        Dummy Time f0: {self.dummy_time_us[0]}
        Dummy Time f1: {self.dummy_time_us[1]}
        Inte Time sec, set : {self.inte_time_s}
        Inte State us, calc : {self.inte_state_us:.6f}
        """)


class Itof(Device):
    """A class to define configuration and control of the
    Gpixel GTOF0503 ITOF sensor.
    """
    def __init__(self, bus: 'I2CBus',
                 device_addr: int, fpga_dbg: FpgaDbg,
                 itof_spi_map: 'MemoryMapPeriph', sensor_type: str):
        self.addr_bigendian = True
        self.data_bigendian = True
        super().__init__(bus, device_addr, 2, 1,
                         PLECO_MEMORY_MAP.pleco,
                         addr_bigendian=self.addr_bigendian,
                         data_bigendian=self.data_bigendian)
        self.itof_spi = itof_spi_map
        self.fpga_dbg = fpga_dbg
        self.sensor_type = sensor_type

    def disable(self):
        """Trigger itof one last time to get it out of any mode
        its in
        """
        self.apply_frame_settings(FrameSettings(0, 20, n_frames_capt=1))
        self.soft_trigger()

    def reset(self):
        """Resets the internal state machine of the itof this must be
        followed by a re-setup and trigger
        """
        self.write_fields(srst_n=0)
        time.sleep(0.1)
        self.write_fields(srst_n=1)
        self.setup()
        self.disable()

    def connect(self):
        self.itof_spi.register_write_callback(self.i2c.write)
        self.itof_spi.register_read_callback(self.i2c.read)
        self.itof_spi.register_readdata_callback(lambda x: x)
        self.periph.register_write_callback(self._itof_spi_write)
        self.periph.register_read_callback(self._itof_spi_read)
        self.periph.register_readdata_callback(lambda x: x)

    def read_fields(self, *args, use_mnemonic: bool = False):
        """This modifies the typical read_fields call due to an
        an error of which the cause is unknown.
        Read fields is appending a 1 at the MSB and returning a tuple

        This is to work around then instances when a tuple is returned.
        """
        rdata = self.periph.read_fields(*args, use_mnemonic=use_mnemonic)

        try:
            if isinstance(rdata, str):
                return rdata
            return rdata[0]
        except TypeError:
            return rdata

    def setup(self):
        # Initial setup
        self.fpga_dbg.write_fields(itof_reset_b=1)

        # Ensures that LVDS lines do not go high-z during idle phase.
        # repeated below but apply earlier.
        self.write_fields(laser_high_z_idle=0)

        self.write_fields(group_hold=1)
        self._select_mipi_lanes(4)
        self.write_fields(inte_laser_state_ir=0,  # laser off during ImageMode
                          apc_en=0,  # Laser APC. Pleco page 64
                          so_freq_en=0,  # default to hardware trigger
                          adc_error=4452,
                          test_bias_continue=0,
                          ld_xemo_tdig_out=1,
                          i_ramp_ota=9,
                          i_ramp_bias=99,
                          ldo_ctrl_en=80,
                          frm_num_lo=1,
                          frm_num_hi=0,
                          )

        self.write_fields(flip_v=1, flip_h=1)

        self._itof_spi_write(569, 1)  # need to bare write (antijam)
        self._itof_spi_write(580, 177)  # need to bare write (antijam)
        self._itof_spi_write(599, 99)  # need to bare write
        self._itof_spi_write(600, 80)  # need to bare write
        self._itof_spi_write(608, 244)  # need to bare write
        self._itof_spi_write(609, 26)  # need to bare write
        self._itof_spi_write(611, 90)  # need to bare write
        self._itof_spi_write(612, 12)  # need to bare write

        # enables monitor pin LD_APC_TDIG0_OUT integration state gating
        self._itof_spi_write(33, 17)  # need to bare write

        # Ensures that LVDS lines do not go high-z during idle phase.
        self.write_fields(laser_high_z_idle=0)

        self.write_fields(vdrn_low=42515,
                          vramp_st_lo=43,  # must override vramp defaults
                          vramp_st_hi=41,
                          vsg_m_bg_adjust=13,
                          vtg_m_bg_adjust=0)

        ebd = {2: 0, 1: 1, 0: 2}
        self.write_fields(ebd_size_v=0,
                          win_eb_en=0)
        self._itof_spi_write(706, 0b1100 | ebd[0])  # sets edb mode correctly

        self.write_fields(af_vld_line=696)

        # Assume reading out only one row for now
        # rwin0 set in apply_cpi_settings
        self.write_fields(rwin1_l=0x0,
                          rwin1_s=0x0,
                          rwin2_l=0x0,
                          rwin2_s=0x0,
                          rwin3_l=0x0,
                          rwin3_s=0x0)

        # exposure control
        self.write_fields(deep_sleep_en=0)

        # ----- Begin calibration block -----
        # If the SPI flash hasn't been written with data yet, the returned
        # value will be 0xff. Here, if the value is larger than the field size,
        # we write a zero so we don't have bogus values for these registers.
        # see write_cal_fields

        laser_sync = LaserMgSync(1)
        self.write_fields(sync_laser_lvds_mg=laser_sync.field)

        # # 20230426 dlays will get written when range calibration is applied

        # doff defaults are wrong, apply here
        # bare write because there are other packed bits
        self._itof_spi_write(578, 178)  # need to bare write
        self._itof_spi_write(579, 107)  # need to bare write
        self.write_fields(pga_gain=10)
        self.write_fields(group_hold=0)

    def write_delay_fields(self,
                           laser_mg_sync: int,
                           dlay_mg_f0_coarse: int, dlay_mg_f0_fine: int,
                           dlay_laser_f0_coarse: int, dlay_laser_f0_fine: int,
                           dlay_mg_f1_coarse: int, dlay_mg_f1_fine: int,
                           dlay_laser_f1_coarse: int, dlay_laser_f1_fine: int):

        # checks on fields
        laser_sync = LaserMgSync(laser_mg_sync)
        dlay_mg_f0 = DelayNsBv.from_fields(dlay_mg_f0_coarse, dlay_mg_f0_fine)
        dlay_laser_f0 = DelayNsBv.from_fields(dlay_laser_f0_coarse,
                                              dlay_laser_f0_fine)
        dlay_mg_f1 = DelayNsBv.from_fields(dlay_mg_f1_coarse, dlay_mg_f1_fine)
        dlay_laser_f1 = DelayNsBv.from_fields(dlay_laser_f1_coarse,
                                              dlay_laser_f1_fine)

        if dlay_mg_f0 > 0 and dlay_laser_f0 > 0:
            raise ValueError('Freq 0 MG and laser delays cannot both be > 0')
        if dlay_mg_f1 > 0 and dlay_laser_f1 > 0:
            raise ValueError('Freq 1 MG and laser delays cannot both be > 0')

        # writes the nominal calibration for mod_int (6, 5)
        self.write_fields(group_hold=1)
        self.write_fields(
            sync_laser_lvds_mg=laser_sync.field,
            dlay_mg_f0_coarse=dlay_mg_f0_coarse,
            dlay_mg_f0_fine=dlay_mg_f0_fine,
            dlay_laser_f0_coarse=dlay_laser_f0_coarse,
            dlay_laser_f0_fine=dlay_laser_f0_fine,
            dlay_mg_f1_coarse=dlay_mg_f1_coarse,
            dlay_mg_f1_fine=dlay_mg_f1_fine,
            dlay_laser_f1_coarse=dlay_laser_f1_coarse,
            dlay_laser_f1_fine=dlay_laser_f1_fine,
        )
        self.write_fields(group_hold=0)

    def write_shrink_expand_fields(self,
                                   nov_sel_laser_f0_shrink: int,
                                   nov_sel_laser_f0_expand: int,
                                   nov_sel_laser_f1_shrink: int,
                                   nov_sel_laser_f1_expand: int,
                                   ):
        self.write_fields(group_hold=1)
        self.write_fields(
            nov_sel_laser_f0_shrink=nov_sel_laser_f0_shrink,
            nov_sel_laser_f0_expand=nov_sel_laser_f0_expand,
            nov_sel_mg_f0_shrink=0,
            nov_sel_laser_f1_shrink=nov_sel_laser_f1_shrink,
            nov_sel_laser_f1_expand=nov_sel_laser_f1_expand,
            nov_sel_mg_f1_shrink=0,
        )
        self.write_fields(group_hold=0)

    def apply_frame_settings(self, frame: FrameSettings):
        """Legacy method to apply settings directly to the gToF without
        relying on the FPGA scan controller.

        According to He at Gpixel, rwin0_s should be
        rwin0_s = frame.roi_rows + 4 - 2
        with no embedded data lines.
        We are not subtracting 2
        """

        self.write_fields(group_hold=1)

        low, high = frame.n_frames_capt.fields
        self.write_fields(frm_num_lo=low, frm_num_hi=high)

        for freq in (0, 1):
            self.write_fields(
                # mod_freq_int value is reduced by 3 as pleco map is
                # {0: 3, 1: 4, ...}
                **{f'mod_freq{freq}_opt': frame.mod_freq_int[freq].field,
                   f'laser_preheat_length_f{freq}': LASER_PREHEAT,
                   f'dum_preheat_length_f{freq}': DUMMY_PREHEAT,
                   f'inte_burst_length_f{freq}': frame.inte_burst_length[freq],
                   # Sets dummy_bf to zero
                   f'inte_total_burst_length_f{freq}':
                       frame.inte_total_burst_length[freq],
                   }
            )

            self.write_fields(**{
                f'dpulse_f{freq}': frame.dpulse[freq].value,
                f'npulse_f{freq}_lo': frame.npulse[freq].fields[0],
                f'npulse_f{freq}_hi': frame.npulse[freq].fields[1],
                f'dlaser_off_group_f{freq}': frame.dlaser[freq].value
            })

        self.write_fields(mod_opt=frame.pleco_mode.kind,
                          bin_mode=0,
                          rwin0_s=frame.start_row.field,
                          rwin0_l=frame.roi_rows.field,
                          cwin0_s=8,
                          cwin0_s_div8=8,
                          cwin0_l_div8=N_COLS,
                          min_frm_length=frame.min_frm_length,
                          sub_frm_line_num=frame.sub_frm_line_num,
                          rd_line_max=frame.rd_line_max,
                          mipi_max_line=frame.mipi_max_line,
                          data_pix_num=frame.data_pix_num,
                          )

        self.write_fields(group_hold=0)

    def soft_trigger(self, check_limits=True):
        if check_limits:
            self._check_valid_timing()

        # switch into soft trigger
        self.write_fields(so_freq_en=1)
        self.write_fields(freq_trig=0)
        self.write_fields(freq_trig=1)
        self.write_fields(freq_trig=0)

        # switch back to hard trigger
        self.write_fields(so_freq_en=0)

    def _check_valid_timing(self):
        log.debug('checking valid timing')
        for f in (0, 1):
            mod_int = ModFreqIntOv.from_field(
                self.read_fields(f'mod_freq{f}_opt'))
            n_burst = self.read_fields(f'inte_burst_length_f{f}')
            npulse_lo = self.read_fields(f'npulse_f{f}_lo')
            npulse_hi = self.read_fields(f'npulse_f{f}_hi')
            npulse = NPulseOv.from_fields(npulse_lo, npulse_hi)
            dpulse = self.read_fields(f'dpulse_f{f}')
            dlaser = self.read_fields(f'dlaser_off_group_f{f}')

            try:
                duty_cycle = (npulse - dlaser) / (npulse + dpulse)
            except ZeroDivisionError:
                return
            las_active_inte_time = ((npulse - dlaser)
                                    * n_burst * mod_int.clk_period_s)

            if duty_cycle > MAX_LOCAL_DC:
                raise RuntimeError(
                    'Cannot trigger pleco. '
                    f'Duty cycle is {duty_cycle * 100:.1f} %, '
                    f'for freq. {f} but the limit is 50%')

            if las_active_inte_time > InteTimeSBv.LIMITS[1]:
                raise RuntimeError(
                    'Cannot trigger pleco. Integration time is '
                    f'{las_active_inte_time / 1e-6:.1f} us for freq. {f}, '
                    f'but the max is {InteTimeSBv.LIMITS[1] / 1e-6:.1f} us.'
                )

            if n_burst <= 0:
                raise RuntimeError(
                    f'Cannot trigger pleco. Number of bursts must be > 0, but '
                    f'was found to be {n_burst} for freq. {f}')

    def _select_mipi_lanes(self, n_lanes):
        if n_lanes == 2:
            # Settings for 2-lane MIPI
            # Set up PLL3 for 1.6Gbps
            self.write_fields(pll3_div_n_lo=200,
                              pll3_div_n_hi=0,
                              pll3_div_i='1/3',
                              pll3_div_b='1/1',
                              lane_num='2x')

            # NOTE WARN 20210604 update from He says leave addresses
            # 634, 635, 636 that control PLL1 at default value

            self.write_fields(dphy_p0_tx_time_t_lpx=10,
                              dphy_p0_tx_time_t_clk_prepare=8,
                              dphy_p0_tx_time_t_clk_zero=52,
                              dphy_p0_tx_time_t_clk_pre=0,
                              dphy_p0_tx_time_t_hs_prepare=9,
                              dphy_p0_tx_time_t_hs_zero=20,
                              dphy_p0_tx_time_t_hs_sot=0,
                              dphy_p0_tx_time_t_hs_eot=1,
                              dphy_p0_tx_time_t_clk_eot=12,
                              dphy_p0_tx_time_t_clk_post=17,
                              phya_xstb_d3_config=0,
                              phya_xstb_d2_config=0)
        elif n_lanes == 4:
            # # Settings for 4-lane MIPI
            # # Set up PLL3 to 480 MHz
            self.write_fields(pll3_div_n_lo=240,
                              pll3_div_n_hi=0,
                              pll3_div_i='1/3',
                              pll3_div_b='1/2',
                              lane_num='4x')

            # NOTE WARN 20210604 update from He says leave addresses
            # 634, 635, 636 that control PLL1 at default value

            self.write_fields(dphy_p0_tx_time_t_lpx=6,
                              dphy_p0_tx_time_t_clk_prepare=6,
                              dphy_p0_tx_time_t_clk_zero=30,
                              dphy_p0_tx_time_t_clk_pre=0,
                              dphy_p0_tx_time_t_hs_prepare=5,
                              dphy_p0_tx_time_t_hs_zero=12,
                              dphy_p0_tx_time_t_hs_sot=0,
                              dphy_p0_tx_time_t_hs_eot=8,
                              dphy_p0_tx_time_t_clk_eot=8,
                              dphy_p0_tx_time_t_clk_post=12,
                              phya_xstb_d3_config=1,
                              phya_xstb_d2_config=1)
        else:
            raise ValueError('Number of MIPI lanes in pleco.select_mipi_lanes'
                             f'must be 2 or 4 but was {n_lanes}')

    #  ######## ITOF SPI methods ######
    def _itof_send_cmd_and_block(self, spi_control_word):
        cnt_before = self.itof_spi.read_fields('spi_cnt')
        self.itof_spi.write_fields(spi_control_word=spi_control_word)

        # Check that the transaction happened
        while True:
            cnt_after = self.itof_spi.read_fields('spi_cnt')
            if ((cnt_after - cnt_before) % 16) == 1:
                break

    def _itof_spi_write(self, addr: int, data: int):
        """Write to GPixel ITOF through FPGA SPI

        payload = {PTR[15:8], PTR[7:0], DATA[7:0]} <- LSB
        """
        val = 0
        # Select clock rate defaults to 20Mhz
        val |= 4 << 24
        # Build the SPI frame separately for returning
        frame = 0
        frame |= (addr & 0xffff) << 8
        frame |= data & 0xff
        val |= frame
        self._itof_send_cmd_and_block(val)

    def _itof_spi_read(self, addr: int) -> int:
        """Read to GPixel ITOF through FPGA SPI
        """
        val = 0
        # Select clock rate defaults to 20Mhz
        val |= 4 << 24
        # Set to read
        val |= 1 << 31
        frame = 0
        frame |= (addr & 0xffff) << 8
        val |= frame
        self._itof_send_cmd_and_block(val)
        return self.itof_spi.read_fields('spi_rx_data') & 0xFF
