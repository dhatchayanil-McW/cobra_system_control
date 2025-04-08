"""
file: sensor_head.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file defines the SensorHead class and all it's
functionality. This class and it's methods are the
main interaction points with the M30 through
the API.
"""
from enum import Enum
import json
from numbers import Number
import time
from typing import Optional, Sequence, Tuple, Union

import Pyro5.api
import numpy as np

import cobra_system_control.exceptions as cobex
from cobra_system_control.calibration_data import (
    CalBase, CalDataEncoder, CalData,
    )
from cobra_system_control.cobra_log import log
from cobra_system_control.dacs import ItoDac, LcmVDac
from cobra_system_control import fe_ctl
from cobra_system_control.fpga_misc import FpgaDbg, ISP
from cobra_system_control.itof import (
    Itof, FrameSettings, N_ROWS, N_COLS)
from cobra_system_control.laser import (
    LaserCiDac, LaserVldaDac,
    ci_max_by_system,
    LaserPowerPercentMappedOvFactory,
)
from cobra_system_control.metadata import (
    MetadataBuffer, PerVirtualSensorMetadata, StaticMetadata, NUM_VIRTUAL_SENSOR,
)
from cobra_system_control.metasurface import LcmAssembly, LcmController
from cobra_system_control.pixel_mapping import (
    PixelMapping, DEFAULT_PIXEL_MAPPING, SUPERSAMPLED_PIXEL_MAPPING)
from cobra_system_control.pixel_mask import create_default_pixel_mask
from cobra_system_control.random_access_scanning import (
    RandomAccessScanning, InteTimeIdxMappedOv, NnLevelOv,
    FrameRateOv, MaxRangeIdxMappedOv,
    STRIPE_MODE_FLAGS, STRIPE_MODE_SUMMED_FLAGS, DspMode,
)
from cobra_system_control.roi_mapping import RoiMapping
from cobra_system_control.scan_control import (
    ScanParams, Scan, SnrThresholdBv, BinningOv,
    ScanTable, SCAN_TABLE_SIZE,
)
from cobra_system_control.spi_flash import (
    SpiFlash, sectors_from_size, reads_from_size, ba_to_pages)
from cobra_system_control.state import State, state_transition
import cobra_system_control.w25q32jw_const as wb

from cobra_system_control.functional_utilities import get_common_length
from cobra_system_control.validation_utilities import is_in_bounds, cast_to_sequence
from cobra_system_control.numerical_utilities import SignalVec


SYSINT = {'m30': 3}

DEFAULT_RTD_ALGORITHM_COMMON = 0b1100
DEFAULT_RTD_ALGORITHM_GRID_MODE = 0b110
DEFAULT_RTD_ALGORITHM_STRIPE_MODE = 0b00
DEFAULT_DSP_MODE = DspMode.CAMERA_MODE

DEFAULT_FRAME_RATE = 960
DEFAULT_INTE_TIME_US = 15
DEFAULT_ROI_ROWS = 8

DEFAULT_HDR_THRESHOLD = 4095
DEFAULT_HDR_LASER_POWER_PERCENT = 25
DEFAULT_HDR_INTE_TIME_US = 1


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class SensorHead:
    """A class to collect and have master control over the peripherals
    on the Sensor Head.
    """
    def __init__(self, *,  # star to make the rest required kwargs
                 whoami: str,
                 compute_platform: 'ComputePlatform',
                 debug: FpgaDbg,
                 isp: ISP,
                 ito_dac: 'ItoDac',
                 itof: Itof,
                 laser_ci_dac: LaserCiDac,
                 cmb_laser_vlda_dac: LaserVldaDac,
                 sh_laser_vlda_dac: LaserVldaDac,
                 cmb_lcm_v_dac: LcmVDac,
                 lcm_ctrl: LcmController,
                 metabuff: MetadataBuffer,
                 scan: Scan,
                 scan_params: ScanParams,
                 spi_flash: SpiFlash,
                 fpga_adc: 'FpgaAdc',
                 fpga_field_funcs: 'FpgaFieldFuncs',
                 ):
        self._whoami = whoami

        self.cal_data_class = CalData

        self._state = State.INITIALIZED

        self._compute_platform = compute_platform
        self._debug = debug
        self._fpga_field_funcs = fpga_field_funcs
        self._fpga_adc = fpga_adc
        self._isp = isp
        self._itof = itof
        self._ito_dac = ito_dac
        self._laser_ci_dac = laser_ci_dac
        self._cmb_laser_vlda_dac = cmb_laser_vlda_dac
        self._sh_laser_vlda_dac = sh_laser_vlda_dac
        self._cmb_lcm_v_dac = cmb_lcm_v_dac
        self._lcm_ctrl = lcm_ctrl
        self._metabuff = metabuff
        self._scan = scan
        self._scan_params = scan_params
        self._spi_flash = spi_flash

        # Must ensure that ito_dac is listed before
        # the fpga_adc to ensure that VREF is configured
        # to be an output before the ADC is calibrated.
        self.periphs = [
            self.debug,
            self.lcm_ctrl,
            self.ito_dac,
            self.itof,
            self.cmb_laser_vlda_dac, self.sh_laser_vlda_dac,
            self.laser_ci_dac, self.cmb_lcm_v_dac,
            self.isp, self.metabuff, self.scan, self.scan_params,
            self.spi_flash.qspi,
            self.fpga_adc,
        ]

        self._random_access_scan = None
        self.laser_power_percent_mapped_ov = None
        self.ci_max = None
        self._rx_pcb_rev = None
        self._calibration_version = "not found"

        # from calibration data, loaded during apply_calibration
        self._cal_data_path = None
        self._mapping_table_path = None
        self._roi_mapping = None
        self._pixel_mapping = None
        self._super_pixel_mapping = None
        self._cal_data = None
        self.sensor_sn = None
        self._sensor_prefix = None
        self._lcm_assembly = LcmAssembly()

        self.aggregate: bool = False

        self.dac_settle_us = self.laser_ci_dac.dac.dac_settle_us

        self._db_sensor_configuration = {}

        # These values are cached during apply_calibration() so they
        # do not need to be recalculated during apply_settings()
        # Initialize to zero
        self.range_cal_offset_mm_lo_0807 = 0
        self.range_cal_offset_mm_hi_0807 = 0
        self.range_cal_mm_per_volt_lo_0807 = 0
        self.range_cal_mm_per_volt_hi_0807 = 0
        self.range_cal_mm_per_celsius_lo_0807 = 0
        self.range_cal_mm_per_celsius_hi_0807 = 0
        self.range_cal_offset_mm_lo_0908 = 0
        self.range_cal_offset_mm_hi_0908 = 0
        self.range_cal_mm_per_volt_lo_0908 = 0
        self.range_cal_mm_per_volt_hi_0908 = 0
        self.range_cal_mm_per_celsius_lo_0908 = 0
        self.range_cal_mm_per_celsius_hi_0908 = 0

        # These values are determined after FPGA ADC
        # calibration and can be cached values set during setup
        self.adc_cal_gain_fmt = SignalVec(False, 12, 19)
        self.adc_cal_offset_fmt = SignalVec(True, 12, 14)

        self._fe_rows = None
        self._fe_reduce_mode = None

        # Value to enable double buffering of scan param ram
        self.scan_ram_msb = 0

    @property
    def state(self):
        # it is possible for non-loopback mode to complete, without interacting
        # with the firmware, check the real system status if scanning
        if self._state is State.SCANNING and (self.scan.read_fields(
                'scan_state', use_mnemonic=True) == 'idle'):
            self._state = State.ENERGIZED
        return self._state

    def connect(self):
        for x in self.periphs:
            x.connect()

    def setup(self):
        """Perform one-time setup and apply default settings to put the system
        in a known good state.
        """
        for x in self.periphs:
            x.setup()
        self.sh_laser_vlda_dac.dac.dac_full_scale = 3.3

        self._rx_pcb_rev = self.config_rx_bd_rev()
        self.laser_ci_dac.set_ci_limit(self.whoami, self._rx_pcb_rev)
        self.ci_max = ci_max_by_system(self.whoami, self._rx_pcb_rev)
        laser_power_factory = LaserPowerPercentMappedOvFactory()
        self.laser_power_percent_mapped_ov = laser_power_factory(
            self.whoami, self._rx_pcb_rev)

        # Keep CI Low
        self.laser_ci_dac.set_voltage(0)

        # Turn on timestamping
        self.scan.write_fields(scan_tstamp_enable=1)
        # Turn on legacy timestamping
        self.scan.write_fields(tstamp_sync_aux_en=1)
        # Turn off external
        self.scan.write_fields(tstamp_trigger_ext_en=0)

        # Configure ITO
        self.ito_dac.set_voltage(-9)

        # apply calibration from spi flash
        # moved above triggering the junk frame to ensure
        # the mapping table data is available to frontend
        self.apply_calibration(self.get_cal_data())

        # moved DAC settle call from Scan
        self.scan.write_fields(
            dac_ci_settle_tc=self.fpga_field_funcs.getf_dac_settle_tc(
                self.laser_ci_dac.dac.dac_settle_us),
        )

        # safely trigger gtof junk frame
        # FE control was added to possibly keep the front end okay during
        # unittesting where this will result in a real frame
        # and reduce_mode isn't set correctly
        fe_ctl.fe_start_streaming(mode=0)
        self.itof.apply_frame_settings(FrameSettings(0, 8))
        self.itof.soft_trigger()
        fe_ctl.fe_stop_streaming()
        log.debug('triggered gToF junk frame')

        self.isp.write_fields(tx_raw16_en=1)
        self.isp.write_fields(roi_aggreg_cnt=0)
        self.isp.write_fields(tx_raw16_as_rgb888=1,
                              tx_swap_bytes=1)
        self.scan.write_fields(rowcal_adjust_en=0)
        # Make sure the pol toggle setting is correct
        self.lcm_ctrl.write_fields(pol_toggle=0)
        # Set the tp1 period for the 9v pattern apply
        self.lcm_ctrl.write_fields(settle_tc=9)

        # write itof fields not written by scan controller and may have
        # been written by itof apply_frame_settings
        self.itof.write_fields(
            cwin0_s=8,
            cwin0_s_div8=8,
            cwin0_l_div8=640,
            frm_num_lo=1,
            frm_num_hi=0,
        )

        self.isp.write_fields(
            quant_mode=0,
            reduce_mode=1,
        )

        # Set the ADC cal values before apply settings
        self.adc_cal_gain_fmt.set_float_vec(self.fpga_adc.cal_gain)
        self.adc_cal_offset_fmt.set_float_vec(self.fpga_adc.cal_offset)

        # apply default settings so the sensor is in a good state
        try:
            self.apply_random_access_scan_settings(
                angle_range=[[-45.0, 45.0, 1.0]],
                max_range_m=25.2)
        except cobex.ScanPatternValueError:
            # Potentially, there was range calibration data written for (9,8)
            # but not (8,7). If we don't check this, we'll get into a boot loop
            self.apply_random_access_scan_settings(
                angle_range=[[-45.0, 45.0, 1.0]],
                max_range_m=32.4)

        # Let the frontend know the calibration files are available
        fe_ctl.fe_reload_cal_data()

        # Repeat the ADC calibration after everything has warmed up a little
        self.fpga_adc.calibrate()
        # Re-up the values after recalibration
        self.adc_cal_gain_fmt.set_float_vec(self.fpga_adc.cal_gain)
        self.adc_cal_offset_fmt.set_float_vec(self.fpga_adc.cal_offset)

    def read_git_sha(
            self, golden_shas: Tuple[int, ...] = tuple()
    ) -> Tuple[int, bool]:
        """Read and return the Git SHA. Support reading of all released
        FPGA golden bitstreams in case the primary bitstream is borked.
        If a golden Git SHA is found, it is returned. Otherwise, the value
        from the standard read_fields('git_sha') call is returned.
        """
        is_golden = False
        gsha = self.debug.read_fields('git_sha')

        is_golden = is_golden or (gsha in golden_shas)
        if is_golden:
            log.debug('FPGA booted into golden image for sensor.')

        log.debug('Detected git sha 0x%x for sensor.', gsha)
        return gsha, is_golden

    def apply_random_access_scan_settings(
            self, *,
            angle_range: Sequence[Tuple[float, float, float]] = None,
            fps_multiple: Union[int, Sequence[int]] = 1,
            laser_power_percent: Union[int, Sequence[int]] = 100,
            inte_time_us: Union[int, Sequence[int]] = DEFAULT_INTE_TIME_US,
            max_range_m: Union[int, Sequence[int]] = 25.2,
            binning: Union[int, Sequence[int]] = 2,
            snr_threshold: Union[Number, Sequence[Number]] = 1.8,
            nn_level: Union[int, Sequence[int]] = 0,
            frame_rate_hz: Union[int, Sequence[int]] = DEFAULT_FRAME_RATE,

            dsp_mode: Union[int, Enum] = DEFAULT_DSP_MODE,
            rtd_algorithm_common: Union[int, Sequence[int]] = DEFAULT_RTD_ALGORITHM_COMMON,
            rtd_algorithm_grid_mode: Union[int, Sequence[int]] = DEFAULT_RTD_ALGORITHM_GRID_MODE,
            rtd_algorithm_stripe_mode: Union[int, Sequence[int]] = DEFAULT_RTD_ALGORITHM_STRIPE_MODE,

            user_tag: Union[int, Sequence[int]] = None,

            roi_rows: int = DEFAULT_ROI_ROWS,
            double_dip: bool = True,
            interleave: bool = False,

            hdr_threshold: Union[int, Sequence[int]] = DEFAULT_HDR_THRESHOLD,
            hdr_laser_power_percent: Union[int, Sequence[int]] = DEFAULT_HDR_LASER_POWER_PERCENT,
            hdr_inte_time_us: Union[int, Sequence[int]] = DEFAULT_HDR_INTE_TIME_US,
    ):
        """This takes inputs from the API and creates arguments to feed to
        apply_settings.

        Args:
            angle_range: Sequence of angle triplets to define angle ranges of
              each Virtual Sensor scan with definition [start, stop, step].
            fps_multiple: Frame rate multiple of each Virtual Sensor.
            laser_power_percent: Percent of max power to set laser to.
            inte_time_us: value for integration time from API.
            max_range_m: Modulation frequency pair from API.
            binning: binning level.
            snr_threshold: Pixels with SNR less than this are set to zero.
            nn_level: Nearest Neighbor filter setting.
            rtd_algorithm_common: Select which algorithms common to both modes
              to use in depth processing
            rtd_algorithm_grid_mode: Select which algorithms for grid mode to
              use in depth processing.
            rtd_algorithm_stripe_mode: Select which algorithms for stripe mode
              to use in depth processing.
            user_tag: Network tag for this iteration of settings.
            roi_rows: Number of rows to read out.
            double_dip: If double_dip is on, compatible ROIs will be shared
                between Virtual Sensors.
            interleave: If on, ROIs from various Virtual Sensors will be interleaved in time.
            hdr_threshold: Threshold value to trigger an HDR retry measurement.
        """
        if angle_range is None:
            angle_range = [[-45.0, 45.0, 1.0]]
        if isinstance(dsp_mode, int):
            dsp_mode = DspMode.from_key(dsp_mode)
        if dsp_mode == DspMode.LIDAR_MODE:
            rtd_algorithm_common |= 1
        try:
            self._random_access_scan = RandomAccessScanning(
                angle_range=angle_range,
                fps_multiple=fps_multiple,
                laser_power_percent=laser_power_percent,
                inte_time_us=inte_time_us,
                max_range_m=max_range_m,
                binning=binning,
                snr_threshold=snr_threshold, nn_level=nn_level,
                frame_rate_hz=frame_rate_hz,
                user_tag=user_tag, roi_rows=roi_rows,
                rtd_algorithm_common=rtd_algorithm_common,
                rtd_algorithm_grid_mode=rtd_algorithm_grid_mode,
                rtd_algorithm_stripe_mode=rtd_algorithm_stripe_mode,
                roi_mapping=self.roi_mapping,
                double_dip=double_dip,
                interleave=interleave,
                dsp_mode=dsp_mode,

                hdr_threshold=hdr_threshold,
                hdr_laser_power_percent=hdr_laser_power_percent,
                hdr_inte_time_us=hdr_inte_time_us,
                laser_power_mapped_cls=self.laser_power_percent_mapped_ov,
            )
        except Exception as e:
            log.error(e)
            raise e
        try:
            self.apply_settings(**self._random_access_scan.appset_dict)
        except Exception as e:
            log.error(e)
            raise e

    def apply_settings(
            self, *,
            angles: Optional[Sequence[Number]] = None,
            orders: Optional[Union[int, Sequence[int]]] = None,
            s_rows: Optional[Union[int, Sequence[int]]] = None,
            inte_time_s: Union[
                float, Sequence[float]] = InteTimeIdxMappedOv.MAP[
                    InteTimeIdxMappedOv.OPTIONS.index(DEFAULT_INTE_TIME_US)],
            ci_v: Union[Number, Sequence[Number]] = None,
            mod_freq_int: Union[
                Tuple[int, int],
                Sequence[Tuple[int, int]]] = MaxRangeIdxMappedOv.MAP[25.2],
            start_stop_flags: Union[int, Sequence[int]] = None,
            summed_rois: int = 0,
            virtual_sensor_bitmask: int = 0b1,

            loopback: bool = True,
            roi_rows: int = DEFAULT_ROI_ROWS,

            virtual_sensor_metadata: Union[PerVirtualSensorMetadata, Sequence[PerVirtualSensorMetadata]] = None,
            static_metadata: StaticMetadata = None,

            rtd_algorithm_common: Union[int, Sequence[int]] = DEFAULT_RTD_ALGORITHM_COMMON,
            rtd_algorithm_grid_mode: Union[int, Sequence[int]] = DEFAULT_RTD_ALGORITHM_GRID_MODE,
            rtd_algorithm_stripe_mode: Union[int, Sequence[int]] = DEFAULT_RTD_ALGORITHM_STRIPE_MODE,

            user_tag: Union[Number, Sequence[Number]] = 0xb3,
            binning: Union[Number, Sequence[Number]] = 2,
            snr_threshold: Union[Number, Sequence[Number]] = 1.8,
            nn_level: Union[int, Sequence[int]] = 0,

            test_mode: int = 0,
            hdr_threshold: int = DEFAULT_HDR_THRESHOLD,
            hdr_ci_v: Union[Number, Sequence[Number]] = None,
            hdr_inte_time_s: Union[float, Sequence[float]] = None,

            dsp_mode: int = DEFAULT_DSP_MODE,
            disable_network_stream: bool = False,
            zero_range_cal: bool = False,
            disable_range_temp_correction: bool = False,
            disable_rawtodepth: bool = False,

            # Default to maximum frame rate
            frame_rate_hz: Union[int, Sequence[int]] = DEFAULT_FRAME_RATE,
            ito_freq_mult: int = None,
            tp1_period_us: Optional[Tuple[float]] = (None, None),
            pol_cnt: Optional[Tuple[int]] = (None, None),
            scan_fetch_delay: int = None,
            scan_trigger_delay: int = None,
    ):
        """Applies all sensor head configuration settings

        This method is intended to be flexible enough for use across standard
        operating modes and calibration modes. This method takes either
        ``orders`` *and* ``s_rows`` as keyword argument inputs, *or* ``angles``
        as a keyword argument. If the latter is passed in, it will be
        converted into ``orders`` and ``s_rows`` using system calibrations
        in this method.

        At this time, ``orders``, ``s_row``, ``inte_time_s``, and ``ci_v``
        can be provided as equal-length sequences. The sequences will be used
        to populate the scan table in the order provided. ``roi_rows`` and
        ``mode`` are forward looking and not yet supported.

        Args:
            angles: virtual_sensor_ angle(s) used in roi selection and lcm steering. If
              this argument is supplied, ``orders`` and ``s_rows`` must be None
            orders: steering order(s). If this argument is supplied, s_rows
              must not be None, and ``angles`` must be None.
            s_rows: gToF starting row(s). If this argument is supplied,
            ``orders`` must not be None, and ``angles must be None.
            inte_time_s: gToF subframe integration time(s)
            ci_v: laser ci voltage(s)
            mod_freq_int: gToF modulation frequencies
            start_stop_flags: Flags to tell RTD when a new frame is starting
                and ending.
                [0]: first roi
                [1]: last roi
                [2]: sum this roi (will be deprecated)
                [3]: save summed rois (will be deprecated)
            summed_rois: Not renamed to not break legacy code.
                Summing not supported but saving to /run is
                Saving from R2D is still supported but saving using
                the frontend should be preferred.
            virtual_sensor_bitmask: Define which Virtual Sensor the ROI belongs to

            loopback: specifies whether to use the scan controller's loopback
            roi_rows: gToF number of rows in each ROI

            virtual_sensor_metadata: Container for metadata for each Virtual Sensor
            static_metadata: Container for static metadata

            rtd_algorithm_common: Select which algorithm is used in depth
                processing as defined in metadata_map.yml
            rtd_algorithm_grid_mode:
            rtd_algorithm_stripe_mode:
            user_tag: Value specified by user for identifying streams
            binning: Level of symmetric binning in
            snr_threshold: Threshold filter level on signal to background level
            nn_level: Nearest Neighbor filter level

            test_mode: used to specify test pattern emission from FPGA (see
                metadata memory map for details)
            hdr_threshold: Values above this will cause an ROI to be immediately
                recaptured with lower integration time and laser power.

            disable_network_stream: Stop data from being sent over the network
            zero_range_cal: Zero out all TOF registers related
                to range calibration to do phase calibration
            disable_range_temp_correction: Turn off range correction based on
                temperature in the DSP
            disable_rawtodepth: Turns off rawtodepth processing. Used during the
                saving of raw ROIs in engineering mode.

            ito_freq_mult: int = None,
            tp1_period_us: Tuple of TP1 period; if provided,
                           it overrides the default
            pol_cnt: Optional[Tuple[int]] = (None, None),
            scan_fetch_delay: int = None,
            scan_trigger_delay: int = None,

        CI limits determined through empirical testing.
        """
        if ci_v is None:
            ci_v = self.ci_max
        else:
            if isinstance(ci_v, (Sequence, np.ndarray)):
                ci_v = [min(self.ci_max, x) for x in ci_v]
            else:
                ci_v = min(self.ci_max, ci_v)

        if isinstance(dsp_mode, int):
            dsp_mode = DspMode.from_key(dsp_mode)

        # HDR values
        if hdr_ci_v is None:
            if self.laser_power_percent_mapped_ov is None:
                self._rx_pcb_rev = self.config_rx_bd_rev()
                laser_power_factory = LaserPowerPercentMappedOvFactory()
                self.laser_power_percent_mapped_ov = laser_power_factory(
                    self.whoami, self._rx_pcb_rev)
            hdr_ci_v = self.laser_power_percent_mapped_ov(
                DEFAULT_HDR_LASER_POWER_PERCENT).mapped
        if hdr_inte_time_s is None:
            hdr_inte_time_s = InteTimeIdxMappedOv(
                DEFAULT_HDR_INTE_TIME_US).mapped

        if roi_rows == 480:
            reduce_mode = 0
        else:
            reduce_mode = 1

        # # compute orders and s_row from provided angles
        if angles is not None and all(x is None for x in (orders, s_rows)):
            if isinstance(angles, Number):
                angles = np.asarray([angles])
            if isinstance(angles, (Sequence, np.ndarray)):
                angles = np.asarray(angles)
            if self.roi_mapping is None:
                raise cobex.CalibrationError(
                    'ROI map unavailable, cannot compute '
                    'orders and start rows from angles.')
            orders, s_rows = self.roi_mapping(
                angles=angles, roi_rows=roi_rows, trim_duplicates=True)
        # user provided orders and s_rows, nothing to do
        elif angles is None and all(x is not None for x in (orders, s_rows)):
            pass
        # user provided s_rows and needs help getting the orders
        elif angles is None and orders is None and s_rows is not None:
            if isinstance(s_rows, Number):
                s_rows = np.asarray([s_rows])
            if isinstance(s_rows, (Sequence, np.ndarray)):
                s_rows = np.asarray(s_rows)
            if self.roi_mapping is None:
                raise cobex.CalibrationError(
                    'ROI map unavailable, cannot compute orders from start rows.')

            # 20221117, if providing start rows, the duplicates should
            # not be trimmed.
            orders, s_rows = self.roi_mapping(
                s_rows=s_rows, roi_rows=roi_rows, trim_duplicates=False)

        # otherwise the user provided an incorrect combination, raise an error
        else:
            raise ValueError(
                'method requires either "angles" or "s_rows" '
                'or "s_rows" and "orders" as '
                'keyword arguments (but not all of them)')

        self._fe_rows = roi_rows
        self._fe_reduce_mode = reduce_mode

        # finds common length among allowed sequence-able parameters
        # throws error if two or more sequences of different lengths are input
        common_length = get_common_length(
            ci=ci_v, orders=orders, s_row=s_rows, roi_rows=roi_rows,
            inte_time=inte_time_s,
            fr=frame_rate_hz,
            rtdc=rtd_algorithm_common,
            rtdg=rtd_algorithm_grid_mode,
            rtds=rtd_algorithm_stripe_mode,
            ssf=start_stop_flags,
            b=binning, snr=snr_threshold, nn=nn_level,
            hdrc=hdr_ci_v, hdri=hdr_inte_time_s,
        )

        # convert any int / float to sequence with common length
        s_rows = cast_to_sequence(s_rows, common_length)
        roi_rows = cast_to_sequence(roi_rows, common_length)
        inte_time_s = cast_to_sequence(inte_time_s, common_length)
        virtual_sensor_bitmask = cast_to_sequence(virtual_sensor_bitmask, common_length)
        rtd_algorithm_common = cast_to_sequence(rtd_algorithm_common, common_length)
        rtd_algorithm_grid_mode = cast_to_sequence(rtd_algorithm_grid_mode, common_length)
        rtd_algorithm_stripe_mode = cast_to_sequence(rtd_algorithm_stripe_mode, common_length)

        # cast other sequences (performing limit checks)
        ci_v = cast_to_sequence(ci_v, common_length)
        orders = cast_to_sequence(
            orders, common_length,
            func=self.lcm_assembly.order_ov)
        binning = cast_to_sequence(binning, common_length, func=BinningOv)
        snr_threshold = cast_to_sequence(
            snr_threshold, common_length, func=SnrThresholdBv)
        nn_level = cast_to_sequence(nn_level, common_length, func=NnLevelOv)
        frame_rate_hz = cast_to_sequence(frame_rate_hz, common_length, func=FrameRateOv)

        hdr_ci_v = cast_to_sequence(hdr_ci_v, common_length)
        hdr_inte_time_s = cast_to_sequence(hdr_inte_time_s, common_length)

        # need to array mod_freq_int as well
        if isinstance(mod_freq_int[0], (int, float)):
            mod_freq_int = [tuple(mod_freq_int)] * len(orders)
        elif len(mod_freq_int) != len(orders):
            raise cobex.ScanPatternSizeError(
                'Length of modulation frequencies and scanning orders is not equal. '
                'Cannot cast to equal sequences')

        if len(set(mod_freq_int)) != 1:
            raise cobex.ScanPatternValueError(
                'All max unambiguous range indices must be the same but '
                f'the scan definition has {len(set(mod_freq_int))} different values')

        if zero_range_cal or (
                # This sensor may be fresh and have no range cal data
                # and we need to be able to handle this.
                # We'll write zeros if this is the case
                (not self._cal_data.range0807.is_valid)
                and (not self._cal_data.range0908.is_valid)
        ):
            self.itof.write_delay_fields(
                laser_mg_sync=0,
                dlay_mg_f0_coarse=0, dlay_mg_f0_fine=0,
                dlay_laser_f0_coarse=0, dlay_laser_f0_fine=0,
                dlay_mg_f1_coarse=0, dlay_mg_f1_fine=0,
                dlay_laser_f1_coarse=0, dlay_laser_f1_fine=0,
            )
            self.itof.write_shrink_expand_fields(
                nov_sel_laser_f0_shrink=0, nov_sel_laser_f0_expand=1,
                nov_sel_laser_f1_shrink=0, nov_sel_laser_f1_expand=1,
            )
        else:
            # What modulation frequency set are we using?
            mf = mod_freq_int[0]
            mfstr = f'{mf[0]:02.0f}{mf[1]:02.0f}'
            cal_grp = getattr(self._cal_data, f'range{mfstr}')
            if cal_grp.is_valid:
                self.itof.write_delay_fields(
                    laser_mg_sync=cal_grp.sync_laser_lvds_mg.vdig[0],
                    dlay_mg_f0_coarse=cal_grp.dlay_mg_f0_coarse.vdig[0],
                    dlay_mg_f0_fine=cal_grp.dlay_mg_f0_fine.vdig[0],
                    dlay_laser_f0_coarse=cal_grp.dlay_laser_f0_coarse.vdig[0],
                    dlay_laser_f0_fine=cal_grp.dlay_laser_f0_fine.vdig[0],
                    dlay_mg_f1_coarse=cal_grp.dlay_mg_f1_coarse.vdig[0],
                    dlay_mg_f1_fine=cal_grp.dlay_mg_f1_fine.vdig[0],
                    dlay_laser_f1_coarse=cal_grp.dlay_laser_f1_coarse.vdig[0],
                    dlay_laser_f1_fine=cal_grp.dlay_laser_f1_fine.vdig[0],
                )
                self.itof.write_shrink_expand_fields(
                    nov_sel_laser_f0_shrink=cal_grp.pw_laser_f0_shrink.vdig[0],
                    nov_sel_laser_f0_expand=cal_grp.pw_laser_f0_expand.vdig[0],
                    nov_sel_laser_f1_shrink=cal_grp.pw_laser_f1_shrink.vdig[0],
                    nov_sel_laser_f1_expand=cal_grp.pw_laser_f1_expand.vdig[0],
                )
            else:
                raise cobex.ScanPatternValueError(
                    'The sensor does not have valid calibration data for this '
                    'max unambiguous range m setting. '
                    'Please try the other value')

        # make sequence of frame settings (performs limit checks)
        frames = tuple(
            FrameSettings(
                s, r, inte_time_s=i, hdr_inte_time_s=hdri,
                mod_freq_int=mf,
            )
            for (s, r, i, hdri, mf) in zip(
                    s_rows, roi_rows,
                    inte_time_s,
                    hdr_inte_time_s,
                    mod_freq_int,
            ))

        # configure static scan controller config
        self.scan.write_fields(
            scan_loopback=int(loopback),
        )

        try:
            _adc_cal_gain = self.adc_cal_gain_fmt.get_dig_vec()[0]
            _adc_cal_offset = self.adc_cal_offset_fmt.get_dig_vec()[0]
        except TypeError:
            # Calling apply_settings() after connect() without a setup()
            _adc_cal_gain = 0
            _adc_cal_offset = 0

        static_metadata = static_metadata or StaticMetadata(
            rtd_output=int(disable_network_stream),
            reduce_mode=reduce_mode,
            sensor_sn=self.sensor_sn,
            test_mode=test_mode,
            quant_mode=0,
            mipi_raw_mode=1,
            hdr_threshold=hdr_threshold,
            system_type=SYSINT[self.whoami],
            rx_pcb_type=self.rx_pcb_rev or 0,
            tx_pcb_type=0,
            lcm_type=2,
            range_cal_offset_mm_lo_0807=self.range_cal_offset_mm_lo_0807,
            range_cal_offset_mm_hi_0807=self.range_cal_offset_mm_hi_0807,
            range_cal_mm_per_volt_lo_0807=self.range_cal_mm_per_volt_lo_0807,
            range_cal_mm_per_volt_hi_0807=self.range_cal_mm_per_volt_hi_0807,
            range_cal_mm_per_celsius_lo_0807=self.range_cal_mm_per_celsius_lo_0807,
            range_cal_mm_per_celsius_hi_0807=self.range_cal_mm_per_celsius_hi_0807,
            range_cal_offset_mm_lo_0908=self.range_cal_offset_mm_lo_0908,
            range_cal_offset_mm_hi_0908=self.range_cal_offset_mm_hi_0908,
            range_cal_mm_per_volt_lo_0908=self.range_cal_mm_per_volt_lo_0908,
            range_cal_mm_per_volt_hi_0908=self.range_cal_mm_per_volt_hi_0908,
            range_cal_mm_per_celsius_lo_0908=self.range_cal_mm_per_celsius_lo_0908,
            range_cal_mm_per_celsius_hi_0908=self.range_cal_mm_per_celsius_hi_0908,
            adc_cal_gain=_adc_cal_gain,
            adc_cal_offset=_adc_cal_offset,
        )

        # Write metadata buffer with arrayed virtual_sensor_metadata
        if virtual_sensor_metadata is not None:
            if isinstance(virtual_sensor_metadata, (list, tuple, np.ndarray)):
                # virtual_sensor_metadata is already iterable
                pass
            elif isinstance(virtual_sensor_metadata, PerVirtualSensorMetadata):
                log.debug('PerVirtualSensorMetadata defined for Virtual Sensor 0')
                virtual_sensor_metadata = [PerVirtualSensorMetadata]
            else:
                raise cobex.PerVirtualSensorMetadataError(
                    f'Defined type of virtual_sensor_metadata is {type(virtual_sensor_metadata)}')
        else:
            # The Virtual Sensor metadata were not defined through an
            # apply_random_access_scanning() call so some of the sequenced parameters
            # will all be the same.
            if dsp_mode == DspMode.CAMERA_MODE:
                # max(roi_rows) for now.
                # +4 added to account for row cal
                n_rows = min(480, max(s_rows) - min(s_rows) + max(roi_rows) + 4)
                n_rois = len(orders)
                vs_s_rows = max(0, min(s_rows) - 2)
            elif dsp_mode == DspMode.LIDAR_MODE:
                n_rows = DEFAULT_ROI_ROWS
                n_rois = 1
                vs_s_rows = max(0, min(s_rows))

            # rtd_algorithm bits are defined in metadata_map.yml
            if disable_range_temp_correction:
                rtd_algorithm_common = list(np.asarray(rtd_algorithm_common) & ~(1 << 3))
            if disable_rawtodepth:
                rtd_algorithm_common = list(np.asarray(rtd_algorithm_common) | (1 << 11))
            if dsp_mode == DspMode.LIDAR_MODE:
                rtd_algorithm_common = list(np.asarray(rtd_algorithm_common) | (1 << 0))

            virtual_sensor_metadata = PerVirtualSensorMetadata.empty_array()
            # Binning is now arrayed so we need to pick just one.
            # Added a -2 to srow to account for row cal.
            virtual_sensor_metadata[0] = PerVirtualSensorMetadata.build(
                user_tag, binning[0],
                vs_s_rows, n_rows, n_rois,
                rtd_algorithm_common[0],
                rtd_algorithm_grid_mode[0], rtd_algorithm_stripe_mode[0],
                snr_threshold[0],
                nn_level[0],
            )

        ci_fields_unshifted = [self.laser_ci_dac.field_unshifted_from_voltage(x)
                               for x in ci_v]
        hdr_ci_fields_unshifted = [self.laser_ci_dac.field_unshifted_from_voltage(x)
                                   for x in hdr_ci_v]

        start_stop_flags = make_start_stop_flag_array(
            start_stop_flags, len(orders), dsp_mode, summed_rois)

        # create and write scan table
        scan_table = ScanTable.build(
            self.fpga_field_funcs,
            orders,
            ci_fields_unshifted,
            hdr_ci_fields_unshifted,
            frames, virtual_sensor_bitmask,
            binning, frame_rate_hz,
            start_stop_flags,
            ito_freq_mult,
            pol_cnt,
            tp1_period_us,
            scan_fetch_delay, scan_trigger_delay,

        )

        # Write the scan table, then stop scanning to write the metadata buffer
        # This allow apply_settings() to be called when the system is scanning
        self.write_scan_table(scan_table)
        self.stop(stop_fe_streaming=False)

        # One could consider double buffering the metadata ram in the
        # future as well.
        self.metabuff.write_metadata_buffer_virtual_sensor_data(
            virtual_sensor_metadata)
        self.metabuff.write_metadata_buffer_static(static_metadata)
        # configure isp controller with static metadata
        self.isp.write_fields(
            reduce_mode=reduce_mode,
            hdr_sat_limit=hdr_threshold,
            test_mode=test_mode)

        # Populate the db_sensor_configuration dictionary
        self._db_sensor_configuration['binning'] = virtual_sensor_metadata[0].binning
        self._db_sensor_configuration['snr_threshold'] = virtual_sensor_metadata[0].snr_threshold
        self._db_sensor_configuration['rtd_algorithm_common'] = virtual_sensor_metadata[0].rtd_algorithm_common
        self._db_sensor_configuration['rtd_algorithm_grid_mode'] = virtual_sensor_metadata[0].rtd_algorithm_grid_mode
        self._db_sensor_configuration['rtd_algorithm_stripe_mode'] = virtual_sensor_metadata[0].rtd_algorithm_stripe_mode
        self._db_sensor_configuration['nn_level'] = virtual_sensor_metadata[0].nn_level
        self._db_sensor_configuration['reduce_mode'] = static_metadata.reduce_mode
        self._db_sensor_configuration['hdr_threshold'] = static_metadata.hdr_threshold
        return scan_table

    def write_scan_table(self, scan_table):
        """Writes the scan param memory using the ScanParam peripheral.

        The scan param memory is 1024 ROIs. To access writing to the second
        chunk of 512, the scan_ram_addr_msb register must be asserted in
        the Scan peripheral.
        """
        future_scan_ram_partition = int(not self.scan_ram_msb)
        # Set the MSB of the Scan RAM to the unused portion
        self.scan.write_fields(scan_ram_addr_msb=future_scan_ram_partition)
        log.debug('Writing the Scan table to partition %s',
                  future_scan_ram_partition)
        self.scan_params.write_scan_table(scan_table)
        self.scan_ram_msb = future_scan_ram_partition

    @property
    def valid_scan_table_pointer_range(self) -> Optional[Tuple[int, int]]:
        """Gets the valid pointers from the ScanParam periph and
        scales them by the scan param memory partition being
        used
        """
        if self.scan_params.scan_table.valid_ptr_range is None:
            return None
        else:
            return tuple([x + self.scan_ram_msb * SCAN_TABLE_SIZE
                          for x in self.scan_params.valid_ptr_range])

    def apply_calibration(self, cal_data: 'CalData'):
        """Applies both sensor specific and common calibrations
        """
        # --- Version ---
        if cal_data.cal_version.is_valid:
            if self.is_compatible_calibration_version(cal_data):
                log.debug('Calibration version %s.%s.%s and System version %s are compatible',
                          cal_data.cal_version.major_version.vfxp[0],
                          cal_data.cal_version.minor_version.vfxp[0],
                          cal_data.cal_version.patch_version.vfxp[0],
                          self.compute_platform.os_build_version,
                          )
                self._calibration_version = (
                    f'{cal_data.cal_version.major_version.vfxp[0]:.0f}.'
                    f'{cal_data.cal_version.minor_version.vfxp[0]:.0f}.'
                    f'{cal_data.cal_version.patch_version.vfxp[0]:.0f}'
                )
                # Add your own logic here.
            else:
                log.debug('Calibration versioning not present on this Sensor Head. '
                          'Unknown compatibility with System version %s.',
                          self.compute_platform.os_build_version,
                          )
                # Add your own logic here.

        # --- INFO ---
        if cal_data.sensor_info.is_valid:
            self.isp.write_fields(sensor_id=cal_data.sensor_info.sn.vdig[0])
            pref = cal_data.sensor_info.prefix.vdig[0]
            self._sensor_prefix = bytearray.fromhex(
                hex(pref)[2::]).decode('utf-8')
        elif cal_data.info.is_valid:
            self.isp.write_fields(sensor_id=cal_data.info.sensor_sn.vdig[0])
        else:
            log.warning('Missing serial number')
        self.sensor_sn = self.isp.read_fields('sensor_id')

        ## ------------COMMON------------##
        # --- Dynamic range ---
        if cal_data.dyn.is_valid:
            default_doff = ((178 >> 2) & 0x3f) | (107 << 6)
            new_doff = default_doff - cal_data.dyn.doff_diff_adu.vdig[0]
            self.itof.write_fields(
                doff_lo=new_doff & 0x3f,
                doff_hi=(new_doff >> 6) & 0xff,
                pga_gain=cal_data.dyn.pga_gain.vdig[0],
            )
        else:
            log.warning('Missing dynamic range calibration')

        # --- Camera calibration ---
        if cal_data.cam.is_valid:
            self._pixel_mapping = PixelMapping(
                fx=cal_data.cam.fx.vfxp[0],
                fy=cal_data.cam.fy.vfxp[0],
                cx=cal_data.cam.cx.vfxp[0],
                cy=cal_data.cam.cy.vfxp[0],
                k1=cal_data.cam.k1.vfxp[0],
                k2=cal_data.cam.k2.vfxp[0],
                k3=cal_data.cam.k3.vfxp[0],
                p1=cal_data.cam.p1.vfxp[0],
                p2=cal_data.cam.p2.vfxp[0],
                n_rows=N_ROWS, n_cols=N_COLS,
            )
            # Need to write the supersampled mapping to disk
            # but use the default for A2A
            # fx, fy, cx, cy are scaled
            # but distortion coefficients don't change
            self._super_pixel_mapping = PixelMapping(
                fx=cal_data.cam.fx.vfxp[0] * 2,
                fy=cal_data.cam.fy.vfxp[0] * 2,
                cx=(cal_data.cam.cx.vfxp[0] + 0.5) * 2 - 1,
                cy=(cal_data.cam.cy.vfxp[0] + 0.5) * 2 - 1,
                k1=cal_data.cam.k1.vfxp[0],
                k2=cal_data.cam.k2.vfxp[0],
                k3=cal_data.cam.k3.vfxp[0],
                p1=cal_data.cam.p1.vfxp[0],
                p2=cal_data.cam.p2.vfxp[0],
                n_rows=(N_ROWS*2-1), n_cols=(N_COLS*2-1),
            )

        else:
            log.warning('Missing camera calibration, using default mapping')
            self._pixel_mapping = DEFAULT_PIXEL_MAPPING
            self._super_pixel_mapping = SUPERSAMPLED_PIXEL_MAPPING
        self._mapping_table_path = '/home/root/cobra/mapping_table_A.bin'
        self.super_pixel_mapping.write_mapping_table_file(self.mapping_table_path)

        # --- Angle to angle ---
        if cal_data.a2a.is_valid:
            if self.pixel_mapping == DEFAULT_PIXEL_MAPPING:
                log.warning('Using A2A with uncalibrated pixel map')
            self._roi_mapping = RoiMapping(
                a2a_coefficients=(
                    cal_data.a2a.ps_c_0.vfxp[0],
                    cal_data.a2a.ps_c_1.vfxp[0],
                    cal_data.a2a.ps_c_2.vfxp[0],
                    cal_data.a2a.ps_c_3.vfxp[0],
                ),
                pixel_mapping=self.pixel_mapping,
                lcm_assembly=self.lcm_assembly,
            )
            log.debug('Using ROI a2a coefficients: %s',
                      self._roi_mapping.a2a_coefficients)
        else:
            self._roi_mapping = RoiMapping(
                a2a_coefficients=(
                    48.44, -0.23821, 0.0002052, -0.000000285),
                pixel_mapping=self.pixel_mapping,
                lcm_assembly=self.lcm_assembly,
            )
            log.warning('Missing angle to angle calibration, '
                        'applying defaults until properly calibrated')

        self._cal_data_path = '/run/lumotive/cal_data_A.json'
        with open(self.cal_data_path, 'w', encoding='utf8') as f:
            json.dump(cal_data, f, cls=CalDataEncoder)

        # --- Range offset / pulse width ---
        if cal_data.range0807.is_valid:
            mfstr = '0807'
            cal_grp = getattr(cal_data, f'range{mfstr}')
        else:
            mfstr = None
            log.warning('Missing range calibration')

        if mfstr is not None:
            log.debug('Writing range cal for %s', mfstr)
            self.itof.write_delay_fields(
                laser_mg_sync=cal_grp.sync_laser_lvds_mg.vdig[0],
                dlay_mg_f0_coarse=cal_grp.dlay_mg_f0_coarse.vdig[0],
                dlay_mg_f0_fine=cal_grp.dlay_mg_f0_fine.vdig[0],
                dlay_laser_f0_coarse=cal_grp.dlay_laser_f0_coarse.vdig[0],
                dlay_laser_f0_fine=cal_grp.dlay_laser_f0_fine.vdig[0],
                dlay_mg_f1_coarse=cal_grp.dlay_mg_f1_coarse.vdig[0],
                dlay_mg_f1_fine=cal_grp.dlay_mg_f1_fine.vdig[0],
                dlay_laser_f1_coarse=cal_grp.dlay_laser_f1_coarse.vdig[0],
                dlay_laser_f1_fine=cal_grp.dlay_laser_f1_fine.vdig[0],
            )
            self.itof.write_shrink_expand_fields(
                nov_sel_laser_f0_shrink=cal_grp.pw_laser_f0_shrink.vdig[0],
                nov_sel_laser_f0_expand=cal_grp.pw_laser_f0_expand.vdig[0],
                nov_sel_laser_f1_shrink=cal_grp.pw_laser_f1_shrink.vdig[0],
                nov_sel_laser_f1_expand=cal_grp.pw_laser_f1_expand.vdig[0],
            )

        ## --- Range Temp Cal ---- ##
        # Configure metadata buffer with static metadata
        if cal_data.range_tmp.is_valid:
            # WARN all these values are shifted by 4 to fit into 12bits until we have a new FPGA image.
            range_cal_offset_mm_0807 = cal_data.range_tmp.rng_offset_mm_0807.vdig[0]
            self.range_cal_offset_mm_lo_0807 = range_cal_offset_mm_0807 & 0xfff
            self.range_cal_offset_mm_hi_0807 = (range_cal_offset_mm_0807 >> 12) & 0xf

            range_cal_mm_per_volt_0807 = cal_data.range_tmp.mm_per_volt_0807.vdig[0]
            self.range_cal_mm_per_volt_lo_0807 = range_cal_mm_per_volt_0807 & 0xfff
            self.range_cal_mm_per_volt_hi_0807 = (range_cal_mm_per_volt_0807 >> 12) & 0xf

            range_cal_mm_per_celsius_0807 = cal_data.range_tmp.mm_per_celsius_0807.vdig[0]
            self.range_cal_mm_per_celsius_lo_0807 = range_cal_mm_per_celsius_0807 & 0xfff
            self.range_cal_mm_per_celsius_hi_0807 = (range_cal_mm_per_celsius_0807 >> 12) & 0xf

            range_cal_offset_mm_0908 = cal_data.range_tmp.rng_offset_mm_0908.vdig[0]
            self.range_cal_offset_mm_lo_0908 = range_cal_offset_mm_0908 & 0xfff
            self.range_cal_offset_mm_hi_0908 = (range_cal_offset_mm_0908 >> 12) & 0xf

            range_cal_mm_per_volt_0908 = cal_data.range_tmp.mm_per_volt_0908.vdig[0]
            self.range_cal_mm_per_volt_lo_0908 = range_cal_mm_per_volt_0908 & 0xfff
            self.range_cal_mm_per_volt_hi_0908 = (range_cal_mm_per_volt_0908 >> 12) & 0xf

            range_cal_mm_per_celsius_0908 = cal_data.range_tmp.mm_per_celsius_0908.vdig[0]
            self.range_cal_mm_per_celsius_lo_0908 = range_cal_mm_per_celsius_0908 & 0xfff
            self.range_cal_mm_per_celsius_hi_0908 = (range_cal_mm_per_celsius_0908 >> 12) & 0xf

        # ----- pixel mask
        self.apply_pixel_mask_calibration(cal_data)

        # N.B. If the front end is streaming and you want it to
        # start using new calibration data you must call fe_stop_streaming
        # and fe_start_streaming

        # Save this off so that we don't need to read the spi flash a bunch
        # of times when changing the scan table/modulation frequencies
        self._cal_data = cal_data

    def is_compatible_calibration_version(self, cal_data: 'CalData'):
        """Checks the version used to calibrate the system against
        the system version currently running for compatibility.

        Currently, all releases using the new calibration versioning
        will be compatible.
        """
        if cal_data.cal_version.is_valid:
            # pylint: disable-next=unused-variable
            cal_data_version = (int(cal_data.cal_version.major_version.vfxp[0]),
                                int(cal_data.cal_version.minor_version.vfxp[0]),
                                int(cal_data.cal_version.patch_version.vfxp[0]))
        if (
                (self.compute_platform.os_build_version != "Not available")
                and (self.compute_platform.os_build_version != "")
        ):
            try:
                # pylint: disable-next=unused-variable
                os_version = tuple([int(x) for x in self.compute_platform.os_build_version[1:].split('.')])
            # pylint: disable-next=broad-exception-caught
            except Exception as e:
                log.debug(e)
            # Add comparison logic here to determine if calibration is compatible.
        return True

    def apply_pixel_mask_calibration(self, cal_data: 'CalData'):
        # --- Pixel masking ---
        if cal_data.cam.is_valid:
            mask = create_default_pixel_mask(cal_data.cam.cx.vfxp[0],
                                             cal_data.cam.cy.vfxp[0])
        else:
            # Uses fixed center in case there is no camera calibration
            mask = create_default_pixel_mask(320, 240)

        # Easier for R2D to use 480x640 mask and for the values to be 0, 0xffff
        mult_mask = (mask * 0xffff).astype(np.uint16)
        path = '/run/lumotive/pixel_mask_A.bin'
        with open(path, 'wb') as fid:
            mult_mask.tofile(fid)
        log.debug('Wrote pixel mask to %s', path)

    def config_rx_bd_rev(self):
        """In M30, reads the PCB revision from a resistor chain
        connected to the FPGA, representing a binary
        representation of PCB revision. 0->Rev1.
        """
        if self.whoami == 'm30':
            self.debug.write_fields(pcbrev_input_en=1)
            str_rev = self.debug.read_fields('pcbrev', use_mnemonic=True)
            try:
                int_rev = int(str_rev[-1])
            except TypeError:
                # No matching mnemonic
                log.warning('The pcbrev return %s has no matching mnemonic. '
                            'Setting rx_pcb_rev to 0', str_rev)
                int_rev = 0
            self.debug.write_fields(pcbrev_input_en=0)
        else:
            int_rev = 2
        log.debug('RX PCB REV = %s', int_rev)
        return int_rev

    def enable(self):
        """Enables all the voltages. Calls enable() on all the peripherals
        Don't enable the high voltage dacs because it causes a dip on VIN
        """
        for periph in set(self.periphs) ^ { self.cmb_lcm_v_dac, self.cmb_laser_vlda_dac, self.sh_laser_vlda_dac }:
            periph.enable()

        # In M30, there is an 18V LDO before the efuse
        lcm_v_min = 12
        lcm_v_set = 21
        laser_vlda_min = 10
        laser_vlda_set = 18
        delay_time_s = 0.05

        # Set the LCM voltage to its minimum value before enabling
        self.cmb_lcm_v_dac.set_voltage(lcm_v_min)
        time.sleep(delay_time_s)
        self.cmb_lcm_v_dac.enable()

        # Slowly ramp up LCM voltage
        for v in range(lcm_v_min + 1, lcm_v_set + 1):
            self.cmb_lcm_v_dac.set_voltage(v)
            time.sleep(delay_time_s)

        # Set VLDA to its minimum value before enabling
        self.set_laser_vlda_combined(laser_vlda_min)
        time.sleep(delay_time_s)
        self.cmb_laser_vlda_dac.enable()
        self.sh_laser_vlda_dac.enable()

        # Slowly ramp up VLDA
        for v in range(laser_vlda_min + 1, laser_vlda_set + 1):
            self.set_laser_vlda_combined(v)
            time.sleep(delay_time_s)

    # I don't understand why pylint is throwing an error here. Everything
    # works fine.
    # pylint: disable=too-many-function-args
    def start(
            self,
            start_pointer: Optional[int] = None,
            stop_pointer: Optional[int] = None,
            start_fe_streaming: Optional[bool] = True,
    ):
        # pylint: enable=too-many-function-args
        """Start the scan controller.

        By default, the controller is set to start by looping over all applied
        scan entries; this behavior can be changed by providing pointers.

        If the controller is configured for loopback: the scan will repeat
        between the pointers indefinitely.

        If controller is configured for non-loopback: the scan will run between
        the pointers exactly once.
        """
        log.debug('sensor head start called')
        # Make sure the scan controller has access to the metadata buffer
        self.scan.write_fields(scan_lmmi_meta_en=0)

        if start_fe_streaming:
            mode = fe_ctl.fe_get_mode(
                self._fe_rows, self._fe_reduce_mode, self.aggregate)
            fe_ctl.fe_start_streaming(mode)
            log.debug('Started frontend streaming %s rows', self._fe_rows)

        # writing the pointers starts the scan controller
        if start_pointer is not None and stop_pointer is not None:
            self.write_scan_fifo(start_ptr=start_pointer, stop_ptr=stop_pointer)
        elif start_pointer is not None or stop_pointer is not None:
            raise ValueError('if passing in pointers, must pass start_pointer and stop_pointer')
        else:
            self.write_scan_fifo(*self.valid_scan_table_pointer_range)

        # Turn on customer-exposed DATA LED
        self.compute_platform.data_led.enable()
        mode = ('loopback' if self.scan.read_fields('scan_loopback') == 1
                else 'non-loopback')
        log.info('*** Started scanning SH in %s mode ***', mode)

    def write_scan_fifo(self, start_ptr: int, stop_ptr: int):
        """Writes the scan fifo pointers; can be called when the system is
        scanning in loopback or non-loopback mode
        """
        if any([not is_in_bounds(v, *self.valid_scan_table_pointer_range)
                for v in (start_ptr, stop_ptr)]):
            raise ValueError(
                f'start / stop ptrs must be in '
                f'{self.valid_scan_table_pointer_range}, '
                f'but start is {start_ptr} and stop is {stop_ptr}'
            )

        is_loopback = self.scan.read_fields('scan_loopback') == 1
        if is_loopback:
            self.scan.write_fields(scan_halt=1)
            self.scan.wait_for_scan_idle()
            self.scan.write_fields(scan_halt=0)

        # wait until fifo count is below watermark before adding more
        while self.scan.read_fields('fifo_count') >= 4:
            time.sleep(0.001)
        self.scan.write_fields(fifo_wdata=start_ptr & 0xFF)
        self.scan.write_fields(fifo_wdata=start_ptr >> 8)
        self.scan.write_fields(fifo_wdata=stop_ptr & 0xFF)
        self.scan.write_fields(fifo_wdata=stop_ptr >> 8)

    def stop(self, stop_fe_streaming: Optional[bool] = True):
        """Stops the scan controller and stop frontend streaming
        """
        if (self.state is State.INITIALIZED):
            return
        self.scan.stop()

        # Turn off customer-exposed DATA LED
        self.compute_platform.data_led.disable()

        if stop_fe_streaming:
            fe_ctl.fe_stop_streaming()
            log.info('Stopped point cloud streaming')

    def disable(self):
        """Disables the peripherals, VLDA voltage, and LCM voltage.

        Stops first so the state goes back to READY if SCANNING
        """
        if self.state is State.SCANNING:
            self.stop()

        # Disable LCM/Laser voltages on Sensor Head side
        self.lcm_ctrl.write_fields(gpio_pwr_en=0)
        self.sh_laser_vlda_dac.disable()

        log.info('*** Stopped scanning SH ***')
        for x in reversed(self.periphs):
            x.disable()
        log.debug('Disabled SH')

    def disconnect(self):
        """Disconnects from hardware"""
        self.disable()
        for x in reversed(self.periphs):
            x.disconnect()
        log.debug('Disconnected SH')

    def _check_matching_cal_in_spi_flash(self, cal: CalBase):
        ba_written = self._read_cal(type(cal)).ba
        if cal.ba != ba_written:
            raise RuntimeError('Error writing spi flash')
        else:
            log.debug('Wrote %s successfully', type(cal))

    def get_cal_data(self) -> 'CalData':
        """Reads calibration data from the spi flash"""
        if self._cal_data is None:
            return self._read_cal(self.cal_data_class)
        else:
            log.debug('Loading cal data class attribute')
            return self._cal_data

    def set_cal_data(self, cal_data: 'CalData'):
        """Applies calibration data to system and writes to spi flash"""
        if not isinstance(cal_data, CalData):
            raise TypeError('cal_data object is not of type CalData')
        self.apply_calibration(cal_data)
        self._write_cal(cal_data)
        self._check_matching_cal_in_spi_flash(cal_data)

    def serial_number(self) -> int:
        """Returns serial number written to spi flash"""
        return self.isp.read_fields('sensor_id')

    def _write_cal(self, cal_data: CalBase):
        """Erases enough memory to write new calibration data

        User needs to be aware that more data may be deleted than
        is written.
        """
        # Erase in 4k blocks
        num_sectors = sectors_from_size(cal_data.size_bytes())
        for block in range(num_sectors):
            addr = (cal_data.ADDRESS_BASE + (block * wb.W25Q32JW_SECTOR_SIZE))
            self.spi_flash.qspi.sector_erase(addr)

        # Write in 256 byte blocks
        log.debug('Writing cal data %s with %s bytes',
                  cal_data, len(cal_data.ba))
        pages = ba_to_pages(cal_data.ba)
        for idx, page in enumerate(pages):
            addr = cal_data.ADDRESS_BASE + (idx * wb.W25Q32JW_PAGE_SIZE)
            self.spi_flash.qspi.page_program(addr, page)

    def _read_cal(self, cal_type: CalBase) -> 'Cal':
        """Reads the SPI Flash to ensure the data present
        matches the expected values
        """
        log.debug('Loading %s from Spi Flash', cal_type)
        return self._read_cal_normal(cal_type)

    def _read_cal_normal(self, cal_type):
        blocks_to_read = reads_from_size(cal_type.size_bytes())
        log.debug('Reading %s blocks in spi flash to get create '
                  'a %s instance', blocks_to_read, cal_type.__name__)
        read_bytes = bytearray()
        # Read in 1k blocks
        for block in range(blocks_to_read):
            addr = (cal_type.ADDRESS_BASE
                    + (block * wb.W25Q32JW_FAST_READ_SIZE))
            data = self.spi_flash.qspi.fast_read_data(addr)
            if isinstance(data, bytearray):
                read_bytes.extend(data)
            else:
                raise TypeError('need to extend bytearray, '
                                f'data is {type(data)}, {data}')
        # Make the same size
        read_bytes = read_bytes[0:cal_type.size_bytes()]
        return cal_type(read_bytes)

    def sh_vlda_dac_adc_calibration(self):
        """Calibrates the slope and offset values for the Sensor Head VLDA
        DAC from ADC measurements.

        The Min rail output voltage is with maximum DAC output
        The Max rail output voltage is with minimum DAC output

        The FPGA takes 256 * 0.64us * 34 = 5.6ms to loop through
        all the ADC measurements before the one we're interested in
        gets updated. Use a 10ms sleep.

        The Microchip MCP48FVB12 DAC settles in about 10us but the
        capacitance on the rail means it settles in about 350ms.
        Use a 1s sleep
        """
        def cmb_adc_dac_loop_and_log(vdac, adc_class, before_or_after: str):
            min_max_v = []
            for dac_code in [2**vdac.dac.dac_bits-1, 0]:
                vdac.dac.dac_write(
                    'write_update',
                    vdac.chan_idx,
                    dac_code << vdac.dac.bit_shift
                )
                time.sleep(1)  # let the rail settle
                rdac = vdac.get_voltage()
                l_rdata = []
                for _ in range(10):
                    l_rdata.append(adc_class.get_mon_vlda())
                    time.sleep(0.01)  # FPGA ADC update

                rdata_v = np.mean(l_rdata)
                min_max_v.append(rdata_v)

                log.info(
                    '%s calibration for SH VLDA ADC/DAC. %s '
                    'DAC setting %0.5f V; '
                    'ADC measurement %0.5f V; '
                    'Abs Error %0.5f V; '
                    'Rel Error %0.5f V. ',
                    before_or_after,
                    vdac,
                    rdac,
                    rdata_v,
                    rdac - rdata_v,
                    abs(1 - rdac / rdata_v),
                )
            return min_max_v

        self.cmb_laser_vlda_dac.set_voltage(50)
        self.cmb_laser_vlda_dac.enable()
        vdac = self.sh_laser_vlda_dac
        self.sh_laser_vlda_dac.enable()

        min_max_v = cmb_adc_dac_loop_and_log(
            vdac, self.fpga_adc, 'Before')

        # Get slope and offset by solving system of equations
        # 3.3V = m * vmin + b
        # 0V = m * vmax + b
        # 3.3V = m * vmin - (m * vmax)
        # b = - (m * vmax)
        m = 3.3 / (min_max_v[0] - min_max_v[1])
        b = -m * min_max_v[1]
        vdac.slope = m
        vdac.offset = b

        log.info(
            'Slope %0.5f, '
            'Offset %0.5f ',
            vdac.slope,
            vdac.offset
        )

        # Check output after calibration
        _ = cmb_adc_dac_loop_and_log(
            vdac, self.fpga_adc, 'After')

    def read_fpga_adc_voltage(self, monitor: str):
        """Reads an FPGA ADC registor, monitor, and applies
        the gain and offset values. This is convenience
        method that is exposed by Pyro.
        """
        return self.fpga_adc.read_adc_and_adjust(monitor)

    def fpga_adc_gain_offset(self) -> tuple:
        return self.fpga_adc.cal_gain, self.fpga_adc.cal_offset

    def set_laser_vlda_combined(self, vlda_v: float):
        """Sets the laser vlda voltage considering both the DAC on
        the compute PCB and the DAC on the the RX.
        This is a convenience function to help set a more accurate
        VLDA for the laser.
        """
        # These will get clipped with this high input value
        max_cmb_vlda_v = self.cmb_laser_vlda_dac.voltage_from_field(
            self.cmb_laser_vlda_dac.field_from_voltage(50))

        # Don't limit the input voltage
        self.cmb_laser_vlda_dac.set_voltage(max_cmb_vlda_v)
        # Limit on RX
        self.sh_laser_vlda_dac.set_voltage(vlda_v)

    def get_laser_vlda_combined(self) -> float:
        """Gets the laser vlda voltage considering both the DAC on
        the compute PCB and the DAC on the the RX.
        This is a convenience function to help get a more accurate
        VLDA for the laser.
        """
        max_cmb_vlda_v = self.cmb_laser_vlda_dac.voltage_from_field(
            self.cmb_laser_vlda_dac.field_from_voltage(50))
        cmb_v = self.cmb_laser_vlda_dac.get_voltage()
        cmb_frac = cmb_v / max_cmb_vlda_v
        if cmb_frac < 0.95:
            raise ValueError('Please set NCB 24V rail to max')
        return self.sh_laser_vlda_dac.get_voltage()

    @property
    def whoami(self):
        return self._whoami

    @property
    def compute_platform(self):
        return self._compute_platform

    @property
    def cal_data_path(self):
        return self._cal_data_path

    @property
    def mapping_table_path(self):
        return self._mapping_table_path

    @property
    def cal_data(self):
        return self._cal_data

    @property
    def itof(self) -> Itof:
        return self._itof

    @property
    def ito_dac(self) -> ItoDac:
        return self._ito_dac

    @property
    def fpga_field_funcs(self) -> 'FpgaFieldFuncs':
        return self._fpga_field_funcs

    @property
    def laser_ci_dac(self) -> LaserCiDac:
        return self._laser_ci_dac

    @property
    def cmb_laser_vlda_dac(self) -> LaserVldaDac:
        return self._cmb_laser_vlda_dac

    @property
    def sh_laser_vlda_dac(self) -> LaserVldaDac:
        return self._sh_laser_vlda_dac

    @property
    def cmb_lcm_v_dac(self) -> LcmVDac:
        return self._cmb_lcm_v_dac

    @property
    def lcm_ctrl(self) -> 'LcmController':
        return self._lcm_ctrl

    @property
    def debug(self) -> FpgaDbg:
        return self._debug

    @property
    def isp(self) -> ISP:
        return self._isp

    @property
    def metabuff(self) -> MetadataBuffer:
        return self._metabuff

    @property
    def spi_flash(self) -> SpiFlash:
        return self._spi_flash

    @property
    def fpga_adc(self) -> 'FpgaAdc':
        return self._fpga_adc

    @property
    def scan(self) -> Scan:
        return self._scan

    @property
    def scan_params(self) -> ScanParams:
        return self._scan_params

    @property
    def pixel_mapping(self) -> PixelMapping:
        return self._pixel_mapping

    @property
    def super_pixel_mapping(self) -> PixelMapping:
        return self._super_pixel_mapping

    @property
    def roi_mapping(self) -> RoiMapping:
        return self._roi_mapping

    @property
    def random_access_scan(self) -> RandomAccessScanning:
        return self._random_access_scan

    @property
    def lcm_assembly(self) -> LcmAssembly:
        return self._lcm_assembly

    @property
    def is_available(self):
        return self.state not in (State.INITIALIZED,)

    @property
    def db_sensor_configuration(self):
        return self._db_sensor_configuration

    @property
    def sensor_id(self):
        return f'{self._sensor_prefix}{self.sensor_sn:06d}'

    @property
    def rx_pcb_rev(self):
        return self._rx_pcb_rev

    @property
    def fpga_adc_cal_gain(self):
        return self._fpga_adc.cal_gain

    @property
    def fpga_adc_cal_offset(self):
        return self._fpga_adc.cal_offset

    @property
    def calibration_version(self):
        return self._calibration_version

    # apply state transition map to relevant Cobra methods here to avoid
    # cluttering the space above with decorators
    connect = state_transition(
        {State.INITIALIZED: State.CONNECTED,
         State.CONNECTED: State.CONNECTED})(connect)
    setup = state_transition(
        {State.CONNECTED: State.READY,
         State.READY: State.READY})(setup)
    apply_settings = state_transition(
        {State.CONNECTED: State.CONNECTED,
         State.READY: State.READY,
         State.ENERGIZED: State.ENERGIZED,
         State.SCANNING: State.ENERGIZED})(apply_settings)
    apply_calibration = state_transition(
        {State.READY: State.READY,
         State.CONNECTED: State.CONNECTED,
         State.ENERGIZED: State.ENERGIZED})(apply_calibration)
    enable = state_transition(
        {State.READY: State.ENERGIZED,
         State.ENERGIZED: State.ENERGIZED})(enable)
    start = state_transition({State.ENERGIZED: State.SCANNING,
                              State.SCANNING: State.SCANNING})(start)
    write_scan_fifo = state_transition(
        {State.SCANNING: State.SCANNING,
         State.ENERGIZED: State.SCANNING,
         })(write_scan_fifo)
    stop = state_transition(
        {State.SCANNING: State.ENERGIZED,
         State.ENERGIZED: State.ENERGIZED,
         State.READY: State.READY,
         State.CONNECTED: State.CONNECTED,
         State.INITIALIZED: State.INITIALIZED})(stop)
    disable = state_transition(
        {State.SCANNING: State.READY,
         State.ENERGIZED: State.READY,
         State.READY: State.READY,
         State.CONNECTED: State.CONNECTED,
         State.INITIALIZED: State.INITIALIZED})(disable)
    disconnect = state_transition(
        {State.INITIALIZED: State.INITIALIZED,
         State.CONNECTED: State.INITIALIZED,
         State.READY: State.INITIALIZED,
         State.ENERGIZED: State.INITIALIZED,
         State.SCANNING: State.INITIALIZED})(disconnect)


def make_start_stop_flag_array(
        start_stop_flags: Union[Sequence[int], None],
        num_flags: int, dsp_mode: Enum,
        summed_rois: int = 0,
):
    """If start stop flags are not provided, this creates the flag array
    with the proper parameters for each mode.

    If the flags are not None, they are returned unchanged.
    """
    if start_stop_flags is None:
        if dsp_mode == DspMode.CAMERA_MODE:
            start_stop_flags = [0] * num_flags
            if summed_rois > 0:
                for k, _ in enumerate(start_stop_flags):
                    for j in range(NUM_VIRTUAL_SENSOR):
                        start_stop_flags[k] |= (0b11 << (j*4+2))

            for j in range(NUM_VIRTUAL_SENSOR):
                start_stop_flags[0] |= (0b01 << (j*4))
                start_stop_flags[-1] |= (0b10 << (j*4))
        elif dsp_mode == DspMode.LIDAR_MODE:
            if summed_rois > 0:
                start_stop_flags = [STRIPE_MODE_SUMMED_FLAGS] * num_flags
            else:
                start_stop_flags = [STRIPE_MODE_FLAGS] * num_flags

    return start_stop_flags
