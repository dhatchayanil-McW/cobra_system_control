"""
file: random_access_scanning.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file provides all the functionality of Programmable Lidar
as defined in the Lumotive Application Note: Programmable Lidar.
This class is the main interface to receive commands from the API
to change scan configuration (through the method
sensorhead.apply_random_access_settings())

The output of this class is a dictionary that is fed
directly to sensorhead.apply_settings().

New process with Programmable Lidar:
API calls apply_random_access_scan_settings()
  function feeds parameters to RandomAccessScanning class
  RAS does error checking from inputs
  RAS get common size of the inputs
  RAS Creates PerVirtualSensorMetadata list for each Virtual Sensor defined
  RAS Creates dictionary entries so apply_settings() can create a StaticMetadata object
  RAS creates attributes from schedule the scan
  RAS returns lists needed for apply_settings with to_apply_settings
  Triple dip/double dip: Share ROIs across Virtual Sensors if possible for frame rate increase.
  schedule scan creates dictionary that can be fed to apply settings directly
  dictionary sent to apply_settings()

With new process, apply_random_access_scan_settings() can be bypassed
and apply_settings() used as normal. This should keep most existing scripts from
breaking.

With the new framework, apply_settings()  needs to create
and write the metadata buffer correctly. If you keep these
arguments as None, they will be calculated automatically:
if virtual_sensor_metadata is None - calculate automatically
if staticmetadata is None - calculate automatically
"""
from collections import Counter
import copy
import dataclasses as dc
from enum import Enum
from numbers import Number
import random
from typing import Union, Sequence, Tuple

import numpy as np

import cobra_system_control.exceptions as cobex
from cobra_system_control.functional_utilities import get_common_length
from cobra_system_control.metadata import PerVirtualSensorMetadata
from cobra_system_control import remote
from cobra_system_control.scan_control import (
    SnrThresholdBv, BinningOv, SCAN_TABLE_SIZE)
from cobra_system_control.validation_utilities import cast_to_sequence
from cobra_system_control.values_utilities import OptionValue


DEFAULT_ANGLE_STEP = 1

NON_ARRAY_ITEMS = [
    'virtual_sensor_metadata',
    'hdr_threshold',
    'dsp_mode',
]

INTE_TIME_US_OPTIONS = list(range(1, 20+1))

STRIPE_MODE_FLAGS = 0b00110011001100110011001100110011
STRIPE_MODE_SUMMED_FLAGS = 0b11111111111111111111111111111111


class DspMode(Enum):
    """An enum for the available DSP Modes
    """
    CAMERA_MODE = 0
    LIDAR_MODE = 1

    def __init__(self, key: int):
        self.key = key

    @classmethod
    def from_key(cls, val: int):
        for name, _ in cls.__members__.items():
            if cls[name].key == val:
                return cls[name]


class InteTimeIdxMappedOv(OptionValue):
    """Sets the valid integration time options
    """
    MAP = list(np.asarray(INTE_TIME_US_OPTIONS) / 1e6)
    OPTIONS = list(INTE_TIME_US_OPTIONS)

    @property
    def mapped(self):
        return InteTimeIdxMappedOv.MAP[
            InteTimeIdxMappedOv.OPTIONS.index(self.value)]


class MaxRangeIdxMappedOv(OptionValue):
    """Sets the valid options for mod freq tuples
    based on max range
    """
    OPTIONS = [25.2, 32.4]
    MAP = {25.2: (8, 7),
           32.4: (9, 8),
           }

    @property
    def mapped(self) -> tuple:
        return MaxRangeIdxMappedOv.MAP[self.value]

    def __hash__(self):
        """Hash method to enable set() function on instances
        of this class
        """
        return hash(self.value)


class FrameRateOv(OptionValue):
    """Sets the valid frame rate options in Hz.

    The nominal rate of 15us integration time is 840.5 -> 840 Hz
    The nominal rate of 5us integration time is 960 Hz
    """
    OPTIONS = list(range(300, 960+10, 10))


class FpsMultipleOv(OptionValue):
    """A class to set the options for fps multiple
    """
    OPTIONS = list(range(1, 2**5))

    @property
    def field(self):
        return self.value


class NnLevelOv(OptionValue):
    """A class to set the options for the nearest
    neighbor filter
    """
    OPTIONS = list(range(6))

    @property
    def field(self):
        return self.value


def fix_angle_triplet(t: Tuple[float, float, float]) -> np.ndarray:
    """Fixes the signs of a Virtual Sensor angle triplet to produce the
    correct list of angles depending on the sign of the angles
    """
    if not len(t) == 3:
        raise cobex.ScanPatternSizeError(
            f'angle_range triplet is size {len(t)} and not size 3')
    start = t[0]
    stop = t[1]
    step = t[2]
    if start == stop:
        return start, stop+1, step
    elif start > stop:
        stop = stop - 1
        if step > 0:
            step = -1 * step
    elif start <= stop:
        stop = stop + 1
        if step < 0:
            step = -1 * step
    return start, stop, step


def schedule_scan(future_apply_settings_dict: dict, sorting_list: list):
    """Schedules the scan based based on the sorting_list and a dictionary
    that holds lists to be fed as arguments to apply_settings()
    """
    # The Sorting Hat chooses the houses
    sort_idx = np.argsort(np.asarray(sorting_list))
    # The children are ordered
    for k, v in future_apply_settings_dict.items():
        if k not in NON_ARRAY_ITEMS:
            future_apply_settings_dict[k] = [v[i] for i in sort_idx]
    return future_apply_settings_dict


def replace_flag(flag, new_val, virtual_sensor):
    """Changes the value of a start/stop flag for a specific
    Virtual Sensor.
    """
    flag = np.asarray(flag)
    new_val = np.asarray(new_val)
    bit_mask = 0xf << (4*virtual_sensor)
    return (flag & (~bit_mask)) | (new_val << (4*virtual_sensor))


def is_valid_normal(virtual_sensor_flags: Sequence[int]) -> bool:
    """Don't change anything if the scan flags are generally valid
    """
    return (
        # Starts with start flag
        (virtual_sensor_flags[0] == 1)
        # Ends with stop flag
        and (virtual_sensor_flags[-1] == 2)
        # No non-zero flags in sequence
        and (0 not in np.diff(virtual_sensor_flags[virtual_sensor_flags != 0]))
        # No zero flags after a stop flag
        and (-2 not in np.diff(virtual_sensor_flags[1:-1]))
        # Even number of flags (a start and stop pair)
        and (np.flatnonzero(virtual_sensor_flags).size % 2 == 0)
    )


def is_almost_valid_reversed(virtual_sensor_flags: Sequence[int]) -> bool:
    """If the order can be reversed and be valid, then we should do that
    Like: [2, 0, 1, 2, 0, 1, 2, 0, 1]
    Like: [2, 1, 2, 1]
    Technically, [2,1,2,1] is valid but we can reduce the number functions
    we need if we just reverse it.
    """
    temp = virtual_sensor_flags[::-1]
    return is_valid_normal(temp)


def is_almost_valid_threes(virtual_sensor_flags: Sequence[int]) -> bool:
    """Make sure single flag Virtual Sensors are 3's
    """
    return len(virtual_sensor_flags) == 1


def flag_equalizer(flags: list, bitmask: list, fps: list) -> list:
    """This function was added to fix bugs related to double dipping.
    More massaging of the start_stop_flags is needed to avoid
    incorrect Virtual Sensors being emitted from R2D. This attempts to fix
    the ordering and position of the flags after the scan has been
    sequenced so that
    - A Virtual Sensor starts with a start flag
    - A Virtual Sensor ends with a stop flag
    - A Virtual Sensor doesn't have the same flag in sequence (or with zeros between)
    - A Virtual Sensor has a start flag immediately after a stop flag

    Need to make sure the np.diff has no zeros
    """
    flags = np.asarray(flags)

    for virtual_sensor, fr in enumerate(fps):
        loc_idx = np.flatnonzero((np.asarray(bitmask) >> virtual_sensor) & 0b1 == 1)
        virtual_sensor_flags = (flags[loc_idx] >> (4*virtual_sensor)) & 0xf

        if is_valid_normal(virtual_sensor_flags):
            continue
        elif is_almost_valid_reversed(virtual_sensor_flags):
            flags[loc_idx] = replace_flag(
                flags[loc_idx], virtual_sensor_flags[::-1], virtual_sensor)
        elif is_almost_valid_threes(virtual_sensor_flags):
            flags[loc_idx] = replace_flag(flags[loc_idx], 3, virtual_sensor)
        elif not all(virtual_sensor_flags == 3):
            # If all the entries are 3, it is a fps multiplied
            # single angle scan.
            # Otherwise, it needs to be configured correctly.
            # clear out the old
            flags[loc_idx] = replace_flag(flags[loc_idx], 0, virtual_sensor)
            # Put in new equidistant ones
            new_flag_indexes = np.round(np.linspace(
                0, len(loc_idx), fr.value+1)).astype(int)

            # start flag on the first ROI
            flags[loc_idx[0]] = replace_flag(
                flags[loc_idx[0]], 1, virtual_sensor)

            subset = new_flag_indexes[1:-1]
            if len(subset) != 0:
                flags[loc_idx[subset-1]] = replace_flag(
                    flags[loc_idx[subset-1]], 2, virtual_sensor)
                flags[loc_idx[subset]] = replace_flag(
                    flags[loc_idx[subset]], 1, virtual_sensor)

            # Stop flag on the last ROI
            flags[loc_idx[-1]] = replace_flag(
                flags[loc_idx[-1]], 2, virtual_sensor)

        new_flags = (flags[loc_idx] >> (4*virtual_sensor)) & 0xf
        # log.debug(f'virtual_sensor{virtual_sensor} new flags = {new_flags}')
        temp = new_flags[new_flags != 0]
        temp = temp[temp != 3]
        if 0 in np.diff(temp):
            raise ValueError(
                'Start stop flags have a sequential '
                f'start or stop for a Virtual Sensor, {new_flags}')

    # cast back to a list so tests don't complain
    return list(flags)


@remote.register_for_serialization
@dc.dataclass
class RandomAccessScanning:
    """Class to take a random access scan definition and convert it into
    a set of parameters that can be fed to apply_settings().
    Up to 8 distinct Virtual Sensors can be defined.
    This class interleaves scans based on the desired frame rate multiple
    This class double dips ROIs if they are defined in more than one Virtual Sensor
    to increase frame rate

    Args:
    Required Args:
    angle_range: (start, stop, step) translated to an angle list
    fps_multiple: Number of times to multiply the virtual sensor
    laser_power_percent: Value mapped to Laser CI
    inte_time_us: Value mapped to itof integration time
    max_range_m: Value mapped to set of modulation frequencies
    binning: Level of symmetric binning
    user_tag: Value specified by user for identifying streams
    roi_rows: Rows in ROI
    frame_rate_hz: Rate at which single depth measurements are collected
    roi_mapping: RoiMapping class to determine order and start row from angle
    snr_threshold: Threshold filter level on signal to background level
    nn_level: Nearest Neighbor filter level

    dsp_mode: Selects which DSP processing mode to use.
              {0: Grid Mode (Camera Mode), 1: Stripe Mode (Lidar Mode)}
    rtd_algorithm_common: Select which algorithms common to both modes to use in depth processing
    rtd_algorithm_grid_mode: Select which algorithms for grid mode to use in depth processing
    rtd_algorithm_stripe_mode: Select which algorithms for stripe mode to use in depth processing

    double_dip: Should there be ROI sharing between Virtual Sensors?
    interleave: Should different Virtual Sensor ROIs be time interleaved?

    hdr_threshold: Threshold value to trigger HDR retry
    hdr_laser_power_percent:
    hdr_inte_time_us:

    laser_power_mapped_cls:

    Post Init Args:
    total_rois: Total number of rois in the defined scan. Used for
       interleaving the rois
    appset_dict: Dictionary to hold values that are passed
       directly to apply_settings()
    sorting: Calculated floats that are sorted to interleave the
       rois in a defined scan
    """
    angle_range: Sequence[Tuple[float, float, float]]
    fps_multiple: Union[int, Sequence[int]]
    laser_power_percent: Union[int, Sequence[int]]
    inte_time_us: Union[int, Sequence[int]]
    max_range_m: Union[int, Sequence[int]]
    binning: Union[int, BinningOv, Sequence[int], Sequence[BinningOv]]
    user_tag: Union[int, Sequence[int]]
    roi_rows: int
    frame_rate_hz: Union[int, Sequence[int]]

    roi_mapping: 'RoiMapping'
    total_rois: int = dc.field(init=False)

    snr_threshold: Union[SnrThresholdBv,
                         Number,
                         Sequence[Number],
                         Sequence[SnrThresholdBv]]
    nn_level: Union[int, NnLevelOv, Sequence[int], Sequence[NnLevelOv]]

    dsp_mode: Enum
    rtd_algorithm_common: Union[int, Sequence[int]]
    rtd_algorithm_grid_mode: Union[int, Sequence[int]]
    rtd_algorithm_stripe_mode: Union[int, Sequence[int]]

    double_dip: bool
    interleave: bool

    laser_power_mapped_cls: 'LaserPowerPercentMappedOv'

    hdr_threshold: int
    hdr_laser_power_percent: Union[int, Sequence[int]]
    hdr_inte_time_us: Union[int, Sequence[int]]
    hdr_is_shared_strategy_minimum: bool = True

    # Will be passed into apply settings
    appset_dict: dict = dc.field(
        default_factory=lambda:
        {
            'orders': [],
            's_rows': [],
            'ci_v': [],
            'inte_time_s': [],
            'frame_rate_hz': [],
            'mod_freq_int': [],
            'virtual_sensor_bitmask': [],
            'virtual_sensor_metadata': [],
            'start_stop_flags': [],
            'binning': [],
            'hdr_threshold': None,
            'hdr_ci_v': [],
            'hdr_inte_time_s': [],
            'dsp_mode': None,
        }
    )
    sorting: Sequence[Number] = dc.field(default_factory=lambda: [])

    # Full set of data before double dip
    full_appset_dict: dict = dc.field(
        default_factory=lambda:
        {
            'orders': [],
            's_rows': [],
            'ci_v': [],
            'inte_time_s': [],
            'frame_rate_hz': [],
            'mod_freq_int': [],
            'virtual_sensor_bitmask': [],
            'virtual_sensor_metadata': [],
            'start_stop_flags': [],
            'binning': [],
            'hdr_threshold': None,
            'hdr_ci_v': [],
            'hdr_inte_time_s': [],
            'dsp_mode': None,
        }
    )
    full_sorting: Sequence[Number] = dc.field(default_factory=lambda: [])

    # Trimmed data after double dip
    trimmed_appset_dict: dict = dc.field(
        default_factory=lambda:
        {
            'orders': [],
            's_rows': [],
            'ci_v': [],
            'inte_time_s': [],
            'frame_rate_hz': [],
            'mod_freq_int': [],
            'virtual_sensor_bitmask': [],
            'virtual_sensor_metadata': [],
            'start_stop_flags': [],
            'binning': [],
            'hdr_threshold': None,
            'hdr_ci_v': [],
            'hdr_inte_time_s': [],
            'dsp_mode': None,
        }
    )
    trimmed_sorting: Sequence[Number] = dc.field(default_factory=lambda: [])
    ras_applied_settings: dict = dc.field(default_factory=lambda: {})

    def __post_init__(self):
        """Starts with a lot of parameter checking and sizing.
        Then expands the Virtual Sensors into entries for apply_settings and
        orders the scan based on the settings.
        """
        # Check the virtual_sensor angle sequences
        if len(self.angle_range) == 0:
            raise cobex.ScanPatternSizeError('Must define at least one Virtual Sensor')
        if len(self.angle_range) > 8:
            raise cobex.ScanPatternSizeError('Cannot define more than 8 Virtual Sensors')
        for idx, tup in enumerate(self.angle_range):
            if isinstance(tup, Number):
                raise cobex.ScanPatternValueError(
                    'angle_range definition must be a '
                    f'sequence of sequences but is {tup}')
            if len(tup) != 3:
                raise cobex.ScanPatternSizeError(
                    f'Virtual Sensor sequence must define (start, stop, step) but '
                    f'defines {tup}.')

            if not all((abs(tup[0]) <= 45, abs(tup[1]) <= 45)):
                raise cobex.ScanPatternValueError(
                    f'Virtual Sensor angles must be between -45 deg and 45 deg '
                    f'but are ({tup[0]}, {tup[1]})')

        # The parameters need to be sequences to feed into the get_common_length function
        # Check the user tags. If none, provide random values
        if self.user_tag is None:
            self.user_tag = [random.randrange(2**12-1)
                             for _ in range(len(self.angle_range))]

        # finds common length among allowed sequence-able parameters
        # throws error if two or more sequences of different lengths are input
        try:
            common_length = get_common_length(
                angles=self.angle_range, fps=self.fps_multiple,
                power=self.laser_power_percent, inte=self.inte_time_us,
                fr=self.frame_rate_hz,
                freqs=self.max_range_m, tag=self.user_tag,
                xbin=self.binning, nn=self.nn_level,
                snr=self.snr_threshold,
                rtdc=self.rtd_algorithm_common,
                rtdg=self.rtd_algorithm_grid_mode,
                rtds=self.rtd_algorithm_stripe_mode,
                hdr=self.hdr_threshold,
                hdrl=self.hdr_laser_power_percent,
                hdri=self.hdr_inte_time_us,
            )
        except cobex.ScanPatternSizeError as e:
            raise e
        except Exception as e:
            raise cobex.ScanPatternSizeError(
                'Error when getting common length of scan parameters. '
                'Make sure all the parameter lists are length=1 or of '
                'all equal length') from e

        # convert any int / float to sequence with common length, performing
        # limit checks
        # Changed 20210125, these are saved off as the instance variables
        # so the API can query
        self.angle_range = cast_to_sequence(
            self.angle_range, common_length)
        self.fps_multiple = np.clip(self.fps_multiple, 1, None).astype(int)
        self.fps_multiple = cast_to_sequence(
            self.fps_multiple, common_length, func=FpsMultipleOv)
        self.laser_power_percent = cast_to_sequence(
            self.laser_power_percent, common_length)
        self.inte_time_us = cast_to_sequence(
            self.inte_time_us, common_length, func=InteTimeIdxMappedOv)
        self.frame_rate_hz = cast_to_sequence(
            self.frame_rate_hz, common_length, func=FrameRateOv)

        # Virtual Sensors cannot have mixed max_range_idx
        self.max_range_m = cast_to_sequence(
            self.max_range_m, common_length, func=MaxRangeIdxMappedOv)
        if len(set(self.max_range_m)) != 1:
            raise cobex.ScanPatternValueError(
                "Only one max_range_m value may be used "
                "in a single RandomAccessScanning"
                f" call but was called with {set(self.max_range_m)}")

        self.user_tag = cast_to_sequence(self.user_tag, common_length)
        self.binning = cast_to_sequence(
            self.binning, common_length, func=BinningOv)
        self.nn_level = cast_to_sequence(
            self.nn_level, common_length, func=NnLevelOv)
        self.rtd_algorithm_common = cast_to_sequence(self.rtd_algorithm_common, common_length)
        self.rtd_algorithm_grid_mode = cast_to_sequence(self.rtd_algorithm_grid_mode, common_length)
        self.rtd_algorithm_stripe_mode = cast_to_sequence(self.rtd_algorithm_stripe_mode, common_length)
        self.snr_threshold = cast_to_sequence(
            self.snr_threshold, common_length, func=SnrThresholdBv)

        self.hdr_laser_power_percent = cast_to_sequence(
            self.hdr_laser_power_percent, common_length)
        self.hdr_inte_time_us = cast_to_sequence(
            self.hdr_inte_time_us, common_length, func=InteTimeIdxMappedOv)

        # Check the laser power percent values. Have to do this separately since
        # there is some weird issue with serializing the LaserPowerPercentMappedOv
        # class over Pyro. Likely something to do with the factory.
        _ = [self.laser_power_mapped_cls(x) for x in self.laser_power_percent]
        _ = [self.laser_power_mapped_cls(x) for x in self.hdr_laser_power_percent]

        # Dictionary of the common length adjusted parameters that the
        # API can query all at once.
        self.ras_scan_parameters = {
            # do list comprehension on angle range so it gets the API
            # setting and not the fix_angle_triplet return
            # round to required precision
            'angle_range': [x for x in self.angle_range],
            'fps_multiple': [int(x.value) for x in self.fps_multiple],
            'laser_power_percent': self.laser_power_percent,
            'inte_time_us': [x.value for x in self.inte_time_us],
            'frame_rate_hz': [x.value for x in self.frame_rate_hz],
            'max_range_m': [x.value for x in self.max_range_m],
            'user_tag': self.user_tag,
            'binning': [x.value for x in self.binning],
            'nn_level': [x.value for x in self.nn_level],
            'snr_threshold': [x.value for x in self.snr_threshold],
            'interleave': self.interleave,
            'dsp_mode': self.dsp_mode.key,  # return the int to the API
            'hdr_threshold': self.hdr_threshold,
            'hdr_laser_power_percent': self.hdr_laser_power_percent,
            'hdr_inte_time_us': [x.value for x in self.hdr_inte_time_us],
        }

        # Need to determine the total number of ROIs for sorting the scan
        self.total_rois = 0

        for idx, virtual_sensor in enumerate(self.angle_range):
            new_triplet = fix_angle_triplet(virtual_sensor)
            self.angle_range[idx] = new_triplet
            virtual_sensor_angle_array = np.arange(*new_triplet)
            virtual_sensor_rois = np.tile(virtual_sensor_angle_array,
                                          self.fps_multiple[idx].value)
            self.total_rois += len(virtual_sensor_rois)

        # Expands the virtual_sensor (start,stop,step) tuples into lists of
        # orders ,rows, etc for input into SensorHead.apply_settings
        # Only create as many instances as necessary to avoid writing zeros
        temp_virtual_sensormeta = []
        for idx, (
                ang, ut, fps, pi, inte, bx, nn,
                ri, snr, algo_comm, algo_grid, algo_stripe,
                fr, hdrl, hdri,
        ) in enumerate(
                zip(self.angle_range,
                    self.user_tag,
                    self.fps_multiple,
                    self.laser_power_percent,
                    self.inte_time_us,
                    self.binning,
                    self.nn_level,
                    self.max_range_m,
                    self.snr_threshold,
                    self.rtd_algorithm_common,
                    self.rtd_algorithm_grid_mode,
                    self.rtd_algorithm_stripe_mode,
                    self.frame_rate_hz,

                    self.hdr_laser_power_percent,
                    self.hdr_inte_time_us)
        ):
            # The frame rate is not changeable for the HDR retry so
            # the hdr_inte_time_us must be less than or equal to
            # the inte_time_us
            if hdri > inte:
                raise cobex.ScanPatternValueError(
                    'HDR integration time must be less than the integration '
                    f'time setting but inte_time_us = {inte} and '
                    f'hdr_inte_time_us = {hdri}'
                )

            # Virtual Sensor angle tuple is (start, stop, step)
            # Multiply by -1 to ensure negative angles point to the ground
            single_set_angles = np.arange(*ang) * -1

            # This needs to be mapped to order, s_row for the metadata since we
            # need to calculate s_rows, n_rows, n_rois for each Virtual Sensor
            orders, s_rows = self.roi_mapping(
                angles=single_set_angles,
                roi_rows=self.roi_rows, trim_duplicates=True)

            orders_virtual_sensor = np.tile(orders, fps.value)
            s_rows_virtual_sensor = np.tile(s_rows, fps.value)

            self.full_appset_dict['orders'].extend(list(orders_virtual_sensor))
            self.full_appset_dict['s_rows'].extend(list(s_rows_virtual_sensor))

            # The other parameters need to be multiplied
            # if there is an FPS multiplier
            len_vs = len(orders_virtual_sensor)
            self.full_appset_dict['inte_time_s'].extend(
                [InteTimeIdxMappedOv(inte.value).mapped] * len_vs)
            self.full_appset_dict['ci_v'].extend(
                [self.laser_power_mapped_cls(pi).mapped] * len_vs)
            self.full_appset_dict['mod_freq_int'].extend(
                [MaxRangeIdxMappedOv(ri.value).mapped] * len_vs)
            self.full_appset_dict['virtual_sensor_bitmask'].extend([0b1 << idx] * len_vs)
            self.full_appset_dict['frame_rate_hz'].extend([fr.value] * len_vs)
            self.full_appset_dict['binning'].extend([bx] * len_vs)

            self.full_appset_dict['hdr_ci_v'].extend(
                [self.laser_power_mapped_cls(hdrl).mapped] * len_vs)
            self.full_appset_dict['hdr_inte_time_s'].extend(
                [InteTimeIdxMappedOv(hdri.value).mapped] * len_vs)

            # Start stop flags
            single_set_start_stop = np.zeros_like(orders)
            # size of a single flag is 4
            single_set_start_stop[0] |= 0b01 << (idx * 4)
            single_set_start_stop[-1] |= 0b10 << (idx * 4)
            ss_virtual_sensor = np.tile(single_set_start_stop, fps.value)

            self.full_appset_dict['start_stop_flags'].extend(list(ss_virtual_sensor))

            if self.dsp_mode == DspMode.CAMERA_MODE:
                # PerVirtualSensorMetadata needs total number of rows and total number of rois
                # Added +4 to account for row cal
                n_rows = min(480, max(s_rows) - min(s_rows) + self.roi_rows + 4)
                n_rois = len(orders)
            elif self.dsp_mode == DspMode.LIDAR_MODE:
                n_rows = self.roi_rows
                n_rois = 1

            # PerVirtualSensorMetadata for one virtual_sensor
            # Added a -2 to start row to account for row cal
            temp_virtual_sensormeta.append(PerVirtualSensorMetadata.build(
                user_tag=ut,
                binning=bx,
                s_rows=max(0, min(s_rows) - 2), n_rows=n_rows, n_rois=n_rois,
                rtd_algorithm_common=algo_comm,
                rtd_algorithm_grid_mode=algo_grid,
                rtd_algorithm_stripe_mode=algo_stripe,
                snr_threshold=snr,
                nn_level=nn,
            ))

            # Set the sorting values
            self.full_sorting.extend([self.total_rois / len_vs * x
                                      for x in range(len_vs)])

        self.full_appset_dict['virtual_sensor_metadata'] = temp_virtual_sensormeta
        self.full_appset_dict['hdr_threshold'] = self.hdr_threshold
        self.full_appset_dict['dsp_mode'] = self.dsp_mode

        # Reuse ROIs
        if self.double_dip:
            self.triple_dip_double_dip()
            self.appset_dict = copy.deepcopy(self.trimmed_appset_dict)
            self.sorting = copy.deepcopy(self.trimmed_sorting)
        else:
            self.appset_dict = copy.deepcopy(self.full_appset_dict)
            self.sorting = copy.deepcopy(self.full_sorting)

        # Check the number of ROIs requested will fit in the scan table
        table_len = len(self.full_appset_dict['orders'])
        if table_len > SCAN_TABLE_SIZE:
            raise cobex.ScanPatternSizeError(
                f'Defined scan table exceeds total available ROI slots. '
                f'{table_len} defined and 512 available. '
                'Reduce the number of Virtual Sensors, the fps_multiple, '
                'or the angle_range_triplet of some Virtual Sensors')

        if self.interleave:
            self.appset_dict = schedule_scan(self.appset_dict, self.sorting)

        if self.dsp_mode == DspMode.CAMERA_MODE:
            # Make sure the start/stop flags are valid in Grid Mode
            self.appset_dict['start_stop_flags'] = flag_equalizer(
                self.appset_dict['start_stop_flags'],
                self.appset_dict['virtual_sensor_bitmask'],
                self.fps_multiple)

            # If we're dipping, R2D wants to know the new number of
            # ROIs per Virtual Sensor
            # Changed to stop after the first stop flag is seen
            for i, fm in enumerate(self.appset_dict['virtual_sensor_metadata']):
                cnt = 0
                for j, b in zip(self.appset_dict['start_stop_flags'],
                                self.appset_dict['virtual_sensor_bitmask']):
                    flag = ((j >> (i*4)) & 0b11) >> 1
                    fb = (b >> i) & 0b1
                    if fb == 1:
                        cnt += 1
                    if flag == 1:
                        fm.n_rois = cnt
                        break
        elif self.dsp_mode == DspMode.LIDAR_MODE:
            # Emit every ROI in Stripe Mode
            self.appset_dict['start_stop_flags'] = (
                [STRIPE_MODE_FLAGS]
                * len(self.appset_dict['start_stop_flags'])
            )

    def triple_dip_double_dip(self):
        """Iterates through the scan table and finds entries that are repeated
        due to them being in more than one Virtual Sensor. If the parameters are
        compatible, the scan entries are combined and the virtual_sensor_bitmask
        updated so that the entry applies to more than one Virtual Sensor.
        This function must be combined with
        flag_equalizer to make sure the flags are set properly.
        """
        new_appset_dict = {k: (v if k in NON_ARRAY_ITEMS
                               else np.asarray(v))
                           for k, v in self.full_appset_dict.items()}

        temp = Counter(self.full_appset_dict['orders'])
        multi_cnt = Counter({k: v for k, v in temp.items() if v > 1})
        # Need to pop them all at the end otherwise
        # the array sizes will be changing
        pop_idx = []

        for k, v in multi_cnt.items():
            loc_idx = np.flatnonzero(
                (np.asarray(self.full_appset_dict['orders']) == k))
            if len(loc_idx) != v:
                raise cobex.ScanPatternSizeError(
                    f'Found {len(loc_idx)} entries in the '
                    f'order table but expected {v}')

            # short circuit if the start rows don't match
            tr = np.asarray(self.full_appset_dict['s_rows'])[loc_idx]
            if len(set(tr)) != 1:
                continue

            # Use the bitmask with the most entries to
            # determine which one to keep.
            # The highest laser output settings will be
            # copied over to the kept ROI.
            # INFO this may cause issues with the check_high_power_scans()
            #    whenever that feature might be enabled

            tb = np.asarray(self.full_appset_dict['virtual_sensor_bitmask'])[loc_idx]
            bit_count = Counter(tb).most_common()

            max_bitmask = [k for (k, v) in bit_count if v == bit_count[0][1]]
            keepers = np.flatnonzero(np.logical_and(
                np.asarray(
                    self.full_appset_dict['virtual_sensor_bitmask']) == max_bitmask[0],
                np.asarray(
                    self.full_appset_dict['orders']) == k)
            )

            # Use the highest power and frame rate from the shared ROIS
            tc = np.asarray(self.full_appset_dict['ci_v'])[loc_idx]
            ti = np.asarray(self.full_appset_dict['inte_time_s'])[loc_idx]
            tf = np.asarray(self.full_appset_dict['frame_rate_hz'])[loc_idx]
            # Use the lowest HDR power for the shared Virtual Sensor ROIs
            hdrp = np.asarray(self.full_appset_dict['hdr_ci_v'])[loc_idx]

            pwr_lvl = (
                (10 * tc) * ti * (tf / 100)
                * (1 / (10 * hdrp))
            )
            high_idx = np.argmax(pwr_lvl)

            # Use the binning from the VGA ROIs to make sure can run full
            # rate
            tb = np.asarray(self.full_appset_dict['binning'])[loc_idx]
            bin_low_idx = np.argmin(tb)

            new_appset_dict['ci_v'][keepers] = tc[high_idx]
            new_appset_dict['inte_time_s'][keepers] = ti[high_idx]
            new_appset_dict['frame_rate_hz'][keepers] = tf[high_idx]
            new_appset_dict['binning'][keepers] = tb[bin_low_idx]
            new_appset_dict['hdr_ci_v'][keepers] = hdrp[high_idx]

            # OR in the bitmask and SSF data
            for lidx in loc_idx:
                new_appset_dict['virtual_sensor_bitmask'][keepers] |= (
                    new_appset_dict['virtual_sensor_bitmask'][lidx])
                new_appset_dict['start_stop_flags'][keepers] |= (
                    new_appset_dict['start_stop_flags'][lidx])
            # Finally, add to the pop list, the indices we will not keep
            pop_idx.extend(list(set(loc_idx) - set(keepers)))

        # Pop out the items we don't want anymore
        for k, v in new_appset_dict.items():
            if k == 'mod_freq_int':
                continue
            elif k not in NON_ARRAY_ITEMS:
                new_appset_dict[k] = [
                    x for i, x in enumerate(v) if i not in pop_idx]

        # This goes through a set operation and needs to be cast to a tuple
        new_appset_dict['mod_freq_int'] = [
            tuple(x) for i, x in enumerate(new_appset_dict['mod_freq_int'])
            if i not in pop_idx]
        self.trimmed_sorting = np.asarray([
            x for i, x in enumerate(self.full_sorting) if i not in pop_idx])

        # Move it over to the instance attribute
        self.trimmed_appset_dict = new_appset_dict
