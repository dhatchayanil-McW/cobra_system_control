"""
file: calibration_data.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file defines how calibration data is stored in the
SPI Flash including register locations and data formats.
It also provides functions to convert between various
numerical representations of the data values.

Note, due to the removal of old calibrations, there are now
chunks of memory that are free to use for other calibration
groups

Free Memory:
0x57 - 0x6f
0x90 - 0xaf
0xb0 - 0xc0
0xea - 0xff
0x110 - 0x3ff
"""
import dataclasses as dc
import hashlib
import json
from numbers import Number
from typing import (Iterable, Union, Sequence,
                    Tuple, Type, Dict, Optional, TypeVar)

import numpy as np

from cobra_system_control import remote
from cobra_system_control.memory_map import M30_SPI_FLASH_MEMORY_MAP

import cobra_system_control.exceptions as cobex

from cobra_system_control.numerical_utilities import FxpFormat
from cobra_system_control.validation_utilities import Descriptor


# used for type hinting
Cal = TypeVar('Cal', bound='CalBase')


@remote.register_for_serialization
class MultiValue:
    """Holds three representations for calibration values:

    vfxp : quantized float value based on the fixed point format
    vdig : integer representation of vfxp in twos-comp format
    vbytes : a byte representation of vdig
    """
    def __init__(self, vfxp: np.ndarray, vdig: np.ndarray, vbytes: bytearray):
        self.vfxp = vfxp
        self.vdig = vdig
        self.vbytes = vbytes

    def __repr__(self):
        return (f'MultiValue(vfxp={self.vfxp}, '
                f'vdig={self.vdig}, '
                f'vbytes={self.vbytes})')


@remote.register_for_serialization
@dc.dataclass(frozen=True)
class CalItem(Descriptor):
    """Defines the format of a calibration entry including
    its address position, fixed point format,
    number of items in the array (most are one),
    number of bytes for the value and the
    number of total bytes for the item.
    """
    addr_offset: int
    fxp_format: FxpFormat
    nitems: int
    nbytes_val: int = dc.field(init=False)
    nbytes_total: int = dc.field(init=False)

    def __post_init__(self):
        object.__setattr__(self, 'nbytes_val',
                           int(np.ceil(self.fxp_format.n_bits / 8)))
        object.__setattr__(self, 'nbytes_total', self.nbytes_val * self.nitems)

    def __set__(self, instance: 'CalData', value):
        raise dc.FrozenInstanceError('Cannot directly write values')

    def __get__(self, inst: 'CalGroup', owner) -> Union['CalItem',
                                                        MultiValue]:
        if not inst:
            return self
        vfxp, vdig, vbytes = [], [], bytearray()
        for i in range(0, self.nitems):
            dat = inst.ba[self.addr_offset + i * self.nbytes_val:
                          self.addr_offset + (i + 1) * self.nbytes_val]
            vfxp.append(self.bytes_to_fixed(dat)[0])
            vdig.append(self.bytes_to_dig(dat)[0])
            vbytes.extend(dat)

        return MultiValue(np.asarray(vfxp), np.asarray(vdig), vbytes)

    def __set_name__(self, owner, name):
        object.__setattr__(self, 'name', f'{name}')

    @property
    def span(self) -> Tuple[int, int]:
        """Returns address of item [start, stop)"""
        return self.addr_offset, self.addr_offset + self.nbytes_total

    def fixed_to_bytes(self, val: np.array) -> bytearray:
        """Converts fixed point value to a bytearray
        based on the fixed point format.
        """
        dval = self.fixed_to_dig(val)
        return self.dig_to_bytes(dval)

    def bytes_to_fixed(self, val: bytearray) -> np.array:
        """Converts a bytearray to a fixed point value
        based on the fixed point format.
        """
        dval = self.bytes_to_dig(val)
        return self.dig_to_fixed(dval)

    def dig_to_bytes(self, val: np.array) -> bytearray:
        """Converts a digital value to a bytearray
        """
        _tmp = bytearray([])
        for v in val:
            _tmp.extend(bytearray(
                int(v).to_bytes(self.nbytes_val, byteorder='big')))
        return _tmp

    def bytes_to_dig(self, val: bytearray) -> np.array:
        """Converts a bytearray to a digital value.
        """
        _tmp = [int.from_bytes(
            val[i * self.nbytes_val:(i + 1) * self.nbytes_val],
            byteorder='big', signed=False)
                for i in range(len(val) // self.nbytes_val)]
        return np.asarray(_tmp).astype(np.int64)

    def dig_to_fixed(self, val: np.array) -> np.array:
        """Converts a digital value to a fixed point value based
        on the fixed point format.
        """
        if self.fxp_format.signed:
            val = np.where(val < 2 ** (self.fxp_format.n_bits - 1),
                           val,
                           val - 2 ** self.fxp_format.n_bits)
            return val * 2 ** (-self.fxp_format.n_frac)
        else:
            return val * 2 ** (-self.fxp_format.n_frac)

    def fixed_to_dig(self, val: np.array) -> np.array:
        """Converts fixed point value to a digital value
        based on the fixed point format.
        """
        qval = self.quantize(val)
        dval = (qval * 2 ** self.fxp_format.n_frac
                % 2 ** self.fxp_format.n_bits)
        return dval.astype(np.int64)

    def quantize(self, val: np.array) -> np.array:
        """Quantizes each floating point input value to its fixed-point value

        with the specified number of total bits (n_bits) and fractional bits
        (n_frac). For example, vals = -1.749 with n_bits = 4 and n_frac = 2
        (i.e. the format is s1.2) is quantized to -1.75. By default,
        this function quantizes using convergent rounding to even
        (aka banker's rounding or numpy rounding) and protects from
        rounding overflow by clipping to the fixed-point value's limits.
        For example, 1.9999 with the same format (s1.2) is quantized to
        1.75 because 2 is not allowed.
        """
        val = np.asarray(val)
        quant = np.round(val * 2 ** self.fxp_format.n_frac)
        if self.fxp_format.signed:
            clip = np.clip(quant, -2 ** (self.fxp_format.n_bits - 1),
                           2 ** (self.fxp_format.n_bits - 1) - 1)
        else:
            clip = np.clip(quant, 0, 2 ** self.fxp_format.n_bits - 1)
        return clip / 2 ** self.fxp_format.n_frac

    def __str__(self):
        return (
            'CalItem('
            f'addr_offset={self.addr_offset:#04x}, '
            f'nbytes_val={self.nbytes_val}, '
            f'nbytes_total={self.nbytes_total}, '
            f'fxp_format='
            f'FxpFormat(signed={self.fxp_format.signed}, '
            f'n_bits={self.fxp_format.n_bits}, '
            f'n_frac={self.fxp_format.n_frac}), '
            ')\n'
        )


@remote.register_for_serialization
class CalGroup:
    """ Organizes ``CalItem`` objects into groups

    Every group has a hash ``CalItem``. This item is computed using all other
    ``CalItem`` in the group. The hash protects against corrupted data. For
    example, if somehow 1 coefficient in A2A calibration is changed, the
    ``is_valid`` property will return False, indicating a problem with the
    loaded calibration.

    Each group must update all of its ``CalItem`` entries simultaneously. This
    is using the ``update_group`` method, below, which will alter the backing
    bytearray for the calibration.
        """

    ba: bytearray
    hash: CalItem = None

    def __init__(self, ba: bytearray):
        self.ba = ba

    def update_group(
            self,
            vfxp: Optional[Dict[str, Union[Number, Iterable, np.ndarray]]] = None,
            vdig: Optional[Dict[str, Union[int, Iterable, np.ndarray]]] = None,
            vbytes: Optional[Dict[str, bytearray]] = None):
        """Updates the backing bytearray in place with the values specified

        A calibration error will be raised if values for all the group
        ``CalItem`` objects are not specified.

        If you are writing fixed point numbers, pass the dict to vfxp
        If you are writing digital values, pass the dict to vdig
        If you are writing bytearrays directly, pass the dict to vbytes

        Be sure to look at the fixed point formats of the values you are
        writing to make sure they won't get clipped.

        Additionally, the kwarg names can be found in the CalibrationData class
        in calibration_data.py

        Example update a2a cal_data items when performing a calibration:

            wave = 905
            coeffs = (-3.6, -1, -4.6e-4, -3.9e-5)
            cal_data.a2a.update_group(vfxp=dict(
                wave_nm=wave,
                ps_c_0=coeffs[0]
                ps_c_1=coeffs[1]
                ps_c_2=coeffs[2]
                ps_c_3=coeffs[3]
            ))
            # ... now write the cal_data to the spi flash
        """

        cls = self.__class__
        in_set = set()
        if vfxp is not None:
            in_set.update(vfxp.keys())
        if vdig is not None:
            in_set.update(vdig.keys())
        if vbytes is not None:
            in_set.update(vbytes.keys())

        req_set = set(cls.item_names())

        if in_set != req_set:
            raise cobex.CalibrationError(
                f'Updating calibration data for group: {cls.__name__} '
                f'requires {req_set} but got {in_set}'
            )

        # convert all dictionaries to {name: bytearray}
        # 1. no change needed on vbytes
        vbytes = vbytes or {}

        # 2. convert digital values to bytearray
        if vdig is not None:
            for name, value in vdig.items():
                desc: CalItem = getattr(cls, name)
                if isinstance(value, int):
                    new_val = np.array([value]).astype(np.int64)
                else:
                    new_val = np.asarray(value).astype(np.int64)
                vbytes[name] = desc.dig_to_bytes(new_val)

        # 3. convert fixed point values to bytearray
        if vfxp is not None:
            for name, value in vfxp.items():
                desc: CalItem = getattr(cls, name)
                if isinstance(value, (float, int)):
                    new_value = np.array([float(value)])
                else:
                    new_value = np.asarray(value).astype(float)
                vbytes[name] = desc.fixed_to_bytes(new_value)

        # we need to pad in case our bytearray is short, before computing hash
        for name in vbytes.keys():
            desc: CalItem = getattr(cls, name)
            padding = desc.nbytes_total - len(vbytes[name])
            vbytes[name] += bytearray(padding)

        # generate hash based on values other than hash
        values_ba = bytearray()
        for name in cls.item_names():
            values_ba.extend(vbytes[name])
        hash_desc = getattr(cls, 'hash')
        vbytes['hash'] = get_cal_hash(values_ba,
                                      num_vals=hash_desc.nbytes_total)

        # copy the bytearray, update i
        for name, val in vbytes.items():
            desc: CalItem = getattr(cls, name)
            self.ba[desc.addr_offset: desc.addr_offset + desc.nbytes_total] = val

    @property
    def is_valid(self) -> bool:
        """Returns true if the group data is loaded with a matching hash"""
        if not self.is_loaded:
            return False

        vals = bytearray()

        descriptors = (x.__get__(self, None) for x in self.__class__.params())
        for d in descriptors:
            vals.extend(d.vbytes)

        return (self.hash.vbytes
                == get_cal_hash(vals, self.__class__.hash.nbytes_total))

    @property
    def is_loaded(self) -> bool:
        """Indicates whether data has been loaded (not all 0x00 and 0xFF) for
        the given group

        The return statement could be made more robust.
        """
        cls = self.__class__
        descriptors = (cls.hash, *cls.params())

        return not all(x.__get__(self, None).vbytes[0]
                       in (0xFF, 0x00) for x in descriptors)

    @classmethod
    def item_names(cls) -> Sequence[str]:
        """Excludes the hash"""
        return tuple(
            # .items() iterator works on this dict fine.
            # pylint: disable-next=no-member
            name for name, cd in cls.__dict__.items()
            if isinstance(cd, CalItem)
            and name != 'hash'
        )

    @classmethod
    def params(cls) -> Sequence[CalItem]:
        """All ``CalItem``, excluding hash"""
        return tuple(
            # .items() iterator works on this dict fine.
            # pylint: disable-next=no-member
            cd for name, cd in cls.__dict__.items()
            if name != 'hash' and isinstance(cd, CalItem))

    @classmethod
    def all_items(cls) -> Sequence[CalItem]:
        """All ``CalItem``, including hash"""
        return (cls.hash, *cls.params())

    @classmethod
    def __init_subclass__(cls, **kwargs):
        if cls.hash is None:
            raise TypeError(
                f'{cls.__name__} must define a hash class descriptor')
        remote.register_for_serialization(cls)
        return cls

    def __str__(self):
        cls = self.__class__
        valid_str = '(OK)' if self.is_valid else '(INVALID)'
        header = f'--- {" ".join([cls.__name__, valid_str]):<20}'
        if self.is_valid:
            header += f' {"addr off":<10} {"nbytes":<10} {"vdig":<11} {"vfxp":<11}'
            for item in cls.all_items():
                item_name = item.name
                multi_value = item.__get__(self, None)
                header += f'\n    {item_name:<20} {item.addr_offset:<10} {item.nbytes_total:<10} {multi_value.vdig[0]:<11} {multi_value.vfxp[0]:8.8f}'
        header += '\n'
        return header

    def __dict__(self) -> dict:
        cls = self.__class__
        group_dict = {}

        for item in cls.all_items():
            item_name = item.name
            multi_value = item.__get__(self, None)

            if (item.fxp_format.n_frac == 0):
                group_dict[item_name] = int(multi_value.vdig[0])
            else:
                group_dict[item_name] = multi_value.vfxp[0]

        return group_dict


CI = CalItem


class InfoGroup(CalGroup):
    """Defines calibration items for the
    sensor serial number. A legacy group kept
    for backwards compatibility. Superseded
    by SensorInfoGroup
    """
    hash =      CI(0x000, FxpFormat(False, 16, 0), 1)
    sensor_sn = CI(0x002, FxpFormat(False, 16, 0), 1)


class DynGroup(CalGroup):
    """Defines calibration items for the
    dynamic range calibration
    """
    hash =          CI(0x006, FxpFormat(False, 16, 0), 1)
    pga_gain =      CI(0x008, FxpFormat(False, 8, 0), 1)
    doff_diff_adu = CI(0x009, FxpFormat(False, 8, 0), 1)


class CamGroup(CalGroup):
    """Defines calibration items for the intrinsic camera
    calibration.
    """
    hash = CI(0x010, FxpFormat(False, 16, 0), 1)
    fx = CI(0x012, FxpFormat(False, 24, 14), 1)
    fy = CI(0x015, FxpFormat(False, 24, 14), 1)
    cx = CI(0x018, FxpFormat(False, 24, 14), 1)
    cy = CI(0x01b, FxpFormat(False, 24, 14), 1)
    k1 = CI(0x01e, FxpFormat(True, 24, 23), 1)
    k2 = CI(0x021, FxpFormat(True, 24, 23), 1)
    k3 = CI(0x024, FxpFormat(True, 24, 23), 1)
    p1 = CI(0x027, FxpFormat(True, 24, 23), 1)
    p2 = CI(0x02a, FxpFormat(True, 24, 23), 1)


class A2AGroup(CalGroup):
    """Defines calibration items for the Angle2Angle
    calibration that maps LCM steering order to
    imager start row
    """
    hash =    CI(0x030, FxpFormat(False, 16, 0), 1)
    wave_nm = CI(0x032, FxpFormat(False, 16, 6), 1)
    ps_c_0 = CI(0x037, FxpFormat(True, 32, 14), 1)
    ps_c_1 = CI(0x03b, FxpFormat(True, 32, 20), 1)
    ps_c_2 = CI(0x03f, FxpFormat(True, 32, 20), 1)
    ps_c_3 = CI(0x043, FxpFormat(True, 32, 32), 1)


class CalibrationSystemVersion(CalGroup):
    """Defines the system version used when
    measuring the saved calibration values.
    """
    hash = CI(0x50, FxpFormat(False, 32, 0), 1)
    major_version = CI(0x54, FxpFormat(False, 8, 0), 1)
    minor_version = CI(0x55, FxpFormat(False, 8, 0), 1)
    patch_version = CI(0x56, FxpFormat(False, 8, 0), 1)


class RangeGroup0807(CalGroup):
    """Defines calibration items for Range Calibration for
    the (8,7) modulation frequency pair. Range Calibration
    ensures that 1meter measures 1meter.
    """
    hash =                 CI(0x070, FxpFormat(False, 16, 0), 1)
    pw_laser_f0_shrink =   CI(0x072, FxpFormat(False, 8, 0), 1)
    pw_laser_f0_expand =   CI(0x073, FxpFormat(False, 8, 0), 1)
    dlay_laser_f0_coarse = CI(0x074, FxpFormat(False, 8, 0), 1)
    dlay_laser_f0_fine =   CI(0x075, FxpFormat(False, 8, 0), 1)
    dlay_mg_f0_coarse =    CI(0x076, FxpFormat(False, 8, 0), 1)
    dlay_mg_f0_fine =      CI(0x077, FxpFormat(False, 8, 0), 1)
    pw_laser_f1_shrink =   CI(0x078, FxpFormat(False, 8, 0), 1)
    pw_laser_f1_expand =   CI(0x079, FxpFormat(False, 8, 0), 1)
    dlay_laser_f1_coarse = CI(0x07a, FxpFormat(False, 8, 0), 1)
    dlay_laser_f1_fine =   CI(0x07b, FxpFormat(False, 8, 0), 1)
    dlay_mg_f1_coarse =    CI(0x07c, FxpFormat(False, 8, 0), 1)
    dlay_mg_f1_fine =      CI(0x07d, FxpFormat(False, 8, 0), 1)
    sync_laser_lvds_mg =   CI(0x07e, FxpFormat(False, 8, 0), 1)


class RangeGroup0908(CalGroup):
    """Defines calibration items for Range Calibration for
    the (9,8) modulation frequency pair. Range Calibration
    ensures that 1meter measures 1meter.
    """
    hash =                 CI(0x080, FxpFormat(False, 16, 0), 1)
    pw_laser_f0_shrink =   CI(0x082, FxpFormat(False, 8, 0), 1)
    pw_laser_f0_expand =   CI(0x083, FxpFormat(False, 8, 0), 1)
    dlay_laser_f0_coarse = CI(0x084, FxpFormat(False, 8, 0), 1)
    dlay_laser_f0_fine =   CI(0x085, FxpFormat(False, 8, 0), 1)
    dlay_mg_f0_coarse =    CI(0x086, FxpFormat(False, 8, 0), 1)
    dlay_mg_f0_fine =      CI(0x087, FxpFormat(False, 8, 0), 1)
    pw_laser_f1_shrink =   CI(0x088, FxpFormat(False, 8, 0), 1)
    pw_laser_f1_expand =   CI(0x089, FxpFormat(False, 8, 0), 1)
    dlay_laser_f1_coarse = CI(0x08a, FxpFormat(False, 8, 0), 1)
    dlay_laser_f1_fine =   CI(0x08b, FxpFormat(False, 8, 0), 1)
    dlay_mg_f1_coarse =    CI(0x08c, FxpFormat(False, 8, 0), 1)
    dlay_mg_f1_fine =      CI(0x08d, FxpFormat(False, 8, 0), 1)
    sync_laser_lvds_mg =   CI(0x08e, FxpFormat(False, 8, 0), 1)


class SensorInfoGroup(CalGroup):
    """Provides extra system information.
    For the SN Prefixes, these are typically AA<X> but can
    go up to ZZZ.

    To write to spi flash, convert to an integer with:

        int('zzz'.encode('utf-8').hex(), base=16)
    or
        int('aar'.encode('utf-8').hex(), base=16)

    To decode from the value in the spiflash:

        int_val = <x>_prefix.vdig[0]
        str_val = bytearray.fromhex(hex(int_val)[2::]).decode('utf-8')
        # The [2::] is needed to get rid of the 0x prefix

    SN is a repeat of the first group but add the prefix here
    """
    hash = CI(0x0d0, FxpFormat(False, 32, 0), 1)
    prefix = CI(0x0d4, FxpFormat(False, 24, 0), 1)
    sn = CI(0x0d7, FxpFormat(False, 16, 0), 1)
    rev = CI(0x0d9, FxpFormat(False, 8, 0), 1)


class RxPcbInfoGroup(CalGroup):
    """Provides extra information for the RX PCB

    For the SN Prefixes, these are typically AA<X> but can
    go up to ZZZ.

    To write to spi flash, convert to an integer with:

        int('zzz'.encode('utf-8').hex(), base=16)
    or
        int('aar'.encode('utf-8').hex(), base=16)

    To decode from the value in the spiflash:

        int_val = <x>_prefix.vdig[0]
        str_val = bytearray.fromhex(hex(int_val)[2::]).decode('utf-8')
        # The [2::] is needed to get rid of the 0x prefix
    """
    hash = CI(0x0e0, FxpFormat(False, 32, 0), 1)
    prefix = CI(0x0e4, FxpFormat(False, 24, 0), 1)
    sn = CI(0x0e7, FxpFormat(False, 16, 0), 1)
    rev = CI(0x0e9, FxpFormat(False, 8, 0), 1)


class RangeCalTemperatureGroup(CalGroup):
    """Defines calibration items related to the Range-Temperature-
    Calibration that corrects the range based on the laser
    temperature and VLDA voltage.
    """
    hash = CI(0x100, FxpFormat(False, 32, 0), 1)
    rng_offset_mm_0807 = CI(0x104, FxpFormat(True, 16, 5), 1)
    mm_per_volt_0807 = CI(0x106, FxpFormat(True, 16, 12), 1)
    mm_per_celsius_0807 = CI(0x108, FxpFormat(False, 16, 7), 1)
    rng_offset_mm_0908 = CI(0x10a, FxpFormat(True, 16, 5), 1)
    mm_per_volt_0908 = CI(0x10c, FxpFormat(True, 16, 12), 1)
    mm_per_celsius_0908 = CI(0x10e, FxpFormat(False, 16, 7), 1)


class GroupWrapper(Descriptor):
    """Wrapper descriptor class that wraps a group
    when adding to a CalData dataclass
    """
    def __init__(self, group_type):
        self.group_type = group_type

    def __get__(self, instance, owner) -> Union['GroupWrapper', CalGroup]:
        if instance is None:
            return self
        return self.group_type(instance.ba)


@remote.register_for_serialization
class CalBase:
    """Defines a base class for a system for a group of calibration
    groups that are valid for that system.

    The groups that make up a class that inherits CalBase are available
    from a SPI Flash register location as defined in
    /resources/periphs/m<25/30>_spi_flash_map.yml.
    """

    ADDRESS_BASE = None
    MAX_OFFSET = None

    def __init__(self, ba: bytearray):
        if len(ba) < self.size_bytes():
            raise cobex.CalibrationSizeError(
                f'bytearray is too short, has {len(ba)} elements '
                f'but needs {self.size_bytes()}')
        self.ba = ba

    @classmethod
    def groups(cls) -> Sequence[Type[CalGroup]]:
        return tuple(d.group_type for d in cls.__dict__.values()
                     if isinstance(d, GroupWrapper))

    @classmethod
    def size_bytes(cls) -> int:
        memory_ends = []
        for group in cls.groups():
            for item in group.all_items():
                memory_ends.append(item.addr_offset + item.nbytes_total)
        return max(memory_ends)

    @classmethod
    def empty(cls, default_val: int = 0):
        """Returns ``CalBase`` instance with an underlying bytearray of
        ``default_val``"""
        return cls(bytearray([default_val] * cls.size_bytes()))

    def __str__(self):
        s = ''
        for g in filter(lambda x: isinstance(x, GroupWrapper),
                        self.__class__.__dict__.values()):
            s += str(g.__get__(self, None))
        return s

    @classmethod
    def __init_subclass__(cls, **kwargs):
        if cls.ADDRESS_BASE is None:
            raise AttributeError(
                'Subclasses of CalBase must define ADDRESS_BASE')
        if cls.MAX_OFFSET is None:
            raise AttributeError(
                'Subclasses of CalBase must define MAX_OFFSET')

        for g in cls.groups():
            for i in g.all_items():
                if (i.span[0] < 0) or ((i.span[1] - 1) > cls.MAX_OFFSET):
                    raise AttributeError(
                        f'item {i.name} spans addresses '
                        f'[{i.span[0]}, {i.span[1]}), but the min and max '
                        f'allowed offsets are 0 and {cls.MAX_OFFSET}'
                    )


@remote.register_for_serialization
class CalData(CalBase):
    """Defines the set of calibration groups used in M25 and M30.
    These groups are all available in a single SPI Flash fast read.
    """
    ADDRESS_BASE = M30_SPI_FLASH_MEMORY_MAP.calibration.addr_base
    MAX_OFFSET = M30_SPI_FLASH_MEMORY_MAP.calibration.fields['_last_word_'].offset

    info = GroupWrapper(InfoGroup)
    dyn = GroupWrapper(DynGroup)
    cam = GroupWrapper(CamGroup)
    a2a = GroupWrapper(A2AGroup)

    cal_version = GroupWrapper(CalibrationSystemVersion)

    range0807 = GroupWrapper(RangeGroup0807)
    range0908 = GroupWrapper(RangeGroup0908)

    sensor_info = GroupWrapper(SensorInfoGroup)
    rx_pcb_info = GroupWrapper(RxPcbInfoGroup)

    range_tmp = GroupWrapper(RangeCalTemperatureGroup)


def get_cal_hash(bvals: bytearray, num_vals=2) -> bytearray:
    """Calculates a SHA256 from any number of args, returning
    2-byte LSB

    When starring a single bytearray, the result of the a in args
    iterator is an int.
    When starring a tuple of bytearrays, the result of the a in args
    iterator is bytearray
    """
    if not isinstance(bvals, bytearray):
        raise TypeError(f'Item fed to get_cal_hash must be a '
                        f'bytearray but is {type(bvals)}')
    h = hashlib.sha256()
    h.update(bvals)   # b"".join(args))
    # Take the last 2 bytes
    return bytearray(h.digest()[-num_vals::])


class CalDataEncoder(json.JSONEncoder):
    """Dumps the calibration data to a JSON file.
    """
    def default(self, o):
        if isinstance(o, (CalData, )):
            ret = {}
            for gw in filter(lambda x: isinstance(x, GroupWrapper),
                             o.__class__.__dict__.values()):
                group = gw.__get__(o, None)
                ret[group.__class__.__name__] = group
        elif isinstance(o, CalGroup):
            ret = {}
            for item in o.all_items():
                mv = item.__get__(o, None)
                ret[item.name] = mv
            return ret
        elif isinstance(o, MultiValue):
            ret = {
                "vfxp": o.vfxp,
                "vdig": o.vdig,
                "vbytes": o.vbytes,
            }
        elif isinstance(o, np.ndarray):
            ret = o.tolist()
        elif isinstance(o, bytearray):
            ret = list(o)
        else:
            ret = json.JSONEncoder.default(self, o)
        return ret
