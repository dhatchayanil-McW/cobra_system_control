"""
file: memory_map.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines a memory map yaml parser and utilities
to read and write registers in the FPGA.

At the end of this file, paths are defined for the YAML memory maps to use for
each system configuration.
Here, PLECO refers to the legacy name for the GPixel GTOF0503.
"""
import copy
import math
import os
from pathlib import Path
from typing import Any, Callable, Dict, List, NewType, Optional, Tuple, Union
import yaml

import cobra_system_control.exceptions as cobex


ReadCallbackRet = NewType('ReadCallbackRet', Any)
WriteCallbackRet = NewType('WriteCallbackRet', Any)
WriteFieldsRet = Union[WriteCallbackRet, Dict[str, WriteCallbackRet]]
MnemonicOrData = Union[str, int]
ReadFieldsRetForOneField = Union[
    MnemonicOrData,
    Tuple[MnemonicOrData, ReadCallbackRet],
]
ReadFieldsRet = Union[
    ReadFieldsRetForOneField,
    Dict[str, ReadFieldsRetForOneField],
]


class MemoryMap:
    """A class that encapsulates a memory map comprised of several peripherals.
    The map is defined in a YAML file, which is parsed on instantiation.
    """

    cached_maps = {}
    map_file_path = None

    def __init__(
        self,
        map_yaml: str,
        array_rep_limit: Optional[int] = None,
        ignore_alignment_error: bool = False,
    ) -> None:
        """Creates a MemoryMap object from the given YAML file. To the extent
        that arrays are defined in the YAML, the number of iterated fields can
        be limited to reduce memory usage and run time.
        """
        self.yaml_params = None
        self.map_yaml = map_yaml
        self.parse_map_yaml(
            map_yaml,
            array_rep_limit=array_rep_limit,
            ignore_alignment_error=ignore_alignment_error,
        )
        self.word_size = self.yaml_params['WORD_SIZE']
        self.word_mask = 2 ** self.word_size - 1
        self.bytes_per_word = self.word_size // 8

    def parse_map_yaml(
        self,
        map_yaml: str,
        array_rep_limit: Optional[int] = None,
        ignore_alignment_error: bool = False,
    ) -> None:
        """Parses the memory map YAML file. To the extent that arrays are
        defined in the YAML, the number of iterated fields can be limited to
        reduce memory usage and run time.
        """
        with open(map_yaml, 'r', encoding='utf8') as f:
            yaml_data = yaml.safe_load(f)

        # PARAMS is a special entry that doesn't correspond to a peripheral
        self.yaml_params = yaml_data.get('PARAMS', {
            'WORD_SIZE': 32,
            'BLOCK_SIZE': 1,
            'ADDR_SIZE': 32,
        })

        self.periphs = {}
        for p, p_dict in yaml_data.items():
            if p == 'PARAMS':
                continue
            else:
                self.periphs[p] = MemoryMapPeriph(
                    self.yaml_params,
                    name=p,
                    **p_dict,
                    map_yaml_path=os.path.dirname(map_yaml),
                    array_rep_limit=array_rep_limit,
                    ignore_alignment_error=ignore_alignment_error,
                )

        for name, periph in self.periphs.items():
            setattr(self, name, periph)

    @classmethod
    def from_cache(cls, map_yaml: Optional[str] = None, **kwargs) -> 'MemoryMap':
        """Returns a *copy* of a MemoryMap object from a cache, creating the
        object if it doesn't exist yet. The cache can be used to avoid
        construction and initialization time since the MemoryMap YAML contents
        are intended to be read-only.

        Call from_cache() 1) with cls as MemoryMap and with the map_yaml arg
        or 2) with cls as a subclass of MemoryMap and map_file_path defined as
        a class attribute of cls. In the latter case, if you don't like the
        default constructor of the subclass, then call the constructor yourself
        and don't use the cache. That is, without over-engineering the cache,
        the cache shall contain only instances with default constructor args.

        Example using MemoryMap:
            ```
            import MemoryMap
            map = MemoryMap.from_cache('/path/to/the/map.yml')
            ```
        Example using SpecificMemoryMap, a subclass of MemoryMap:
            ```
            import MemoryMap
            class SpecificMemoryMap(MemoryMap): ...
            map = SpecificMemoryMap.from_cache()
            ```
        """
        if map_yaml:
            map_file = map_yaml
            obj = cls(map_file, **kwargs)
        else:
            map_file = cls.map_file_path
            obj = cls(**kwargs)
        if map_file not in MemoryMap.cached_maps.keys():
            MemoryMap.cached_maps[map_file] = obj
        return copy.deepcopy(MemoryMap.cached_maps[map_file])


class MemoryMapPeriph:
    """A class that defines one of the peripherals in a MemoryMap. A
    MemoryMapPeriph has several fields and methods for querying and
    reading/writing them.
    """
    def __init__(self, yaml_params: dict, **kwargs):
        self.fields = {}
        self.write_callback = None
        self.read_callback = None
        self.readdata_callback = lambda x: x
        self.word_size = yaml_params['WORD_SIZE']
        self.word_mask = 2 ** self.word_size - 1
        self.bytes_per_word = self.word_size // 8

        # Attributes that come from the yaml
        self.name = None
        self.addr_base = None
        self.n_blocks = None

        # Get fields info from standalone yaml if supported. The values of
        # 'fields_file' attributes are relative to the location of the YAML
        # file containing these 'fields_file' attributes.
        if 'fields_file' in kwargs:
            if 'fields' in kwargs:
                msg = (f"Both 'fields_file' and 'fields' are specified for "
                       f"MemoryMapPeriph '{kwargs['name']}'.")
                raise cobex.MemoryMapError(msg)
            map_yaml = os.path.abspath(os.path.join(
                kwargs['map_yaml_path'],
                kwargs['fields_file'],
            ))
            with open(map_yaml, 'r', encoding='utf8') as f:
                yaml_data = yaml.safe_load(f)
            kwargs['fields'] = yaml_data

        # Init object
        for attr, val in kwargs.items():
            if attr == 'fields':
                self.fields.update(
                    {f: MemoryMapField(yaml_params, name=f, **f_dict)
                     for f, f_dict in val.items()}
                )
            elif attr == 'arrays':
                for arr in val:
                    # arr is a dict object describing a memory array
                    array_rep_limit = kwargs.get('array_rep_limit', None)
                    array_rep_limit = array_rep_limit or arr['rep']
                    for i in range(array_rep_limit):
                        offset_add = arr['offset'] + i * arr['step']
                        for f, f_dict in arr['fields'].items():
                            name = '{}_{:d}'.format(f, i)
                            f_dict_cp = f_dict.copy()
                            f_dict_cp['offset'] = offset_add + f_dict['offset']
                            self.fields[name] = MemoryMapField(
                                yaml_params,
                                name=name,
                                **f_dict_cp,
                            )
            else:
                setattr(self, attr, val)
        # size has units of bytes
        self.size = self.n_blocks * yaml_params['BLOCK_SIZE']

        # Check rules for peripheral alignment
        size_roundup = 2 ** math.ceil(math.log2(self.size))
        if self.addr_base % size_roundup != 0:
            msg = (f"Illegal addr_base ({self.addr_base:#x}). It must be a "
                   f"multiple of the peripheral's size in bytes, rounded up "
                   f"to the nearest power-of-2 bytes "
                   f"({size_roundup}):\n{self}\n")
            if not kwargs.get('ignore_alignment_error', False):
                raise cobex.MemoryMapPeriphAlignmentError(msg)
        ob_fields = [
            f for f, f_obj in self.fields.items() if
            f_obj.offset + f_obj.n_words * self.bytes_per_word > self.size
        ]
        if ob_fields:
            msg = (f"Fields exceed the {self.name} peripheral's addressable "
                   f"space: {ob_fields}.")
            raise cobex.MemoryMapFieldOverflowError(msg)

    def __str__(self):
        return '\n'.join([
            'name:       {}'.format(self.name),
            'addr_base:  {:#011_x}'.format(self.addr_base),
            'size (B):   {:#011_x}'.format(self.size),
            'n_blocks:   {}'.format(self.n_blocks),
            'Num fields: {}'.format(len(self.fields)),
        ])

    def get_field_objs(self, filt=lambda x: x) -> List['MemoryMapField']:
        """Returns the field objects using the provided filter. By default,
        the filter returns all field objects.
        """
        return list(filter(filt, self.fields.values()))

    def get_field_addr(self, field_name: str) -> int:
        """Returns the absolute byte address of a field.
        """
        return self.addr_base + self.fields[field_name].offset

    def register_read_callback(
        self,
        func: Callable[[int], ReadCallbackRet],
    ) -> None:
        """Assign a callback function for reading. The read_callback requires a
        byte address and returns a structure that contains a word of data. The
        word of data is accessed with the readdata_callback.
        """
        self.read_callback = func

    def register_readdata_callback(
        self,
        func: Callable[[ReadCallbackRet], int],
    ) -> None:
        """Assign a callback function for accessing the read data from the
        structure returned by the read_callback. If the read_callback returns
        read data directly then the readdata_callback can simply be
        `lambda x: x`.
        """
        self.readdata_callback = func

    def register_write_callback(
        self,
        func: Callable[[int, int], WriteCallbackRet],
    ) -> None:
        """Assign a callback function for writing. The write_callback requires
        a byte address and a word of data and returns anything.
        """
        self.write_callback = func

    def write_fields(self, **kwargs) -> WriteFieldsRet:
        """Writes a dict of one or more {field: value} pairs using the
        peripheral's write_callback. If more than one field name is given,
        this function returns a dict of {field_name: status} pairs. If only
        one field name is given then the status is returned directly. The
        status returned is directly from the write_callback. Note: this
        function does read-modify-write for each field but the order of writing
        is not guaranteed due to the ordering of the keys in the dict. Thus,
        the user is responsible for ensuring proper ordering of writes using
        multiple calls. Values can be raw numbers or mnemonic keys from the
        YAML. For each field that spans multiple words, the words are read and
        written in order of ascending address.
        """
        dikt = {field_name: None for field_name in kwargs}
        for field_name, value in kwargs.items():
            f_obj = self.fields[field_name]
            addr = self.get_field_addr(field_name)
            data = 0

            # Little endian byte ordering per PLECO (not MIPI CCI)
            for word_idx, a in enumerate(range(
                addr, addr + f_obj.n_bytes, self.bytes_per_word
            )):
                rstruct = self.read_callback(a)
                word = self.readdata_callback(rstruct) & self.word_mask
                data |= word << (word_idx * self.word_size)

            val = f_obj.mnemonic.get(value, value)
            mask = f_obj.get_field_mask()
            vmask = f_obj.get_field_mask(val)
            data = (data & ~mask) | vmask

            for word_idx, a in enumerate(range(
                addr, addr + f_obj.n_bytes, self.bytes_per_word
            )):
                word = (data >> (word_idx * self.word_size)) & self.word_mask
                ret = self.write_callback(a, word)

            # Return only the last write_callback for now (do we even use it?)
            dikt[field_name] = ret
        if len(kwargs) == 1:
            return list(dikt.values())[0]
        else:
            return dikt

    def read_fields(self, *args, use_mnemonic: bool = True) -> ReadFieldsRet:
        """Reads a list of fields using the peripheral's read_callback. If
        more than one field name is given, this function returns a dict of
        {field_name: value} pairs. If only one field name is given then the
        value is returned directly. The value returned is a mnemonic
        key from the YAML unless use_mnemonic=False or a mnemonic doesn't
        exist, in which case a raw number is returned. For a read_callback
        that has encapsulation of read data and hence requires a
        readdata_callback that is not identity, the returned value is the
        tuple (value, rstruct) where value is as described above and
        rstruct is the structure returned by the read_callback. For each
        field that spans multiple words, the words are read in order of
        ascending address.
        """
        dikt = {field_name: None for field_name in args}
        for field_name in args:
            f_obj = self.fields[field_name]
            addr = self.get_field_addr(field_name)
            data = 0

            # Little endian byte ordering per PLECO (not MIPI CCI)
            for word_idx, a in enumerate(range(
                addr, addr + f_obj.n_bytes, self.bytes_per_word
            )):
                rstruct = self.read_callback(a)
                word = self.readdata_callback(rstruct) & self.word_mask
                data |= word << (word_idx * self.word_size)
                if word_idx == 0:
                    uses_encapsulation = (rstruct != data)

            val = f_obj.get_field_value(data)
            if not use_mnemonic:
                mnemonic = val
            else:
                mnemonic = f_obj.imnemonic.get(val, val)
            if uses_encapsulation:
                # Return only the last rstruct for now (do we even use it?)
                dikt[field_name] = (mnemonic, rstruct)
            else:
                dikt[field_name] = mnemonic
        if len(args) == 1:
            return dikt[args[0]]
        else:
            return dikt

    def read_all_periph_fields(
        self,
        with_print: bool = False,
        sort_alpha: bool = False,
        use_mnemonic: bool = True,
    ) -> ReadFieldsRet:
        """Returns a dict of {field_name: value} pairs for all fields in the
        peripheral using read_fields(). If with_print is True then this
        function will print the dict before it is returned. Fields are printed
        in order of address offset unless sort_alpha is True, in which case
        they are printed in alphabetical order.
        """
        dikt = self.read_fields(*self.fields.keys(), use_mnemonic=use_mnemonic)
        if with_print:
            if sort_alpha:
                sort_key = lambda field_name: field_name
            else:
                sort_key = lambda field_name: self.fields[field_name].offset
            for field_name in sorted(dikt, key=sort_key):
                v = dikt[field_name]
                if isinstance(v, tuple):
                    data = v[0]
                else:
                    data = v
                addr = self.get_field_addr(field_name)
                if isinstance(data, str):
                    # mnemonic, which is guaranteed to be of type str
                    print(f'addr {addr:<5}:  {field_name:<30s}  {data}')
                else:
                    print(f'addr {addr:<5}:  {field_name:<30s}  {data:#011_x}  {data}')
            print('')
        return dikt


class MemoryMapField:
    """A class that defines one of the fields in a MemoryMapPeriph. A
    MemoryMapField is a continuous slice of memory that may or may not
    span memory word boundaries.
    """

    def __init__(self, yaml_params, **kwargs):
        # Attributes that come from the yaml
        self.name = None
        self.access = None
        self.offset = None
        self.size = None
        self.pos = None
        self.signed = None
        self.n_frac = None
        self.mnemonic = None
        self.pulsed = None
        self.logging = None
        self.default = None
        self.comment = None

        d = {'access': 'rw',
             'pulsed': False,
             'signed': False,
             'n_frac': 0,
             'logging': None,
             'default': 0,
             'mnemonic': {},
             'comment': ''}
        d.update(kwargs)
        for attr, val in d.items():
            if attr not in [
                'name', 'offset', 'pos', 'size',
                'access', 'pulsed', 'signed', 'n_frac',
                'logging', 'default', 'mnemonic', 'comment',
            ]:
                msg = "Invalid attr '{}' in field: {}".format(attr, d)
                raise cobex.MemoryMapFieldAttributeError(msg)
            if isinstance(val, str):
                val = val.format(**yaml_params)
            setattr(self, attr, val)
        self.n_words = math.ceil(self.size / yaml_params['WORD_SIZE'])

        # Check rules for field alignment
        width = yaml_params['WORD_SIZE']
        bytes_per_word = width // 8
        overflows_word = self.pos + self.size > width
        if self.size <= 0:
            msg = f"Illegal field size (must be > 0):\n{self}\n"
            raise cobex.MemoryMapFieldValueError(msg)
        if not 0 <= self.pos < width:
            msg = f"Illegal field pos (must be 0 <= pos < {width}):\n{self}\n"
            raise cobex.MemoryMapFieldValueError(msg)
        if self.offset < 0:
            msg = f"Illegal field offset (must be >= 0):\n{self}\n"
            raise cobex.MemoryMapFieldValueError(msg)
        if self.offset % bytes_per_word != 0:
            msg = (f"Illegal field offset (must be a multiple of "
                   f"{bytes_per_word:#x}):\n{self}\n")
            raise cobex.MemoryMapFieldValueError(msg)
        if overflows_word:
            if self.pos != 0:
                msg = (f'Multi-word fields (size > {width} bits) must '
                       f'have pos = 0:\n{self}\n')
                raise cobex.MemoryMapFieldAlignmentError(msg)
            n_bytes_roundup = 2 ** math.ceil(math.log2(self.n_bytes))
            if self.offset % n_bytes_roundup != 0:
                msg = (f'Multi-word fields (size > {width} bits) must '
                       f'have an offset that is an integral multiple '
                       f'of its size in bytes, rounded up to the nearest '
                       f'power of 2 ({n_bytes_roundup}):\n{self}\n')
                raise cobex.MemoryMapFieldAlignmentError(msg)
        bad_keys = [k for k in self.mnemonic.keys() if type(k) != str]
        if bad_keys:
            msg = "These mnemonic keys for field '{}' must be of type str: {}."
            msg = msg.format(self.name, str(bad_keys))
            raise cobex.MemoryMapFieldValueError(msg)
        self.imnemonic = {v: k for k, v in self.mnemonic.items()}

    def __str__(self):
        mnemonic_str = ", ".join([
            f'"{k}": {v:#0x}'
            for k, v in sorted(self.mnemonic.items(), key=lambda x: x[1])
        ])
        return '\n'.join([
            'name:       {}'.format(self.name),
            'offset:     0x{:04x}'.format(self.offset),
            'pos:        {}'.format(self.pos),
            'size:       {}'.format(self.size),
            'n_words:    {}'.format(self.n_words),
            'access:     {}'.format(self.access),
            'pulsed:     {}'.format(self.pulsed),
            'mask:       {:#011_x}'.format(self.get_field_mask()),
            'maxvalue:   {}'.format(self.get_field_value(-1)),
            'signed:     {}'.format(self.signed),
            'n_frac:     {}'.format(self.n_frac),
            'logging:    {}'.format(self.logging),
            'default:    {}'.format(self.default),
            'mnemonic:   {{{}}}'.format(mnemonic_str),
            'comment:    "{}"'.format(self.comment.replace('\n', ' ')),
        ])

    def get_field_mask(self, value=None):
        """Returns a word-wise mask for a field. For example, a field at
        position 9 and with length 2 would return 0x600. The optional argument
        'value' can be used to place a value in the mask instead of the usual
        block of 1's. In this way, this function can be used to decompose (AND)
        a word but can also be used to compose (OR) a word.
        """
        mask = (1 << self.size) - 1
        if value is None:
            return mask << self.pos
        else:
            if value & ~mask != 0:
                msg = (f'Value overflow (value={value:#_x}) for '
                       f'MemoryMapField:\n{str(self)}')
                raise ValueError(msg)
            return (value & mask) << self.pos

    def get_field_value(self, data):
        """Returns the value of the field given a word of data.
        """
        mask = (1 << self.size) - 1
        return (data >> self.pos) & mask

    @property
    def n_bytes(self) -> int:
        return math.ceil(self.size / 8)


M30_FPGA_YAML_PATH = Path(Path(__file__).parent,
                          'resources', 'm30_fpga_map.yml').absolute()
M30_SPI_FLASH_YAML_PATH = Path(Path(__file__).parent,
                               'resources', 'm30_spi_flash_map.yml').absolute()

PLECO_YAML_PATH = Path(Path(__file__).parent,
                       'resources', 'pleco_map.yml').absolute()

M30_FPGA_MEMORY_MAP = MemoryMap.from_cache(M30_FPGA_YAML_PATH)
M30_SPI_FLASH_MEMORY_MAP = MemoryMap.from_cache(
    M30_SPI_FLASH_YAML_PATH,
    ignore_alignment_error=True)

PLECO_MEMORY_MAP = MemoryMap.from_cache(PLECO_YAML_PATH)
