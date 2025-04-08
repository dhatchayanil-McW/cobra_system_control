"""
file: mcs_reader.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file provides a class to read MCS files based on
the Intel Hex format.

Wikipedia has a good description of the Intel Hex format
https://en.wikipedia.org/wiki/Intel_HEX
"""
from dataclasses import dataclass, InitVar
from enum import Enum
from pathlib import Path
import numpy as np
import linecache
import sys
from typing import Union, TextIO, List


DEBUG = False


class HexRecordTypeEnum(Enum):
    DATA = 0
    EOF = 1
    EXT_SEG_ADDR = 2
    START_SEG_ADDR = 3
    EXT_LIN_ADDR = 4
    START_LIN_ADDR = 5


@dataclass
class HexRecord:
    """A container for an Intel Hex record as described here:
        https://en.wikipedia.org/wiki/Intel_HEX.
    Intel Hex records are used in MCS files, which are used to describe the
    contents of SPI flash memory. A record has the following ASCII format:
        char index | 0 12 3456 78 9abcde f0
        format     | : CC AAAA TT DDDDDD SS
        example    | : 03 0030 00 02337A 1E
    where
        * : = a literal colon
        * CC = count
        * AAAA = address
        * TT = type
        * DD = a data byte
        * SS = checksum
    """
    count: int
    address: int
    type: HexRecordTypeEnum
    data: bytes
    checksum: Union[int, None] = None
    reverse_data: InitVar[bool] = False

    def __post_init__(self, reverse_data):
        if not 0 <= self.address <= 0xffff:
            msg = f"Address is outside the valid range [0, 0xffff]: {self}."
            raise ValueError(msg)
        if len(self.data) != self.count:
            msg = f"Invalid amount of data ({len(self.data)}): {self}."
            raise ValueError(msg)

        if reverse_data:
            self.data = self.data_mirrored

        sum_ = (self.count
                + (self.address >> 8)
                + self.address
                + self.type.value
                + sum(self.data))
        check = -sum_ & 0xff
        if self.checksum is None:
            self.checksum = check
        elif self.checksum != check:
            raise ValueError(f"Invalid checksum 0x{check:02x}: {self}.")

    @classmethod
    def from_str(cls, s: str) -> 'HexRecord':
        if len(s) < 11:
            msg = f"Invalid string length ({len(s)}): '{s}'."
            raise ValueError(msg)

        count = int(s[1 : 3], 16)
        address = int(s[3 : 7], 16)
        type_ = int(s[7 : 9], 16)
        data = bytes([int(s[9 + 2 * i : 9 + 2 * (i + 1)], 16)
                      for i in range(len(s[9 : -2]) // 2)])
        checksum = int(s[-2:], 16)
        return cls(count, address, HexRecordTypeEnum(type_), data, checksum)

    def to_str(self) -> str:
        # use 'X' for uppercase
        return ''.join([
            ':',
            '{:02X}'.format(self.count),
            '{:04X}'.format(self.address),
            '{:02X}'.format(self.type.value),
            *['{:02X}'.format(d) for d in self.data],
            '{:02X}'.format(self.checksum),
        ])

    @property
    def data_mirrored(self) -> bytes:
        """MCS file data records have little endian byte order but each data
        byte is bit-reversed. This function coverts the MCS data to "usable"
        data.
        """
        return bytes([int('{:08b}'.format(b)[::-1], 2) for b in self.data])

    @classmethod
    def get_ela_record(cls, abs_address: int) -> 'HexRecord':
        """Returns a HexRecord of type extended linear address (ELA) using the
        provided absolute address. See https://en.wikipedia.org/wiki/Intel_HEX.

        ELA records allow for 32-bit addressing (up to 4 GiB). The record's
        address field is ignored (typically zero), its byte count is always
        two, and the two data bytes (big endian) specify the upper 16 bits of
        the 32 bit absolute address for all subsequent Data records (type = 0).
        These upper address bits remain in effect until the next ELA record. If
        a Data record is not preceded by an ELA record then the upper bits of
        the absolute address default to zero.
        """
        return cls(2, 0, HexRecordTypeEnum.EXT_LIN_ADDR,
                   (abs_address >> 16).to_bytes(2, 'big')
        )

    @classmethod
    def get_eof_record(cls) -> 'HexRecord':
        return cls(0, 0, HexRecordTypeEnum.EOF, bytes(), None)


@dataclass
class Bookmark:
    """A mapping between memory address and line number in an MCS file
    """
    filename: str
    line_num: int
    addr_start: int
    addr_stop: int

    def __post_init__(self):
        if self.addr_start >= self.addr_stop:
            msg = (f"addr_start={self.addr_start:#_x} must be less than "
                   f"addr_stop={self.addr_stop:#_x}.")
            raise RuntimeError(msg)

    def __repr__(self):
        return (f'Bookmark(filename={self.filename}, '
                f'line_num={self.line_num}, '
                f'addr_start={self.addr_start:#_x}, '
                f'addr_stop={self.addr_stop:#_x})')


class McsReader:
    """A (hopefully) more efficient MCS reader than intelhex."""

    def __init__(
            self,
            mcs_filename: str,
            mem_size: int = 0x40_0000,
            addr_step: int = 16,
            padding: int = 0xff,
    ):
        """Creates an McsReader instance and parses the provided MCS file
        for formatting rules.

        Args:
            mcs_filename (str): a path to the MCS file. (linecache does not support Path objects)
            mem_size (int = 0x40_0000): the size of the memory described by
                MCS file.
            addr_step (int = 16): the count field for all Hex Records of type
                Data in the MCS file (except trailing ones).
            padding (int = 0xff): the value returned for memory that is not
                described by the MCS file.
        """
        self.mem_size = mem_size
        self.bookmarks = []
        self.addr_step = addr_step
        self.padding = padding

        # Parse the file for address "bookmarks" and contiguous addressing
        with open(mcs_filename, 'r', encoding='utf8') as f:
            addr_base = 0
            addr_next = None
            record_prev = None
            bm_line_num = None
            bm_start = None
            bm_stop = None
            for i, line in enumerate(f):
                line_num = i + 1
                record = HexRecord.from_str(line.strip())
                if DEBUG and record.type != HexRecordTypeEnum.DATA:
                    print(record)
                    print(HexRecord.to_str(record))
                    print(line.strip() == HexRecord.to_str(record))

                if record.type == HexRecordTypeEnum.EXT_LIN_ADDR:
                    if line_num == 1:
                        pass
                    elif record_prev.type == HexRecordTypeEnum.EXT_LIN_ADDR:
                        msg = (f"Detected back-to-back ELA records at line "
                               f"#{line_num}: {record}.")
                        raise RuntimeError(msg)
                    else:
                        # save a bookmark since we have found its end
                        bm_stop = (addr_base | record_prev.address) + record_prev.count
                        bm = Bookmark(mcs_filename, bm_line_num, bm_start, bm_stop)
                        self.bookmarks.append(bm)
                        bm_start = None
                    addr_base = int.from_bytes(record.data, byteorder='big') << 16
                elif record.type == HexRecordTypeEnum.DATA:
                    if (
                            line_num == 1 or
                            record_prev.type == HexRecordTypeEnum.EXT_LIN_ADDR
                    ):
                        # the beginning of a new bookmark
                        bm_line_num = line_num
                        bm_start = addr_base | record.address
                        addr_next = record.address + record.count
                    else:
                        # check record_prev, not record, to skip the last
                        # record before an ELA or EOF and hence allow it to
                        # have less than self.addr_step number of data bytes.
                        if record_prev.count != self.addr_step:
                            msg = (f"Detected data record with inconsistent "
                                   f"data length at line #{line_num - 1}: "
                                   f"{record_prev}.")
                            raise RuntimeError(msg)
                        if record.address != addr_next:
                            msg = (f"Non-contiguous memory block at line "
                                   f"#{line_num}: {record}.")
                            raise RuntimeError(msg)
                        addr_next += record.count
                elif record.type == HexRecordTypeEnum.EOF:
                    if line_num == 1:
                        msg = "EOF record detected on the first line."
                        raise RuntimeError(msg)
                    elif record_prev.type == HexRecordTypeEnum.EXT_LIN_ADDR:
                        msg = (f"EOF record detected immediately after an ELA "
                               f"record at line #{line_num}.")
                        raise RuntimeError(msg)
                    else:
                        # save a bookmark since we have found its end
                        bm_stop = (addr_base | record_prev.address) + record_prev.count
                        bm = Bookmark(mcs_filename, bm_line_num, bm_start, bm_stop)
                        self.bookmarks.append(bm)
                        bm_start = None
                    break
                else:
                    msg = (f"Unsupported record type at line #{line_num}: "
                           f"{record}.")
                    raise RuntimeError(msg)
                record_prev = record
            else:
                # raise RuntimeError('File ended without EOF.')

                # We didn't break from the loop so no EOF record was found.
                # Treat file as if there is an implied EOF.
                if bm_start is None:
                    # This case only happens if the file was empty or the last
                    # record was an ELA (DATA sets bm_start and we know there
                    # wasn't an EOF/break).
                    msg = (f"File unexpectedly ended without EOF. The last "
                           f"record was: {record_prev}.")
                    raise RuntimeError(msg)
                else:
                    # save a bookmark since we have found its end
                    bm_stop = (addr_base | record_prev.address) + record_prev.count
                    bm = Bookmark(mcs_filename, bm_line_num, bm_start, bm_stop)
                    self.bookmarks.append(bm)
                    bm_start = None

        if DEBUG:
            for bm in self.bookmarks:
                print(bm)
            for bm in self.bookmarks:
                print(linecache.getline(mcs_filename, bm.line_num))

        # Check the bookmarks for errors
        for i, bm in enumerate(self.bookmarks_sorted):
            if (i != 0 and
                bm_prev.addr_start <= bm.addr_stop - 1 and
                bm_prev.addr_stop - 1 >= bm.addr_start
            ):
                msg = (f"Overlapping ELA segments detected: {bm_prev} "
                       f"and {bm}.")
                raise RuntimeError(msg)
            bm_prev = bm

    @property
    def addr_min(self):
        return list(self.bookmarks_sorted)[0].addr_start

    @property
    def addr_max(self):
        return list(self.bookmarks_sorted)[-1].addr_stop

    def __getitem__(self, key):
        return self.get_item(key, mirror_data=True)

    def get_item(self, key, mirror_data=False) -> Union[int, bytes]:
        """Get the memory contents for the corresponding address(es) `key`.
        Use mirror_data=True to get the usable data and mirror_data=False to
        get the data as it is formatted in the MCS file.
        """
        if isinstance(key, slice):
            iterator = range(
                0 if key.start is None else key.start,
                self.mem_size if key.stop is None else key.stop,
                1 if key.step is None else key.step,
            )
        elif isinstance(key, int):
            iterator = (key,)
        else:
            raise IndexError(f"Invalid key {key}.")

        ret = []
        for addr in iterator:
            if not 0 <= addr < self.mem_size:
                raise IndexError(f"Invalid address {addr:#_x} for key {key}.")
            bm = self._get_bookmark(addr)
            if bm is None:
                ret.append(self.padding)
            else:
                delta = addr - bm.addr_start
                line_num = bm.line_num + delta // self.addr_step
                byte_index = delta % self.addr_step
                line = linecache.getline(bm.filename, line_num)
                if mirror_data:
                    data = HexRecord.from_str(line.strip()).data_mirrored
                else:
                    data = HexRecord.from_str(line.strip()).data
                ret.append(data[byte_index])
        if isinstance(key, int):
            return ret[0]
        else:
            return bytes(ret)

    def _get_bookmark(self, address: int):
        """Return the bookmark in which the requested address resides.
        This function will return None for addresses that overflow or
        underflow the set of bookmarks.
        """
        for bm in self.bookmarks:
            if bm.addr_start <= address < bm.addr_stop:
                return bm
        if DEBUG:
            print(f"No Bookmark found for address 0x{address:x}.")
        return None

    @property
    def bookmarks_sorted(self):
        """An iterator/generator over the bookmarks, sorted by addr_start."""
        for bm in sorted(self.bookmarks, key=lambda x: x.addr_start):
            yield bm

    def get_files(self) -> tuple:
        """Returns a tuple of file names for all MCS files backing this
        object.
        """
        return tuple({bm.filename for bm in self.bookmarks})

    def dump(
            self,
            filelike=sys.stdout,
            addr_start: int = 0,
            addr_stop: Union[int, None] = None,
            include_eof: bool = True,
    ):
        """Writes the given file-like object (sys.stdout is default) according
        to all defined bookmarks unless addresses are specified. In this case,
        only the bookmarked memory on [addr_start, addr_stop) are dumped.
        Before dumping, addr_stop is conditionally increased in order to emit
        a multiple of self.addr_step items (i.e. an integral number of data
        records).
        """
        addr_stop = self.mem_size if addr_stop is None else addr_stop
        for bm in self.bookmarks_sorted:
            # check whether this bookmark overlaps the desired dump range
            start = max(addr_start, bm.addr_start)
            stop = min(addr_stop, bm.addr_stop)
            if start >= stop:
                continue

            # align records: keep start and pad stop such that the number of
            # items is a multiple of addr_step
            delta = (stop - start)
            stop = stop + (-delta % self.addr_step)
            # align records: subtract extra from start and add pad to stop
            #start = start - (start % self.addr_step)
            #stop = stop + (-stop % self.addr_step)

            rec = HexRecord.get_ela_record(start)
            print(rec.to_str(), file=filelike)
            for addr in range(start, stop, self.addr_step):
                rec = HexRecord(
                    self.addr_step,
                    addr & 0xffff,
                    HexRecordTypeEnum.DATA,
                    bytes(self.get_item(slice(addr, addr + self.addr_step))),
                    None,
                )
                print(rec.to_str(), file=filelike)
        if include_eof:
            rec = HexRecord.get_eof_record()
            print(rec.to_str(), file=filelike, end='')

    def merge(self, other: 'McsReader'):
        """Merge another McsReader into this one. Attributes 'mem_size',
        'addr_step', and 'padding' must match between the two objects.
        Merging is accomplished by merging the bookmarks and then checking
        for overlapping address ranges.
        """
        if other.mem_size != self.mem_size:
            msg = (f"Other McsReader object has mem_size "
                   f"{other.mem_size:#_x}, which doesn't match self "
                   f"({self.mem_size:#_x}). An McsReader object can only "
                   f"describe a single memory device.")
            raise RuntimeError(msg)
        if other.addr_step != self.addr_step:
            msg = (f"Other McsReader object has addr_step "
                   f"{other.addr_step:#_x}, which doesn't match self "
                   f"({self.addr_step:#_x}). Can't merge.")
            raise RuntimeError(msg)
        if other.padding != self.padding:
            msg = (f"Other McsReader object has padding "
                   f"{other.padding:#_x}, which doesn't match self "
                   f"({self.padding:#_x}). Can't merge.")
            raise RuntimeError(msg)
        bookmarks = [*self.bookmarks, *other.bookmarks]
        bookmarks.sort(key=lambda x: x.addr_start)

        # Check the bookmarks for errors
        for i, bm in enumerate(bookmarks):
            if (i != 0 and
                bm_prev.addr_start <= bm.addr_stop - 1 and
                bm_prev.addr_stop - 1 >= bm.addr_start
            ):
                msg = (f"Overlapping ELA segments detected when merging. "
                       f"Bookmarks are {bm_prev} and {bm}.")
                raise RuntimeError(msg)
            bm_prev = bm
        self.bookmarks = bookmarks


class BinMcsReader:
    """
    A more efficient binary MCS reader than intelhex. Note that these binary
    representations of MCS files do not contain any extra metadata like address
    or checksum information, only data.

    Binary MCS files can be created using hex2bin.py from the intelhex package:
        hex2bin.py --pad FF --range 0: file.mcs file.bin
    """

    def __init__(
            self, mcs_filename: Path,
            mem_size: int = 0x40_0000,
            padding: int = 0xff
        ):
        """
        Creates a BinMcsReader instance and parses the provided binary MCS file.

        Args:
            mcs_filename (Path): a path to the binary MCS file.
            mem_size (int = 0x40_0000): the size of the memory described by
                MCS file.
            padding (int = 0xff): the value returned for memory that is not
                described by the MCS file.
        """
        # Read in the binary MCS file as a uint8 numpy array
        self.filename = mcs_filename
        self.mem_size = mem_size
        self.padding  = padding
        self.data     = np.fromfile(mcs_filename, dtype=np.uint8)
        self.data     = np.pad(
            self.data,
            (0, (mem_size - len(self.data))),
            constant_values=padding
        )

    def __len__(self):
        return len(self.data)

    def __getitem__(self, key):
        return self.get_item(key, mirror_data=True)

    def get_item(self, key, mirror_data=False) -> Union[int, bytes]:
        """
        Get the memory contents for the corresponding address(es) `key`.
        Use mirror_data=True to get the usable data and mirror_data=False to
        get the data as it is formatted in the binary MCS file.
        """
        if not isinstance(key, int) and not isinstance(key, slice):
            raise IndexError(f"Invalid key {key}")

        mem_range = range(0, self.mem_size)
        if isinstance(key, int) and (key not in mem_range):
            raise IndexError(f"Invalid address {key:#_x}")
        elif isinstance(key, slice):
            if key.start is not None and (key.start not in mem_range):
                raise IndexError(f"Invalid address {key.start:#_x}")
            if key.stop is not None and (key.stop not in mem_range):
                raise IndexError(f"Invalid address {key.stop:#_x}")

        ret = self.reverse_bits(self.data[key]) if mirror_data else self.data[key]
        return ret if isinstance(key, int) else bytes(ret)

    def get_files(self) -> tuple:
        """
        Returns a tuple of file names for all MCS files backing this object.
        This is a compatibility method for McsReader.
        """
        return tuple({self.filename})

    @staticmethod
    def reverse_bits(x: Union[np.uint8, np.ndarray]) -> Union[np.uint8, np.ndarray]:
        """
        Super fast method for reversing bits. This function is compatible
        with numpy ufuncs and can take a numpy array as an argument.
        """
        x = ((x & 0x55) << 1) | ((x & 0xAA) >> 1)
        x = ((x & 0x33) << 2) | ((x & 0xCC) >> 2)
        x = ((x & 0x0F) << 4) | ((x & 0xF0) >> 4)
        return x

    @property
    def addr_min(self):
        return 0

    @property
    def addr_max(self):
        return len(self.data)


class LcmMcsWriter:
    def __init__(self, base_address: int,
                 lcm_pattern_table: List[bytearray],
                 mirror_user_data: bool,
                 write_eof_record: bool,
                 out_file: str,
                 # Lattice Radiant values
                 bytes_per_data_record: int = 16,
                 # We can fit 64 LCM tables in each segment.
                 tables_per_segment: int = 64,
                 ):
        self.base_address = base_address
        self.mirror_user_data = mirror_user_data
        self.write_eof_record = write_eof_record
        self.out_file = out_file
        self.bytes_per_data_record = bytes_per_data_record
        self.tables_per_segment = tables_per_segment

        self.tables = lcm_pattern_table
        self.bytes_per_table = len(lcm_pattern_table[0])

    def write_table(self, f: TextIO, address: int, table: bytearray):
        """
        Write 1 table (1 kByte = 64 data records)
        """
        size_table = len(table)
        if size_table != self.bytes_per_table:
            msg1 = f'Size of table = {size_table}. '
            msg2 = f'Table size should be {self.bytes_per_table} bytes. '
            raise RuntimeError(msg1 + msg2)

        num_records = self.bytes_per_table // self.bytes_per_data_record
        for i in range(num_records):
            # grab 16 bytes from the table
            data = bytearray()
            table_start = i * self.bytes_per_data_record
            table_end = (i+1) * self.bytes_per_data_record
            data = table[table_start:table_end]
            record_address = (address + i * self.bytes_per_data_record) % 2**16
            record = HexRecord(len(data), record_address, HexRecordTypeEnum.DATA,
                               data, None, self.mirror_user_data)
            f.write(record.to_str())
            f.write('\n')

    def write_mcs(self):
        with open(self.out_file, 'w', encoding='utf8') as f:
            # 64 tables per 16-bit address segment
            seg_count = 0
            for table_index, table in enumerate(self.tables):
                if (table_index % self.tables_per_segment) == 0:
                    # Generate a segment record
                    # HexRecord.get_ela_record takes absolute address
                    abs_seg_address = ((self.base_address >> 16) + seg_count)
                    ela = HexRecord.get_ela_record(abs_seg_address << 16)
                    f.write(ela.to_str())
                    f.write('\n')
                    seg_count += 1

                self.write_table(f,
                                 table_index * self.bytes_per_table,
                                 table)

            if self.write_eof_record:
                f.write(':00000001FF\n')
