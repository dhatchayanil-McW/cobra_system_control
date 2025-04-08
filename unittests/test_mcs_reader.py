from io import StringIO
from pathlib import Path
from intelhex import hex2bin
from tempfile import TemporaryFile
import random

import pytest as pt

from cobra_system_control.mcs_reader import BinMcsReader, McsReader, HexRecord, HexRecordTypeEnum


def get_rand_bytes(n_bytes):
    return [random.randint(0, 0xff) for _ in range(n_bytes)]


def get_rand_uints(n_ints, n_bits):
    return [random.randint(0, 2**n_bits - 1) for _ in range(n_ints)]


def get_rand_ints(n_ints, n_bits):
    return [random.randint(-2**(n_bits - 1), 2**(n_bits - 1) - 1)
            for _ in range(n_ints)]


def test_hexrecord():
    # * check identity relation for from_str and to_str
    # * check construction and attributes via str() representation
    ela = ':02000004382999'
    rec = HexRecord.from_str(ela)
    ela_check = HexRecord.to_str(rec)

    assert ela == ela_check
    assert str(rec) == (
        "HexRecord(count=2, "
        "address=0, "
        "type=<HexRecordTypeEnum.EXT_LIN_ADDR: 4>, "
        "data=b'8)', "
        "checksum=153)"
    )

    # * check auto-population of checksum
    rec = HexRecord(0, 0, HexRecordTypeEnum.EOF, bytes(), None)
    assert 0xff == rec.checksum
    assert ':00000001FF' == rec.to_str()

    # * check invalid checksum (should be 0x99 no 0x9A)
    with pt.raises(ValueError):
        s = ':0200000438299A'
        rec = HexRecord.from_str(s)

    # * check invalid address (too low)
    with pt.raises(ValueError):
        HexRecord(0, -1, HexRecordTypeEnum.DATA, bytes())

    # * check invalid address (too high)
    with pt.raises(ValueError):
        HexRecord(0, 0xffff + 1, HexRecordTypeEnum.DATA, bytes())

    # * check invalid byte count (too low) with correct checksum
    with pt.raises(ValueError):
        s = ':00000000' + '01' + 'FF'
        rec = HexRecord.from_str(s)

    # * check invalid byte count (too high) with correct checksum
    with pt.raises(ValueError):
        s = ':01000000' + '' + 'FF'
        rec = HexRecord.from_str(s)

    # * check invalid byte count (too high, overflows record)
    with pt.raises(ValueError):
        s = ':02000000' + '' + 'FE'
        rec = HexRecord.from_str(s)

    # * check bad record type (0x10 not allowed)
    with pt.raises(ValueError):
        s = ':00000010F0'
        rec = HexRecord.from_str(s)

    # * check string too short
    with pt.raises(ValueError):
        s = ':' * 10
        rec = HexRecord.from_str(s)

    # * check string too short (empty)
    with pt.raises(ValueError):
        s = ''
        rec = HexRecord.from_str(s)

    # * check data_mirrored (one item)
    rec = HexRecord(1, 0, HexRecordTypeEnum.DATA, bytes([0b1010_0011]))
    assert rec.data_mirrored == bytes([0b1100_0101])

    # * check data_mirrored (>1 item)
    rec = HexRecord(3, 0, HexRecordTypeEnum.DATA, bytes([
        0b1010_0011,
        0b1111_0000,
        0b0000_1111,
    ]))
    assert rec.data_mirrored == bytes([
        0b1100_0101,
        0b0000_1111,
        0b1111_0000,
    ])

    # * check data-mirroring at init
    rec = HexRecord(1, 0, HexRecordTypeEnum.DATA, bytes([0x80]), None, True)
    assert bytes([0x01]) == rec.data
    assert bytes([0x80]) == rec.data_mirrored
    assert 0xFE == rec.checksum
    assert ':0100000001FE' == rec.to_str()

    # * check ELA record
    rec = HexRecord.get_ela_record(0xdead_beef)
    assert str(rec) == (
        "HexRecord(count=2, "
        "address=0, "
        "type=<HexRecordTypeEnum.EXT_LIN_ADDR: 4>, "
        f"data={bytes([0xde, 0xad])}, "
        "checksum=111)"
    )

    # * check EOF record
    rec = HexRecord.get_eof_record()
    assert str(rec) == (
        "HexRecord(count=0, "
        "address=0, "
        "type=<HexRecordTypeEnum.EOF: 1>, "
        "data=b'', "
        "checksum=255)"
    )


@pt.mark.parametrize('filename, has_init_ela', [
    pt.param('mcs.mcs', False),
    pt.param('mcs_init_ela.mcs', True),
])
def test_mcs(filename, has_init_ela):
    here = Path(__file__).parent.absolute()
    fid = Path(here, 'resources', filename)
    mcs_reader = McsReader(str(fid))

    # Check file names
    assert mcs_reader.get_files() == (str(fid),)

    # Check ELAs with one and more than one data record; also, ELAs with
    # same address but non-overlapping ranges
    incr = 1 if has_init_ela else 0
    assert [
            (f"Bookmark("
             f"filename={str(fid)}, "
             f"line_num={1 + incr}, "
             f"addr_start=0x0, "
             f"addr_stop=0x20)"),
            (f"Bookmark("
             f"filename={str(fid)}, "
             f"line_num={4 + incr}, "
             f"addr_start=0x1_0001, "
             f"addr_stop=0x1_0011)"),
            (f"Bookmark("
             f"filename={str(fid)}, "
             f"line_num={6 + incr}, "
             f"addr_start=0x1_0011, "
             f"addr_stop=0x1_0021)"),
        ] == [
            str(bm) for bm in
            sorted(mcs_reader.bookmarks, key=lambda x: x.addr_start)
        ]

    mem = {
        0x0000_0000: 0x08,
        0x0000_0001: 0x88,
        0x0000_0002: 0x48,
        0x0000_0003: 0xc8,
        0x0000_0004: 0x28,
        0x0000_0005: 0xa8,
        0x0000_0006: 0x68,
        0x0000_0007: 0xe8,
        0x0000_0008: 0x18,
        0x0000_0009: 0x98,
        0x0000_000a: 0x58,
        0x0000_000b: 0xd8,
        0x0000_000c: 0x38,
        0x0000_000d: 0xb8,
        0x0000_000e: 0x78,
        0x0000_000f: 0xf8,

        0x0000_0010: 0x04,
        0x0000_0011: 0x84,
        0x0000_0012: 0x44,
        0x0000_0013: 0xc4,
        0x0000_0014: 0x24,
        0x0000_0015: 0xa4,
        0x0000_0016: 0x64,
        0x0000_0017: 0xe4,
        0x0000_0018: 0x14,
        0x0000_0019: 0x94,
        0x0000_001a: 0x54,
        0x0000_001b: 0xd4,
        0x0000_001c: 0x34,
        0x0000_001d: 0xb4,
        0x0000_001e: 0x74,
        0x0000_001f: 0xf4,

        0x0001_0001: 0x0c,
        0x0001_0002: 0x8c,
        0x0001_0003: 0x4c,
        0x0001_0004: 0xcc,
        0x0001_0005: 0x2c,
        0x0001_0006: 0xac,
        0x0001_0007: 0x6c,
        0x0001_0008: 0xec,
        0x0001_0009: 0x1c,
        0x0001_000a: 0x9c,
        0x0001_000b: 0x5c,
        0x0001_000c: 0xdc,
        0x0001_000d: 0x3c,
        0x0001_000e: 0xbc,
        0x0001_000f: 0x7c,
        0x0001_0010: 0xfc,

        0x0001_0011: 0x02,
        0x0001_0012: 0x82,
        0x0001_0013: 0x42,
        0x0001_0014: 0xc2,
        0x0001_0015: 0x22,
        0x0001_0016: 0xa2,
        0x0001_0017: 0x62,
        0x0001_0018: 0xe2,
        0x0001_0019: 0x12,
        0x0001_001a: 0x92,
        0x0001_001b: 0x52,
        0x0001_001c: 0xd2,
        0x0001_001d: 0x32,
        0x0001_001e: 0xb2,
        0x0001_001f: 0x72,
        0x0001_0020: 0xf2,
    }
    for k, v in mem.items():
        assert v == mcs_reader[k]

    # check out of in range but unspecified records
    for a in [random.randint(0, 0x3f_ffff) for _ in range(20)]:
        if a not in mem:
            assert 0xff == mcs_reader[a]
        else:
            assert mem[a] == mcs_reader[a]

    # check index out of bounds
    with pt.raises(IndexError):
        mcs_reader[mcs_reader.mem_size]

    with pt.raises(IndexError):
        mcs_reader[-1]


@pt.mark.parametrize('filename', [
    pt.param('mcs_repeat_ela.mcs',),
    pt.param('mcs_init_ela_repeat_ela.mcs',),
    pt.param('mcs_trailing_ela.mcs',),
    pt.param('mcs_eof_only.mcs',),
    pt.param('mcs_empty.mcs',),
])
def test_mcs_negative(filename):
    with pt.raises(RuntimeError):
        here = Path(__file__).parent.absolute()
        fid = Path(here, 'resources', filename)
        _ = McsReader(str(fid))


def test_mcs_merge_and_dump():
    here = Path(__file__).parent.absolute()
    fid = Path(here, 'resources', 'mcs.mcs')
    mcs_reader = McsReader(str(fid))
    merge_fid = Path(here, 'resources', 'mcs_merge.mcs')
    other = McsReader(str(merge_fid))
    mcs_reader.merge(other)
    assert [
            (f"Bookmark("
             f"filename={str(fid)}, "
             f"line_num=1, "
             f"addr_start=0x0, "
             f"addr_stop=0x20)"),
            (f"Bookmark("
             f"filename={str(fid)}, "
             f"line_num=4, "
             f"addr_start=0x1_0001, "
             f"addr_stop=0x1_0011)"),
            (f"Bookmark("
             f"filename={str(fid)}, "
             f"line_num=6, "
             f"addr_start=0x1_0011, "
             f"addr_stop=0x1_0021)"),
            (f"Bookmark("
             f"filename={str(merge_fid)}, "
             f"line_num=2, "
             f"addr_start=0x2_0000, "
             f"addr_stop=0x2_0010)"),
        ] == [
            str(bm) for bm in
            sorted(mcs_reader.bookmarks, key=lambda x: x.addr_start)
        ]

    # Check file names
    assert set(mcs_reader.get_files()) == set((str(fid), str(merge_fid)))

    # Check dump with defaults
    sio = StringIO()
    mcs_reader.dump(sio)
    assert sio.getvalue() == "\n".join([
        ":020000040000FA",
        ":10000000101112131415161718191A1B1C1D1E1F78",
        ":10001000202122232425262728292A2B2C2D2E2F68",
        ":020000040001F9",
        ":10000100303132333435363738393A3B3C3D3E3F77",
        ":020000040001F9",
        ":10001100404142434445464748494A4B4C4D4E4F67",
        ":020000040002F8",
        ":10000000101112131415161718191A1B1C1D1E1F78",
        ":00000001FF",
    ])
    sio.close()

    # Check dump with one data record
    sio = StringIO()
    mcs_reader.dump(sio, addr_start=16, addr_stop=18)
    assert sio.getvalue() == "\n".join([
        ":020000040000FA",
        ":10001000202122232425262728292A2B2C2D2E2F68",
        ":00000001FF",
    ])
    sio.close()

    # Check multiple dumps
    sio = StringIO()
    mcs_reader.dump(sio, addr_start=0, addr_stop=17, include_eof=False)
    mcs_reader.dump(sio, addr_start=0x1_0011, addr_stop=0x1_ffff)
    assert sio.getvalue() == "\n".join([
        ":020000040000FA",
        ":10000000101112131415161718191A1B1C1D1E1F78",
        ":10001000202122232425262728292A2B2C2D2E2F68",
        ":020000040001F9",
        ":10001100404142434445464748494A4B4C4D4E4F67",
        ":00000001FF",
    ])
    sio.close()


def test_mcs_bad_merge():
    here = Path(__file__).parent.absolute()
    f1 = 'mcs.mcs'
    f2 = 'mcs_init_ela.mcs'
    fid1 = Path(here, 'resources', f1)
    fid2 = Path(here, 'resources', f2)
    mcs_reader = McsReader(str(fid1))
    other = McsReader(str(fid2))
    with pt.raises(RuntimeError):
        mcs_reader.merge(other)

    with pt.raises(RuntimeError):
        other.merge(mcs_reader)


@pt.mark.parametrize('filename', [
    pt.param('mcs.mcs',),
])
def test_binmcs(filename):
    here = Path(__file__).parent.absolute()
    fid = Path(here, 'resources', filename)
    out = TemporaryFile()

    # Test hex2bin
    assert(hex2bin(str(fid), out, 0, None, None, 0xFF) == 0)
    out.seek(0)
    mcs_reader = BinMcsReader(out)
    mcs_reader.filename = 'mcs.bin' # give our fake temp file a name

    # Check file names
    assert mcs_reader.get_files() == ('mcs.bin',)

    mem = {
        0x0000_0000: 0x08,
        0x0000_0001: 0x88,
        0x0000_0002: 0x48,
        0x0000_0003: 0xc8,
        0x0000_0004: 0x28,
        0x0000_0005: 0xa8,
        0x0000_0006: 0x68,
        0x0000_0007: 0xe8,
        0x0000_0008: 0x18,
        0x0000_0009: 0x98,
        0x0000_000a: 0x58,
        0x0000_000b: 0xd8,
        0x0000_000c: 0x38,
        0x0000_000d: 0xb8,
        0x0000_000e: 0x78,
        0x0000_000f: 0xf8,

        0x0000_0010: 0x04,
        0x0000_0011: 0x84,
        0x0000_0012: 0x44,
        0x0000_0013: 0xc4,
        0x0000_0014: 0x24,
        0x0000_0015: 0xa4,
        0x0000_0016: 0x64,
        0x0000_0017: 0xe4,
        0x0000_0018: 0x14,
        0x0000_0019: 0x94,
        0x0000_001a: 0x54,
        0x0000_001b: 0xd4,
        0x0000_001c: 0x34,
        0x0000_001d: 0xb4,
        0x0000_001e: 0x74,
        0x0000_001f: 0xf4,

        0x0001_0001: 0x0c,
        0x0001_0002: 0x8c,
        0x0001_0003: 0x4c,
        0x0001_0004: 0xcc,
        0x0001_0005: 0x2c,
        0x0001_0006: 0xac,
        0x0001_0007: 0x6c,
        0x0001_0008: 0xec,
        0x0001_0009: 0x1c,
        0x0001_000a: 0x9c,
        0x0001_000b: 0x5c,
        0x0001_000c: 0xdc,
        0x0001_000d: 0x3c,
        0x0001_000e: 0xbc,
        0x0001_000f: 0x7c,
        0x0001_0010: 0xfc,

        0x0001_0011: 0x02,
        0x0001_0012: 0x82,
        0x0001_0013: 0x42,
        0x0001_0014: 0xc2,
        0x0001_0015: 0x22,
        0x0001_0016: 0xa2,
        0x0001_0017: 0x62,
        0x0001_0018: 0xe2,
        0x0001_0019: 0x12,
        0x0001_001a: 0x92,
        0x0001_001b: 0x52,
        0x0001_001c: 0xd2,
        0x0001_001d: 0x32,
        0x0001_001e: 0xb2,
        0x0001_001f: 0x72,
        0x0001_0020: 0xf2,
    }

    # check for exactness in defined data
    for k, v in mem.items():
        assert v == mcs_reader[k]

    # check for exactness in undefined data
    assert(all([x == 0xFF for x in mcs_reader[0x00_0020:0x01_0001]]))

    # check out of in range but unspecified records
    for a in [random.randint(0, 0x3f_ffff) for _ in range(20)]:
        if a not in mem:
            assert 0xff == mcs_reader[a]
        else:
            assert mem[a] == mcs_reader[a]

    # check index out of bounds
    with pt.raises(IndexError):
        mcs_reader[mcs_reader.mem_size]

    with pt.raises(IndexError):
        mcs_reader[:mcs_reader.mem_size+1]

    with pt.raises(IndexError):
        mcs_reader[-1]

    with pt.raises(IndexError):
        mcs_reader[-1:]

    with pt.raises(IndexError):
        mcs_reader["a"]

    assert(mcs_reader.addr_min == 0)
    assert(mcs_reader.addr_max == mcs_reader.mem_size)
    assert(len(mcs_reader) == mcs_reader.mem_size)
