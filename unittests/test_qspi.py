# pylint: disable=protected-access
import random
import time

import pytest

from cobra_system_control.memory_map import M30_SPI_FLASH_MEMORY_MAP
import cobra_system_control.w25q32jw_const as wb


KIB = 1024


def _recv_qspi_ret(mock, spi_flash, ret):
    """Helper method to set the ``_qspi_receive`` return value when mocking.
    This implicitly passes when not mocking to simplify test code.
    """
    if mock:
        spi_flash.qspi._qspi_receive.return_value = ret


def test_read_mfg_id(mock, spi_flash):
    if mock:
        spi_flash.qspi._qspi_receive.return_value = 0xef15

    mid = spi_flash.qspi.read_mfg_device_id()
    assert mid == 0xef15, f'expected 0xef15, got {mid:07x}'


def test_read_jedec_id(mock, spi_flash):
    if mock:
        spi_flash.qspi._qspi_receive.return_value = 0xef6016

    jedec = spi_flash.qspi.read_jedec_id()
    assert jedec == 0xef6016, f'expected 0xef6016, got {jedec:011x}'


def test_read_unique_id(mock, spi_flash):
    if mock:
        spi_flash.qspi._qspi_receive.return_value = 2**4

    iid = spi_flash.qspi.read_unique_id()
    assert iid < 2**(8 * 8)


def test_wel(mock, spi_flash):
    if mock:
        spi_flash.qspi._qspi_receive.return_value = 0x00
    assert spi_flash.qspi.get_write_enable() is False

    spi_flash.qspi.set_write_enable(True)
    if mock:
        spi_flash.qspi._qspi_receive.return_value = 0x02
    assert spi_flash.qspi.get_write_enable() is True

    spi_flash.qspi.set_write_enable(False)
    if mock:
        spi_flash.qspi._qspi_receive.return_value = 0x00
    assert spi_flash.qspi.get_write_enable() is False


def test_status_read(mock, spi_flash):
    cmd_val = [
        (wb.QSPI_CMD_READ_SR_3, 0x60),
        (wb.QSPI_CMD_READ_SR_2, 0x02),
        (wb.QSPI_CMD_READ_SR_1, 0x00),
    ]
    rdata = []
    for cmd, val in cmd_val:
        spi_flash.qspi._qspi_send([cmd])
        if mock:
            spi_flash.qspi._qspi_receive.return_value = val
        rd = spi_flash.qspi._qspi_receive()
        rdata.extend([rd])
        assert val == rd
    assert len(rdata) == 3, f'rdata status {rdata}'


@pytest.mark.usefixtures('mock_only')
def test_page_program(mock, spi_flash):
    if not mock:
        pytest.skip("Don't overwrite the spi flash.")

    address = 0x200000
    # Need to erase before writing!
    if mock:
        spi_flash.qspi._qspi_receive.return_value = 0

    spi_flash.qspi.sector_erase(address)

    wdata = [random.randint(0, 0xff) for _ in range(wb.W25Q32JW_PAGE_SIZE)]
    spi_flash.qspi.page_program(address, wdata)

    # Fast read will read 1024 bytes but we only wrote 256
    if mock:
        empty = [0xff for _ in range(1024 - 256)]
        spi_flash.qspi._qspi_receive.return_value = wdata + empty

    rdata = spi_flash.qspi.fast_read_data(address)
    trimmed_rdata = rdata[0:256]
    for idx, byte in enumerate(rdata[256::]):
        assert byte == 0xff, f'byte{idx}, {byte} != 0xff'
    for idx, (wby, rby) in enumerate(zip(wdata, trimmed_rdata)):
        assert wby == rby, f'byte{idx}, \\x{wby:02x} != \\x{rby:02x}'


@pytest.mark.usefixtures('mock_only')
def test_sector_erase(mock, spi_flash):
    if not mock:
        pytest.skip("Don't erase the spi flash.")

    def _qspi_ret(r):
        return _recv_qspi_ret(mock, spi_flash, r)

    address = 0x200000

    # _recv_qspi_ret(mock, spi_flash, 0)
    _qspi_ret(0)

    # Need to erase before writing!
    spi_flash.qspi.sector_erase(address)

    # Page program is 256 but need to write 4k
    wdata_full = []
    for pg in range(4 * KIB // wb.W25Q32JW_PAGE_SIZE):
        wdata = [random.randint(0, 0xff) for _ in range(wb.W25Q32JW_PAGE_SIZE)]
        wdata_full.extend(wdata)
        offset = pg * wb.W25Q32JW_PAGE_SIZE
        spi_flash.qspi.page_program(address | offset, wdata)

    # Read back to make sure there is data in the sector
    # fast read gets 1024 bytes
    rdata_full = []
    # Read out 4k
    for pg in range(4):
        offset = pg * wb.W25Q32JW_FAST_READ_SIZE
        wdata_slice = slice(offset, (pg + 1) * wb.W25Q32JW_FAST_READ_SIZE)
        _qspi_ret(wdata_full[wdata_slice])

        rdata = spi_flash.qspi.fast_read_data(address | offset)
        rdata_full.extend(rdata)
    for wby, rby in zip(wdata_full, rdata_full):
        assert wby == rby

    # Erase!
    if mock:
        spi_flash.qspi._qspi_receive.return_value = 0

    _qspi_ret(0)
    spi_flash.qspi.sector_erase(address)

    # Make sure it's 0xff
    rdata_full = []
    for pg in range(4):
        offset = pg * wb.W25Q32JW_FAST_READ_SIZE
        _qspi_ret([0xff] * wb.W25Q32JW_FAST_READ_SIZE)
        rdata = spi_flash.qspi.fast_read_data(address | offset)
        rdata_full.extend(rdata)
    for byte in rdata_full:
        assert byte == 0xff, f'\\x{byte:02x} != \\xff'


@pytest.mark.usefixtures('mock_only')
def test_block_erase(mock, spi_flash):
    if not mock:
        pytest.skip("Don't erase the spi flash.")

    def _qspi_ret(r):
        return _recv_qspi_ret(mock, spi_flash, r)

    address = 0x200000

    # Erase 32k block first
    _qspi_ret(0)
    spi_flash.qspi.block_erase(address, full_block=False)

    # read out 32k and ensure all 0xff
    for fr in range(32):
        offset = fr * wb.W25Q32JW_FAST_READ_SIZE
        _qspi_ret([0xff] * wb.W25Q32JW_FAST_READ_SIZE)
        rdata = spi_flash.qspi.fast_read_data(address | offset)
        for byte in rdata:
            assert byte == 0xff

    # Write to 32k block in 256byte chunks
    wdata_full = []
    for pg in range(32 * KIB // wb.W25Q32JW_PAGE_SIZE):
        wdata = [random.randint(0, 0xff) for _ in range(wb.W25Q32JW_PAGE_SIZE)]
        wdata_full.extend(wdata)
        offset = pg * wb.W25Q32JW_PAGE_SIZE
        _qspi_ret(0)
        spi_flash.qspi.page_program(address | offset, wdata)

    # Read back to make sure there is data in the sector
    # fast read gets 1024 bytes
    rdata_full = []
    # Read out 4k
    for fr in range(32):
        offset = fr * wb.W25Q32JW_FAST_READ_SIZE
        wdata_slice = slice(offset, (fr + 1) * wb.W25Q32JW_FAST_READ_SIZE)
        _qspi_ret(wdata_full[wdata_slice])

        rdata = spi_flash.qspi.fast_read_data(address | offset)
        rdata_full.extend(rdata)
    for wby, rby in zip(wdata_full, rdata_full):
        assert wby == rby

    # Erase 32k block!
    _qspi_ret(0)
    spi_flash.qspi.block_erase(address, full_block=False)

    # Make sure it's 0xff
    for fr in range(32):
        offset = fr * wb.W25Q32JW_FAST_READ_SIZE
        _qspi_ret([0xff] * wb.W25Q32JW_FAST_READ_SIZE)
        rdata = spi_flash.qspi.fast_read_data(address | offset)
        for byte in rdata:
            assert byte == 0xff, f'\\x{byte:02x} != \\xff'


@pytest.mark.usefixtures('mock_only')
def test_block_erase_full(mock, spi_flash):
    if not mock:
        pytest.skip("Don't erase the spi flash.")

    def _qspi_ret(r):
        return _recv_qspi_ret(mock, spi_flash, r)

    address = 0x200000

    # Erase 32k block first
    _qspi_ret(0)
    spi_flash.qspi.block_erase(address, full_block=True)

    # read out 64k and ensure all 0xff
    for fr in range(64):
        offset = fr * wb.W25Q32JW_FAST_READ_SIZE
        _qspi_ret([0xff] * wb.W25Q32JW_FAST_READ_SIZE)
        rdata = spi_flash.qspi.fast_read_data(address | offset)
        for byte in rdata:
            assert byte == 0xff

    # Write to 64k block in 256byte chunks
    wdata_full = []
    for pg in range(64 * KIB // wb.W25Q32JW_PAGE_SIZE):
        wdata = [random.randint(0, 0xff) for _ in range(wb.W25Q32JW_PAGE_SIZE)]
        wdata_full.extend(wdata)
        offset = pg * wb.W25Q32JW_PAGE_SIZE
        _qspi_ret(0)
        spi_flash.qspi.page_program(address | offset, wdata)

    # Read back to make sure there is data in the sector
    # fast read gets 1024 bytes
    rdata_full = []
    # Read out 4k
    for fr in range(64):
        offset = fr * wb.W25Q32JW_FAST_READ_SIZE
        wdata_slice = slice(offset, (fr + 1) * wb.W25Q32JW_FAST_READ_SIZE)
        _qspi_ret(wdata_full[wdata_slice])

        rdata = spi_flash.qspi.fast_read_data(address | offset)
        rdata_full.extend(rdata)
    for wby, rby in zip(wdata_full, rdata_full):
        assert wby == rby

    # Erase 64k block!
    _qspi_ret(0)
    spi_flash.qspi.block_erase(address, full_block=True)

    # Make sure it's 0xff
    for fr in range(64):
        offset = fr * wb.W25Q32JW_FAST_READ_SIZE
        _qspi_ret([0xff] * wb.W25Q32JW_FAST_READ_SIZE)
        rdata = spi_flash.qspi.fast_read_data(address | offset)
        for byte in rdata:
            assert byte == 0xff, f'\\x{byte:02x} != \\xff'


@pytest.mark.skip("Only check sparingly since program-erase endurance is "
                  "limited to 100K cycles for Winbond W25Q32JW.")
def test_program_erase(mock, spi_flash):
    if not mock:
        pytest.skip("Only check sparingly since program-erase endurance is "
                    "limited to 100K cycles for Winbond W25Q32JW.")

    periph = M30_SPI_FLASH_MEMORY_MAP.bitstream_jump_table
    sector_addr = periph.get_field_addr('scratch_sector')
    num_reads = wb.W25Q32JW_SECTOR_SIZE // wb.W25Q32JW_FAST_READ_SIZE
    num_programs = wb.W25Q32JW_SECTOR_SIZE // wb.W25Q32JW_PAGE_SIZE
    assert num_reads == 4
    assert num_programs == 16

    # Test sector erase (include blocking functionality)
    spi_flash.qspi.sector_erase(sector_addr, blocking=False)
    assert spi_flash.qspi.flash_is_busy
    time.sleep(2 * wb.W25Q32JW_SECTOR_ERASE_TIME_NS / 1e9)
    assert not spi_flash.qspi.flash_is_busy

    for i in range(num_reads):
        addr = sector_addr + i * wb.W25Q32JW_FAST_READ_SIZE
        rdata = spi_flash.qspi.fast_read_data(addr)
        assert rdata == bytearray([0xff] * wb.W25Q32JW_FAST_READ_SIZE)

    # Test page program
    wdata = []
    for i in range(num_programs):
        addr = sector_addr + i * wb.W25Q32JW_PAGE_SIZE
        data = [random.randint(0, 0xff) for _ in range(wb.W25Q32JW_PAGE_SIZE)]
        wdata.append(bytearray(data))
        spi_flash.qspi.page_program(addr, data)

    for i in range(num_programs):
        addr = sector_addr + i * wb.W25Q32JW_PAGE_SIZE
        data = wdata[i]
        rdata = spi_flash.qspi.fast_read_data(addr, length=len(data))
        assert rdata == data
