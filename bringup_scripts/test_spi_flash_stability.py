import random
import sys
import time
from pathlib import Path
from typing import Union

import numpy as np

from cobra_system_control.comms import Ad2I2C, I2CBus
from cobra_system_control.config import AD2_SERIAL_NUMBER
from cobra_system_control.spi_flash import SpiFlash
import cobra_system_control.w25q32jw_const as wb


KIB = 1024


def test_sector_reads_empty_data(spi_flash):
    print('\nTesting empty data')
    address = 0x200000
    bads = 0

    # Need to erase before writing!
    spi_flash.sector_erase(address)

    niter = 10
    # Read the sector a bunch of times
    for i in range(niter):
        # Read out 4k
        print('iter', i)
        for pg in range(4):
            offset = pg * wb.W25Q32JW_FAST_READ_SIZE

            rdata = spi_flash.fast_read_data(address | offset)
            for idx, byte in enumerate(rdata):
                if byte != 0xff:
                    print(f'byte {pg * 256 + idx} is wrong')
                    bads += 1
    total_bytes = niter * 4 * wb.W25Q32JW_FAST_READ_SIZE
    print(f'{bads} Bads. Bad byte percent of {total_bytes} bytes = {bads / total_bytes * 100:0.2f} %')


def test_block_reads_empty_data(spi_flash):
    print('\nTesting empty data')
    address = 0x200000
    bads = 0
    # Erase 32k block first
    spi_flash.block_erase(address, full_block=False)

    niter = 3
    # Read the sector a bunch of times
    for i in range(niter):
        # Read out 32k
        print('iter', i)
        for pg in range(32):
            offset = pg * wb.W25Q32JW_FAST_READ_SIZE

            rdata = spi_flash.fast_read_data(address | offset)
            for idx, byte in enumerate(rdata):
                if byte != 0xff:
                    print(f'byte {pg * 256 + idx} is wrong')
                    bads += 1
    total_bytes = niter * 32 * wb.W25Q32JW_FAST_READ_SIZE
    print(f'{bads} Bads. Bad byte percent of {total_bytes} bytes = {bads / total_bytes * 100:0.2f} %')


def test_sector_reads_written_data(spi_flash):
    print('\nTesting written data')
    address = 0x200000
    bads = 0

    niter = 10
    # Read the sector a bunch of times
    for i in range(niter):
        print('iter', i)
        # Need to erase before writing!
        spi_flash.sector_erase(address)
        # Page program is 256 but need to write 4k
        wdata_full = []
        for pg in range(4 * KIB // wb.W25Q32JW_PAGE_SIZE):
            wdata = [random.randint(0, 0xff)
                     for i in range(wb.W25Q32JW_PAGE_SIZE)]
            wdata_full.extend(wdata)
            offset = pg * wb.W25Q32JW_PAGE_SIZE
            spi_flash.page_program(address | offset, wdata)
        # Read back to make sure there is data in the sector
        # fast read gets 1024 bytes
        rdata_full = []
        # Read out 4k
        for pg in range(4):
            offset = pg * wb.W25Q32JW_FAST_READ_SIZE
            rdata = spi_flash.fast_read_data(address | offset)
            rdata_full.extend(rdata)

        for idx, (wby, rby) in enumerate(zip(wdata_full, rdata_full)):
            if wby != rby:
                print(f'byte {idx} is wrong')
                bads += 1
    total_bytes = niter * 4 * wb.W25Q32JW_FAST_READ_SIZE
    print(f'{bads} Bads. Bad byte percent of {total_bytes} bytes = {bads / total_bytes * 100:0.2f} %')


def test_block_reads_written_data(spi_flash):
    print('\nTesting written data')
    address = 0x200000
    bads = 0

    niter = 3
    # Read the sector a bunch of times
    for i in range(niter):
        print('iter', i)
        # Erase 32k block first
        spi_flash.block_erase(address, full_block=False)
        # Page program is 256 but need to write 32k
        wdata_full = []
        for pg in range(32 * KIB // wb.W25Q32JW_PAGE_SIZE):
            wdata = [random.randint(0, 0xff)
                     for i in range(wb.W25Q32JW_PAGE_SIZE)]
            wdata_full.extend(wdata)
            offset = pg * wb.W25Q32JW_PAGE_SIZE
            spi_flash.page_program(address | offset, wdata)
        # Read back to make sure there is data in the sector
        # fast read gets 1024 bytes
        rdata_full = []
        # Read out 4k
        for pg in range(32):
            offset = pg * wb.W25Q32JW_FAST_READ_SIZE
            rdata = spi_flash.fast_read_data(address | offset)
            rdata_full.extend(rdata)

        for idx, (wby, rby) in enumerate(zip(wdata_full, rdata_full)):
            if wby != rby:
                print(f'byte {idx} is wrong')
                bads += 1
    total_bytes = niter * 32 * wb.W25Q32JW_FAST_READ_SIZE
    print(f'{bads} Bads. Bad byte percent of {total_bytes} bytes = {bads / total_bytes * 100:0.2f} %')




if __name__ == "__main__":
    i2c_bus = Ad2I2C(AD2_SERIAL_NUMBER)
    spi_flash = SpiFlash(i2c_bus, 0x11)
    spi_flash.connect()
    # reset the spi flash
    spi_flash.send_qspi([wb.QSPI_CMD_ENABLE_RESET])
    spi_flash.send_qspi([wb.QSPI_CMD_RESET_DEVICE])
    # 30us reset time listed in the Windbond spec
    time.sleep(0.1)  # reset time per Winbond spec

    test_sector_reads_empty_data(spi_flash)
    test_sector_reads_written_data(spi_flash)
    test_block_reads_empty_data(spi_flash)
    test_block_reads_written_data(spi_flash)
