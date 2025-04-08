"""
file: mcs_updater.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file provides logic for updating the MCS files written
to the SPI Flash for the FPGA image and LCM voltage patterns.
"""
import time
from pathlib import Path
import queue
from typing import Union, List, Sequence

import numpy as np

from cobra_system_control.cobra_log import log
from cobra_system_control.mcs_reader import BinMcsReader
import cobra_system_control.w25q32jw_const as wb


FOD = Path(__file__).parent.parent.absolute()


def sensors_to_upgrade_fpga(
        c: 'Cobra', released_sha, allow_upgrade: bool = False) -> bool:
    """ Checks FPGA version against known compatible shas
    ## Added 20211029 added fpga sha check
    Returns a list of sensors that need to be upgraded
    """
    sen = c.sen
    # Git sha check for updating FPGA
    gsha, _ = sen.read_git_sha((c.golden_sha,))
    log.debug('Detected git sha 0x%x for sensor', gsha)

    if gsha != released_sha:
        if allow_upgrade:
            msg = (f'Sensor head version {gsha:#010x} '
                   f'does not match approved version {released_sha:#010x}; '
                   'will update')
            log.info(msg)
            c.msg_queue.put(msg)
            return True
        else:
            msg = (f'After FPGA update, sensor version {gsha:#010x} '
                   f'does not match approved version  {released_sha:#010x}')
            log.error(msg)
            c.msg_queue.put(msg)
            return False
    else:
        return False


def sensors_to_upgrade_lcm(
        c: 'Cobra', allow_upgrade: bool = False) -> bool:
    """Checks the LCM type in the SPI flash and makes sure the
    voltage or delta voltage patterns in the SPI flash are correct
    """
    sen = c.sen

    bin_out = sen.lcm_assembly.lcm_bin_path
    reader = BinMcsReader(bin_out)
    # Verify if it matches what is currently in the SPI flash
    num_tables = len(sen.lcm_assembly.orders)
    verified = verify_lcm_tables(
        sen.spi_flash, reader, c.msg_queue,
        [0, 1, num_tables//2, num_tables-2, num_tables-1])

    if not verified:
        msg = 'Sensor head pattern tables do not match for lcm'
        log.info(msg)
        c.msg_queue.put(msg)
        if allow_upgrade:
            return True
        else:
            return False
    else:
        return False


def final_check_pages(pages: List[bytearray], addr_base: int, addr_max: int):
    """Checks list of pages to make sure they are all the correct size
    and that they fit within the preallocated space
    """
    for page in pages:
        if len(page) != wb.W25Q32JW_PAGE_SIZE:
            raise ValueError(f'Page is not length 256, is length {len(page)}')
    if len(pages) > ((addr_max + 1 - addr_base) / wb.W25Q32JW_PAGE_SIZE):
        raise ValueError(
            f'Bitstream is too big. About to write from {addr_base:#010x} '
            f'to {addr_max:#010x}. '
            f' {len(pages)*256:#010_x}')


def delete_memory(sf: 'SpiFlash', addr_base: int, addr_max: int):
    """The bitstream is located from memory address 0x00_0000 to 0x10_0000
    which is 1024k. We can delete in 64k blocks with is 1024/64 = 16 erases.

    LCM tables are located from memory address 0x10_0000 to 0x18_0000
    which is 512k. We can delete in 64k blocks with is 512/64 = 8 erases.
    """
    num_erases = int(((addr_max) - addr_base) / wb.W25Q32JW_FULL_BLOCK_SIZE)
    log.debug('Erasing %s blocks', num_erases)
    for big_block in range(num_erases):
        addr = addr_base | (big_block * wb.W25Q32JW_FULL_BLOCK_SIZE)
        log.info('Deleting Addr 0x%x', addr)
        sf.qspi.block_erase_full(addr)


def log_fpga_update_msg(msg, q=None):
    """
    Log a message to the cobra log and to the queue if it exists
    """
    log.info(msg)
    if q:
        q.put(msg)


def write_memory(addr_base: int, num_pages: int,
                 sf: 'SpiFlash', mr: BinMcsReader,
                 q: queue.Queue):
    """Writes data provided by the McsReader to the SPI Flash
    given the address base and number of pages.
    """
    pstart = time.time()
    for page in range(num_pages):
        addr = addr_base + (page * wb.W25Q32JW_PAGE_SIZE)
        if addr % 0x10000 == 0:
            log.info('Programming Addr 0x%x', addr)
            if q:
                q.put('Programming progress: '
                      f'{page * 100 / num_pages:.0f}%')
        sf.qspi.page_program(addr, mr[addr:addr+wb.W25Q32JW_PAGE_SIZE])
    log.debug('Programming_time: %i s', time.time() - pstart)


def update_fpga(sf: 'SpiFlash', bin_path: Union[str, Path],
                q: queue.Queue = None):
    """Updates and verifies rewriting of the FPGA bitstream
    in the SPI Flash
    """
    mr = BinMcsReader(bin_path)
    start = time.time()
    delstart = time.time()
    addr_min = sf.memmap_periph.bitstream_primary.addr_base
    addr_max = addr_min + sf.memmap_periph.bitstream_primary.size
    # Check that the MCS has replacement data for what we are going to delete
    if addr_max > mr.addr_max:
        raise ValueError(
            'MCS data does not fill the FPGA bitstream partition'
            f' only {mr.addr_max} is defined but should be up to {addr_max}')

    delete_memory(sf, addr_min, addr_max)
    log.debug('delete_time=%i', time.time() - delstart)
    num_pages = (addr_max - addr_min) // wb.W25Q32JW_PAGE_SIZE
    write_memory(addr_min, num_pages, sf, mr, q)

    vstart = time.time()
    # Verifying subset
    verified = verify_fpga(sf, mr, q)
    log.debug('Verify time: %i seconds', time.time() - vstart)
    log_fpga_update_msg(
        f'Sensor head update time: {time.time() - start:.0f} s', q)
    return verified


def update_lcm(sf: 'SpiFlash', bin_path: Union[str, Path],
               q: queue.Queue = None):
    """Updates and verifies rewriting of the lcm patterns
    in the spi flash
    """
    mr = BinMcsReader(bin_path)
    start = time.time()
    delstart = time.time()
    addr_min = sf.memmap_periph.lcm_patterns.addr_base
    addr_max = addr_min + sf.memmap_periph.lcm_patterns.size
    # Check that the MCS has replacement data for what we are going to delete
    if addr_max > mr.addr_max:
        raise ValueError('MCS data does not fill the LCM pattern partition'
                         f' only {mr.addr_max} is defined but should be up to'
                         f' {addr_max}')

    delete_memory(sf, addr_min, addr_max)
    log.debug('delete_time=%i', time.time() - delstart)
    num_pages = (addr_max - addr_min) // wb.W25Q32JW_PAGE_SIZE
    write_memory(addr_min, num_pages, sf, mr, q)

    vstart = time.time()
    # Verifying subset
    verified = verify_lcm_tables(sf, mr, q)
    log.info('Verify time: %i seconds', time.time() - vstart)
    log_fpga_update_msg(f'Sensor head update time: {time.time() - start:.0f} s', q)
    return verified


def verify_sf(addr_base: int, chunks: Sequence[int],
              sf: 'SpiFlash', mr: BinMcsReader, q: queue.Queue):
    """Verifies the contents of the spi flash against the mcs loaded
    """
    if not isinstance(chunks, (list, tuple, np.ndarray)):
        raise TypeError('chunks must be a list, tuple, or ndarray '
                        f'but is {type(chunks)}')
    log.debug('Verifying %i chunks', len(chunks))
    last_printed_idx = 0
    for idx, ck in enumerate(chunks):
        addr = addr_base + (ck * wb.W25Q32JW_FAST_READ_SIZE)
        sf_ba = sf.qspi.fast_read_data(addr,
                                       length=wb.W25Q32JW_FAST_READ_SIZE)
        mcs_ba = mr[addr:addr+wb.W25Q32JW_FAST_READ_SIZE]
        perc_verified = idx * 100 // len(chunks)
        if (perc_verified // 10 != last_printed_idx):
            log.info('Verifying addr = 0x%x, progress: %i perc',
                     addr, perc_verified)
            if q:
                q.put(f'Verification progress: {perc_verified}%')
            last_printed_idx = perc_verified // 10

        if sf_ba != mcs_ba:
            log_fpga_update_msg(
                f'Verification failure of update near address {addr}')
            return False
    return True


def verify_fpga(sf: 'SpiFlash', mr: BinMcsReader, q: queue.Queue):
    """Reads the primary bitstream in the SPI Flash and compares
    it to the MCS file to ensure matching
    """
    addr_min = sf.memmap_periph.bitstream_primary.addr_base
    chunks = sf.memmap_periph.bitstream_primary.size // wb.W25Q32JW_FAST_READ_SIZE
    return verify_sf(addr_min, list(range(chunks)), sf, mr, q)


def verify_lcm_tables(sf: 'SpiFlash', mr: BinMcsReader, q: queue.Queue,
                      chunks: Union[int, List] = None):
    """Reads a number of chunks in the SPI Flash where the lcm
    pattern tables sit and makes sure they match the generated MCS
    """
    max_chunk = sf.memmap_periph.lcm_patterns.size // wb.W25Q32JW_FAST_READ_SIZE
    if chunks is None:
        chunks = range(max_chunk)
    else:
        if isinstance(chunks, int):
            chunks = [chunks]

    chunks = [x for x in chunks if x < max_chunk]
    chunks = sorted(chunks)

    # Chunks are read 1K at a time. Make sure we don't read too many
    address_start = sf.memmap_periph.lcm_patterns.addr_base
    return verify_sf(address_start, chunks, sf, mr, q)
