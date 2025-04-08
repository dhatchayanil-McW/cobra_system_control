"""
file: spi_flash.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines drivers to erase and program the
SPI Flash in the correct sizes through
Quad SPI serial communication.
Constants related to the specific SPI Flash
part are in w25q32jw_const.py.
"""
from typing import Union, List, Dict, Tuple

import numpy as np

from cobra_system_control.device import Device
from cobra_system_control.cobra_log import log
from cobra_system_control.mcs_reader import BinMcsReader
import cobra_system_control.w25q32jw_const as wb


class QspiController(Device):
    """A class to encapsulate the QSPI interface to the flash memory and the
    FPGA controller that implements it."""

    RX_FIFO_DEPTH = 1024
    TX_FIFO_DEPTH = 1024

    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph'):
        self.addr_bigendian = True
        self.data_bigendian = True
        super().__init__(bus, device_addr, 2, 1, memmap_periph,
                         addr_bigendian=self.addr_bigendian,
                         data_bigendian=self.data_bigendian)
        self.memmap_periph = memmap_periph

    def _address_to_bytearray(self, address):
        # QSPI addresses are always MSB first
        return bytearray(address.to_bytes(3, byteorder='big'))

    def _qspi_send(self, wdata: List[int]):
        """Add bytes to the TX FIFO. Dummy bytes and fill byte for RX should
        not be added. Both FIFOs are cleared before filling the TX FIFO.
        """
        if len(wdata) > QspiController.TX_FIFO_DEPTH:
            raise RuntimeError(f'Sending too many bytes ({len(wdata)}).')

        self.write_fields(fifo_tx_release=0)
        self.write_fields(reset=0x1 | 0x2)  # fifo_tx_reset | fifo_rx_reset
        self.i2c.write_bytes(self.memmap_periph.get_field_addr('fifo_tx_wdata'), bytearray(wdata))
        self.write_fields(fifo_tx_release=1)
        while self.read_fields('spi_busy'):
            # I2C transactions are slow enough that we don't need a sleep here
            continue
        self.write_fields(fifo_tx_release=0)

    def _qspi_receive(
            self,
            return_byte_array: bool = False,
            length: Union[int, None] = None,
    ) -> Union[int, bytearray]:
        """Receive data from the RX FIFO. If `length` is None, data is returned
        until the RX FIFO empties; otherwise, `length` bytes are returned. This
        behavior reduces unnecessary I2C traffic if the user desires less data
        than the FIFO contains. This function guarantees that the RX FIFO is
        empty upon returning.
        """
        if length and length > QspiController.RX_FIFO_DEPTH:
            raise RuntimeError(f'Requesting too many bytes ({length}).')

        data = []
        for _ in range(QspiController.RX_FIFO_DEPTH):
            done = (self.read_fields('fifo_rx_empty') == 1 or
                    length is not None and len(data) == length)
            if done:
                break
            data.append(self.read_fields('fifo_rx_rdata'))
        if length is not None and len(data) != length:
            msg = f'Failed to return the requested length = {length} bytes.'
            raise RuntimeError(msg)
        self.write_fields(reset='fifo_rx_reset')

        if return_byte_array:
            return bytearray(data)
        else:
            # Interpret data as a single int sent MSB/MSb first
            return int.from_bytes(bytes(data), byteorder='big')

    @property
    def flash_is_busy(self) -> bool:
        """Returns whether the flash memory is presently busy with an erase
        or write operation. During this time the flash memory will ignore
        all instructions except Read Status Register commands.

        This command modifies the TX and RX FIFOs since it uses Read Status
        Register commands.
        """
        self._qspi_send([wb.QSPI_CMD_READ_SR_1])
        rdata = self._qspi_receive()
        return bool(rdata & 0x1)

    def get_write_enable(self) -> bool:
        """Returns the flash memory's WEL (write enable latch) bit, which
        indicates whether erasing and programming instructions are accepted.
        WEL clears automatically with each erase or program.

        This command modifies the TX and RX FIFOs.
        """
        self._qspi_send([wb.QSPI_CMD_READ_SR_1])
        rdata = self._qspi_receive()
        return bool((rdata >> 1) & 0x1)

    def set_write_enable(self, val: bool):
        """Sets the flash memory's WEL (write enable latch) bit, which
        indicates whether erasing and programming instructions are accepted.
        WEL clears automatically with each erase or program.

        This command modifies the TX FIFO.
        """
        cmd = wb.QSPI_CMD_WR_EN if bool(val) else wb.QSPI_CMD_WRITE_DISABLE
        self._qspi_send([cmd])

    def read_mfg_device_id(self) -> int:
        """Returns the MFG DEVICE ID as an int (e.g. 0xef15)."""
        self._qspi_send([
            wb.QSPI_CMD_MFG_DEVICE_ID,
            0x00, 0x00, 0x00,
        ])
        return self._qspi_receive()

    def read_jedec_id(self) -> int:
        """Returns the JEDEC ID as an int (e.g. 0xef6016)."""
        self._qspi_send([wb.QSPI_CMD_JEDEC_ID])
        return self._qspi_receive()

    def read_unique_id(self) -> int:
        """Returns a 64-bit unique number from the flash memory. The
        number is factory-set and read-only.
        """
        self._qspi_send([wb.QSPI_CMD_READ_UNIQUE_ID])
        return self._qspi_receive()

    def fast_read_data(
            self,
            address: int,
            length=None,
    ) -> bytearray:
        """Returns data read from the flash memory at the specified address.
        The RX FIFO is filled with 1 KiB by the H/W and these bytes are all
        returned unless `length` is specified, in which case only the specified
        number of bytes are read from the FIFO. This behavior reduces unnecessary
        I2C traffic. The RX FIFO is guaranteed to be empty upon returning.
        """
        length = length or QspiController.RX_FIFO_DEPTH
        self._qspi_send([
            wb.QSPI_CMD_FAST_READ,
            *self._address_to_bytearray(address),
        ])

        read_bytes = self.i2c.read_bytes(
            self.memmap_periph.get_field_addr('fifo_rx_rdata'),
            length, inc_addr=False)
        self.write_fields(reset='fifo_rx_reset')
        return read_bytes

    def page_program(
            self,
            address: int,
            data: List[int],
            blocking: bool = True,
    ):
        """Program flash memory. Up to a page of memory (1 to 256 bytes) may
        be programmed at a time but the memory must have been previously
        erased. This function prevents writing beyond a page boundary. If the
        `blocking` argument is False then the function will return as soon
        as the Page Program command is sent; otherwise, the function will
        poll the memory until the memory is done with programming.
        """
        length = len(data)
        if not 0 < length <= wb.W25Q32JW_PAGE_SIZE:
            msg = (f"Attempting to program {length} bytes but a page is "
                   f"limited to [1, {wb.W25Q32JW_PAGE_SIZE}] bytes.")
            raise RuntimeError(msg)
        end = (address % wb.W25Q32JW_PAGE_SIZE) + length
        if end > wb.W25Q32JW_PAGE_SIZE:
            msg = (f"Attempting to program beyond a page boundary (address = "
                   f"0x{address:x}, length = {length}), which is not "
                   f"allowed.")
            raise RuntimeError(msg)

        # must enable writes first
        self._qspi_send([wb.QSPI_CMD_WR_EN])
        self._qspi_send([
            wb.QSPI_CMD_PAGE_PROGRAM,
            *self._address_to_bytearray(address),
            *data,
        ])
        while blocking and self.flash_is_busy:
            # I2C transactions are slow enough that we don't need a sleep here
            continue

    def sector_erase(self, address: int, blocking: bool = True):
        """Erases 4 KiB of memory. Memory must be erased prior to programming.
        If the `blocking` argument is False then the function will return as
        soon as the Erase command is sent; otherwise, the function will poll
        the memory until the memory is done erasing.
        """
        if address % wb.W25Q32JW_SECTOR_SIZE != 0:
            msg = (f"The address (0x{address:x}) must fall on a sector "
                   f"boundary (i.e. be a multiple of "
                   f"0x{wb.W25Q32JW_SECTOR_SIZE:x}).")
            raise RuntimeError(msg)

        # must enable writes first
        self._qspi_send([wb.QSPI_CMD_WR_EN])
        self._qspi_send([
            wb.QSPI_CMD_SECTOR_ERASE,
            *self._address_to_bytearray(address),
        ])
        while blocking and self.flash_is_busy:
            # I2C transactions are slow enough that we don't need a sleep here
            continue

    def block_erase(
            self,
            address: int,
            full_block: bool = False,
            blocking: bool = True,
    ):
        """Erases 32 or 64 KiB of memory according to `full_block`. Memory
        must be erased prior to programming. If the `blocking` argument is
        False then the function will return as soon as the Erase command is
        sent; otherwise, the function will poll the memory until the memory
        is done erasing.
        """
        if full_block:
            command = wb.QSPI_CMD_BLOCK_ERASE_FULL
            size = wb.W25Q32JW_FULL_BLOCK_SIZE
        else:
            command = wb.QSPI_CMD_BLOCK_ERASE_HALF
            size = wb.W25Q32JW_HALF_BLOCK_SIZE
        if address % size != 0:
            msg = (f"The address (0x{address:x}) must fall on a block "
                   f"boundary (i.e. be a multiple of 0x{size:x}).")
            raise RuntimeError(msg)

        # must enable writes first
        self._qspi_send([wb.QSPI_CMD_WR_EN])
        self._qspi_send([
            command,
            *self._address_to_bytearray(address),
        ])
        while blocking and self.flash_is_busy:
            # I2C transactions are slow enough that we don't need a sleep here
            continue

    def block_erase_full(self, address: int, blocking: bool = True):
        self.block_erase(address, full_block=True, blocking=blocking)

    def block_erase_half(self, address: int, blocking: bool = True):
        self.block_erase(address, full_block=False, blocking=blocking)


class SpiFlashPartition:
    """This class describes the highest level partitioning in the SPI Flash and
    provides consistent methods for accessing the partition.
    """

    def __init__(
            self,
            qspi: QspiController,
            mcs_reader: BinMcsReader,
            mmp: 'MemoryMapPeriph',
    ):
        self.qspi = qspi
        self.mcs_reader = mcs_reader
        self.mmp = mmp

        if self.mmp.size % wb.W25Q32JW_SECTOR_SIZE != 0:
            msg = (f"The partition's size in bytes isn't a multiple of "
                   f"{wb.W25Q32JW_SECTOR_SIZE:#_x} so it won't be fully "
                   f"erasable and writeable. You must reevaluate the "
                   f"memory map.")
            raise RuntimeError(msg)
        if wb.W25Q32JW_SECTOR_SIZE % wb.W25Q32JW_PAGE_SIZE != 0:
            raise RuntimeError('Error: check the SPI Flash datasheet.')

    def erase_partition(self):
        """Erases the entire partition's memory using the fewest number of
        erase commands possible.
        """
        size = self.mmp.size
        addr = self.mmp.addr_base
        for erase_size, erase_func in (
                (wb.W25Q32JW_FULL_BLOCK_SIZE, self.qspi.block_erase_full),
                (wb.W25Q32JW_HALF_BLOCK_SIZE, self.qspi.block_erase_half),
                (wb.W25Q32JW_SECTOR_SIZE, self.qspi.sector_erase),
        ):
            num_erases = size // erase_size
            for _ in range(num_erases):
                log.debug("Erasing 0x%x bytes at address "
                          "0x%x for peripheral %s.",
                          erase_size, addr, self.mmp.name)
                erase_func(addr)
                addr += erase_size
                size -= erase_size
            if size == 0:
                break
        else:
            msg = (f"The partition's size in bytes wasn't a multiple of "
                   f"{wb.W25Q32JW_SECTOR_SIZE:#_x} so it couldn't be fully "
                   f"erased.")
            raise RuntimeError(msg)

    def program_partition(self, skip_empty: bool = True):
        """Programs a partition's memory with the data contents provided by
        the McsReader object. Writing of empty pages (memory already at
        the default value) can be skipped with `skip_empty`.
        """
        for addr in range(
                self.mmp.addr_base,
                self.mmp.size,
                wb.W25Q32JW_PAGE_SIZE,
        ):
            data = self.mcs_reader[addr:addr + wb.W25Q32JW_PAGE_SIZE]
            if skip_empty and all(b == 0xff for b in data):
                log.debug("Skipping address 0x%x for peripheral "
                          "%s (all data is 0xff).", addr, self.mmp.name)
                continue
            log.debug("Programming address 0x%x for peripheral "
                      "%s (first data = 0x%x and "
                      "last data = 0x%x.",
                      addr, self.mmp.name, data[0], data[-1])
            self.qspi.page_program(addr, data)

    def read_and_verify(
            self,
            addr_start: Union[int, None] = None,
            addr_stop: Union[int, None] = None,
    ) -> Dict[int, Tuple[int]]:
        """Read the specified address range in the partition
        """
        ifnone = lambda x, y: x if x is not None else y
        addr_start = ifnone(addr_start, self.mmp.addr_base)
        addr_stop = ifnone(addr_stop, self.mmp.addr_base + self.mmp.size)
        if not 0 <= addr_start - self.mmp.addr_base < self.mmp.size:
            msg = (f"Starting address {addr_start:#_x} is not within this "
                   f"partition's address range.")
            raise ValueError(msg)
        if not addr_start < addr_stop <= self.mmp.addr_base + self.mmp.size:
            msg = (f"Stopping address {addr_stop:#_x} must be greater than "
                   f"the starting address {addr_start:#_x} and within the "
                   f"partition's address range.")
            raise ValueError(msg)

        size = addr_stop - addr_start
        addr = addr_start
        mismatch = {}
        while size != 0:
            length = min(size, QspiController.RX_FIFO_DEPTH)
            log.debug("Reading 0x%x bytes from address "
                      "0x%x for peripheral %s.",
                      length, addr, self.mmp.name)
            data_mem = self.qspi.fast_read_data(addr, length=length)
            data_mcs = self.mcs_reader[addr:addr + length]
            mismatch.update({
                (addr + i): (x, y) for i, (x, y) in
                enumerate(zip(data_mcs, data_mem)) if x != y
            })
            size -= length
            addr += length
        return mismatch


class SpiFlash():
    """A collection of SpiFlashPartition objects"""
    def __init__(
            self,
            qspi: QspiController,
            memmap_periph: 'MemoryMapPeriph',
            mcs_filename: Union[str, None] = None,
    ):
        self.qspi = qspi
        self.memmap_periph = memmap_periph
        self._mcs_filename = mcs_filename

    @property
    def mcs_filename(self):
        return self._mcs_filename

    @mcs_filename.setter
    def mcs_filename(self, val: str):
        if val is None:
            self.mcs_reader = None
            for p_name, p_obj in self.memmap_periph.periphs.items():
                setattr(self, p_name, None)
        elif val != self._mcs_filename:
            self.mcs_reader = BinMcsReader(val)
            for p_name, p_obj in self.memmap_periph.periphs.items():
                setattr(self, p_name, SpiFlashPartition(
                    self.qspi, self.mcs_reader, p_obj,
                ))
        self._mcs_filename = val

    @property
    def partitions(self):
        return [getattr(self, p_name) for p_name, p_obj in
                sorted(self.memmap_periph.periphs.items(),
                       key=lambda x: x[1].addr_base)]


# legacy cal stuff; see cobra_system_control/sensor_head.py
def pages_from_size(size_bytes: int) -> int:
    return int(np.ceil(
        size_bytes / wb.W25Q32JW_PAGE_SIZE))


# legacy cal stuff; see cobra_system_control/sensor_head.py
def reads_from_size(size_bytes: int) -> int:
    return int(np.ceil(
            size_bytes / wb.W25Q32JW_FAST_READ_SIZE))


# legacy cal stuff; see cobra_system_control/sensor_head.py
def sectors_from_size(size_bytes: int):
    return int(np.ceil(
            size_bytes / wb.W25Q32JW_SECTOR_SIZE))


# legacy cal stuff; see cobra_system_control/sensor_head.py
def ba_to_pages(ba: bytearray) -> List[bytearray]:
    """Creates a list of bytearrays in page program sizes
    (256 bytes). Zeros are extend onto the end of the result
    of to_bytearray() to ensure all bytearrays in the list
    are of len 256 bytes.
    """
    missing_bytes = (wb.W25Q32JW_PAGE_SIZE
                     - (len(ba) %
                        wb.W25Q32JW_PAGE_SIZE))
    cal_array = ba.copy()
    cal_array.extend(bytearray(missing_bytes))
    pages = []
    for page in range(pages_from_size(len(cal_array))):
        pages.append(cal_array[
            page
            * wb.W25Q32JW_PAGE_SIZE:(page + 1)
            * wb.W25Q32JW_PAGE_SIZE])
    return pages
