import io
import struct
import fcntl

import logging
from threading import Lock
from typing import *
import time
###from cobradriver import I2CBus

#from cobra_system_control.spi_flash import QspiController
from cobra_system_control.compute import nxp_enable

import cobra_memory_map.w25q32jw_const as wb
from cobra_memory_map import M25FpgaMemoryMap, M25SpiFlashMemoryMap, MemoryMapPeriph


STRUCT_MAP = {
    1: 'B',
    2: 'H',
    4: 'I',
    8: 'Q',
}


class I2CBus:
    def __init__(self, bus: int, ioctl_i2c_slave: int):
        self.resource = f'/dev/i2c-{bus}'
        self.lock = Lock()
        self.fw = None
        self.fr = None
        self._is_connected = False
        self.ioctl_i2c_slave = ioctl_i2c_slave

    def connect(self):
        if not self._is_connected:
            self.fw = io.open(self.resource, "wb", buffering=0)
            self.fr = io.open(self.resource, "rb", buffering=0)
            self._is_connected = True

    def write(self, device_addr: int, ba: bytearray):
        with self.lock:
            fcntl.ioctl(self.fw, self.ioctl_i2c_slave, device_addr)
            self.fw.write(ba)

    def read(self, device_addr: int, ba: bytearray, nbytes: int) -> bytearray:
        with self.lock:
            fcntl.ioctl(self.fw, self.ioctl_i2c_slave, device_addr)
            fcntl.ioctl(self.fr, self.ioctl_i2c_slave, device_addr)
            self.fw.write(ba)
            ret = self.fr.read(nbytes)
        return ret

    def disconnect(self):
        if self.fw is not None:
            self.fw.close()
        if self.fr is not None:
            self.fr.close()
        self._is_connected = False


class I2CDevice:
    def __init__(self, bus: 'I2CBus',
                 device_addr: int, addr_nbytes: int, data_nbytes: int,
                 addr_bigendian: bool, data_bigendian: bool):
        self.bus = bus
        self.device_addr = device_addr
        self.addr_nbytes = addr_nbytes
        self.data_nbytes = data_nbytes
        addr_endian = ">" if addr_bigendian else "<"
        data_endian = ">" if data_bigendian else "<"
        self.addr_pack = f'{addr_endian}{STRUCT_MAP[addr_nbytes]}'
        self.data_pack = f'{data_endian}{STRUCT_MAP[data_nbytes]}'

    def write(self, addr: int, data: int):
        ba = bytearray([
            *struct.pack(self.addr_pack, addr),
            *struct.pack(self.data_pack, data)
        ])
        self.bus.write(self.device_addr, ba)
        #self.simple.write_raw(ba)

    def read(self, addr: int) -> int:
        ba = bytearray([*struct.pack(self.addr_pack, addr)])
        ba = self.bus.read(self.device_addr, ba, self.data_nbytes)
        #ba = self.simple.write_read_raw(ba, self.data_nbytes)
        return struct.unpack(self.data_pack, ba)[0]



class Device:
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 addr_bytes: int, data_bytes: int,
                 mmap_periph: MemoryMapPeriph,
                 addr_bigendian: bool, data_bigendian: bool):
        self.i2c = I2CDevice(bus, device_addr, addr_bytes, data_bytes,
                             addr_bigendian, data_bigendian)
        self.periph = mmap_periph
    def read_all_periph_fields(self, with_print=False):
        return self.periph.read_all_periph_fields(with_print=with_print)

    def connect(self):
        self.i2c.bus.connect()
        self.periph.register_write_callback(self.i2c.write)
        self.periph.register_read_callback(self.i2c.read)
        self.periph.register_readdata_callback(lambda x: x)

    def write_fields(self, **kwargs):
        self.periph.write_fields(**kwargs)

    def read_fields(self, *args, use_mnemonic: bool = False):
        return self.periph.read_fields(*args, use_mnemonic=use_mnemonic)

    def get_pos(self, field_name: str) -> int:
        """Gets the bit position in a word"""
        return self.periph.fields[field_name].pos

    def get_offset(self, field_name: str) -> int:
        """Gets the addr offset of a word"""
        return self.periph.fields[field_name].offset

    def get_abs_addr(self, field_name: str) -> int:
        """Returns the absolute byte address of a word.
        """
        return self.addr_base + self.periph.fields[field_name].offset

    @property
    def addr_base(self):
        return self.periph.addr_base

    def get_size(self, field_name: str) -> int:
        """Returns the size of the field, in bits"""
        return self.periph.fields[field_name].size

RX_FIFO_DEPTH = 1024
TX_FIFO_DEPTH = 1024

class QspiController(Device):   #QspiController:
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: MemoryMapPeriph):
        self.addr_bigendian = True
        self.data_bigendian = True
        super().__init__(bus, device_addr, 2, 1, memmap_periph,
                         addr_bigendian=self.addr_bigendian,
                         data_bigendian=self.data_bigendian)
        self.memmap_periph = memmap_periph

    # def fast_read_data(self, address, length: int = QspiController.RX_FIFO_DEPTH):
    #     self._qspi_send([
    #         wb.QSPI_CMD_FAST_READ,
    #         *self._address_to_bytearray(address),
    #     ])
    #
    #     num_chunks = 4
    #     read_bytes = bytearray()
    #     for i in range(num_chunks):
    #         rb = self.read_addressed(self.memmap_periph.get_field_addr('fifo_rx_rdata'), length/num_chunks)
    #         print('looped read', len(rb))
    #         read_bytes.extend(rb)
    #         while self.flash_is_busy:
    #             print('flash is busy, sleeping')
    #             time.sleep(0.1)
    #
    #     #read_bytes = self.read_addressed(self.memmap_periph.get_field_addr('fifo_rx_rdata'), length)
    #
    #     self.write_fields(reset='fifo_rx_reset')
    #
    #     return read_bytes
    def setup(self):
        self.write_fields(reset='all_reset')

    def fifo_reset(self):
        self.write_fields(reset='fifo_rx_reset')

    def _qspi_receive(
            self,
            return_byte_array: bool = False,
            length: Union[int, None] = None,
            reset: bool = True,
    ) -> Union[int, bytearray]:
        """Receive data from the RX FIFO. If `length` is None, data is returned
        until the RX FIFO empties; otherwise, `length` bytes are returned. This
        behavior reduces unnecessary I2C traffic if the user desires less data
        than the FIFO contains. This function guarantees that the RX FIFO is
        empty upon returning.
        """
        if length and length > RX_FIFO_DEPTH:
            raise RuntimeError(f'Requesting too many bytes ({length}).')

        data = []
        print('full', self.read_fields('fifo_rx_full'), 'empty', self.read_fields('fifo_rx_empty'))

        for i in range(RX_FIFO_DEPTH):
            if self.read_fields('fifo_rx_empty') == 1:
                break
            if (length is not None) and (len(data) == length):
                break
            data.append(self.read_fields('fifo_rx_rdata'))

        if (length is not None) and (len(data) != length):
            msg = f'Failed to return the requested length = {length} bytes, is {len(data)}.'
            raise RuntimeError(msg)

        #for i in range(RX_FIFO_DEPTH):
        #    done = (self.read_fields('fifo_rx_empty') == 1 or
        #            length is not None and len(data) == length)
        #    #print(length, len(data), data)
        #    if done:
        #        break
        #    data.append(self.read_fields('fifo_rx_rdata'))

        if reset:
            print('am reseting fifo')
            self.fifo_reset()

        if return_byte_array:
            return bytearray(data)
        else:
            # Interpret data as a single int sent MSB/MSb first
            return int.from_bytes(bytes(data), byteorder='big')

    def _qspi_send(self, wdata):
        if len(wdata) > TX_FIFO_DEPTH:
            raise RuntimeError(f'Sending too many bytes ({len(wdata)}).')
        self.write_fields(fifo_tx_release=0)
        self.write_fields(reset=0x1 | 0x2)  # fifo_tx_reset | fifo_rx_reset
        for byte in wdata:
            self.write_fields(fifo_tx_wdata=byte)

        self.write_fields(fifo_tx_release=1)
        while self.read_fields('spi_busy'):
            # I2C transactions are slow enough that we don't need a sleep here
            continue
        self.write_fields(fifo_tx_release=0)

    def fast_read_data(self, address, length: int = RX_FIFO_DEPTH):
        self._qspi_send([
            wb.QSPI_CMD_FAST_READ,
            *self._address_to_bytearray(address),
        ])
        print('full', self.read_fields('fifo_rx_full'), 'empty', self.read_fields('fifo_rx_empty'))
        num_chunks = 4
        read_bytes = self._qspi_receive(return_byte_array=True)
        #for i in range(num_chunks):
            #rb = self._qspi_receive(return_byte_array=True, length=int(length//num_chunks), reset=False)
            #rb = self.read_addressed(self.memmap_periph.get_field_addr('fifo_rx_rdata'), length/num_chunks)
            #print('looped read', i, len(rb))
            #read_bytes.extend(rb)
            #while self.flash_is_busy:
            #    print('flash is busy, sleeping')
            #    time.sleep(0.1)

        self.write_fields(reset='fifo_rx_reset')

        return read_bytes

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

    @property
    def flash_is_busy(self):
        self._qspi_send([wb.QSPI_CMD_READ_SR_1])
        rdata = self._qspi_receive()
        return bool(rdata & 0x1)

    def _address_to_bytearray(self, address):
        return bytearray(address.to_bytes(3, byteorder='big'))


def main():
    nxp_enable()

    ib = I2CBus(2, 0x0703)
    ib.connect()

    #ib = I2CBus(2)
    #ib.connect()

    fa = 0x11

    qspi = QspiController(ib, fa, M25FpgaMemoryMap().qspi)

    qspi.connect()
    qspi.setup()

    print('flash busy', qspi.flash_is_busy)

    # Read MFG ID
    mid = qspi.read_mfg_device_id()
    print(f'expected 0xef15, got {mid:07x}')
    print('flash busy', qspi.flash_is_busy)

    jedec = qspi.read_jedec_id()
    print(f'expected 0xef6016, got {jedec:011x}')
    print('flash busy', qspi.flash_is_busy)

    unq = qspi.read_unique_id()
    print(f'expected XX  got {unq:020x}')
    print('flash busy', qspi.flash_is_busy)

    data_addr_base = M25SpiFlashMemoryMap.m25_spi_flash().bitstream_primary.addr_base

    rdata = qspi.fast_read_data(data_addr_base)
    print(len(rdata), rdata)
    print('flash busy', qspi.flash_is_busy)


if __name__ == "__main__":
    main()
