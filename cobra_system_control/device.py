"""
file: device.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provides a driver for the I2C bus, an I2C Device
that will utilized the bus with various byte packing and
endianness and a Device class to connect
an FPGA YAML memory map to FPGA peripherals.
All FPGA peripherals inherit from the Device class.
"""
import contextlib
import io
import struct
from threading import Lock
from typing import ContextManager

try:
    import fcntl
except (ModuleNotFoundError, OSError):
    pass

import Pyro5.api

from cobra_system_control.cobra_log import log


STRUCT_MAP = {
    1: 'B',
    2: 'H',
    4: 'I',
    8: 'Q',
}


class I2CBus:
    """A class to interface with the CPU I2C bus
    for serial communication.
    """
    def __init__(self, bus: int):
        self.resource = f'/dev/i2c-{bus}'
        self.lock = Lock()
        self.fw = None
        self.fr = None
        self._is_connected = False
        self.ioctl_i2c_slave = 0x0703

    def connect(self):
        if not self._is_connected:
            self.fw = io.open(self.resource, "wb", buffering=0)
            self.fr = io.open(self.resource, "rb", buffering=0)
            self._is_connected = True

    def write(self, device_addr: int, ba: bytearray):
        with self.lock:
            fcntl.ioctl(self.fw, self.ioctl_i2c_slave, device_addr)
            try:
                self.fw.write(ba)
            except OSError:
                return

    def read(self, device_addr: int, ba: bytearray, nbytes: int) -> bytearray:
        with self.lock:
            fcntl.ioctl(self.fw, self.ioctl_i2c_slave, device_addr)
            fcntl.ioctl(self.fr, self.ioctl_i2c_slave, device_addr)
            try:
                self.fw.write(ba)
                ret = self.fr.read(nbytes)
            except OSError as e:
                print('i2c read error', e)
                return
        return ret

    def disconnect(self):
        if self.fw is not None:
            self.fw.close()
        if self.fr is not None:
            self.fr.close()
        self._is_connected = False


class I2CDevice:
    """A class to define a device that uses the I2C Bus with
    a given data format.
    """
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

    def read(self, addr: int) -> int:
        ba = bytearray([*struct.pack(self.addr_pack, addr)])
        ba = self.bus.read(self.device_addr, ba, self.data_nbytes)
        return struct.unpack(self.data_pack, ba)[0]

    def write_bytes(self, addr: int, data: bytearray):
        ba = bytearray([
            *struct.pack(self.addr_pack, addr),
            *data
        ])
        self.bus.write(self.device_addr, ba)

    def read_bytes(self, addr: int, length: int, inc_addr: bool = True) -> int:
        MAX_READ = 256
        read_bytes = bytearray()
        while length > 0:
            ba = bytearray([*struct.pack(self.addr_pack, addr)])
            bytes_to_read = MAX_READ if length > MAX_READ else length
            rb = self.bus.read(self.device_addr, ba, bytes_to_read)
            read_bytes.extend(rb)
            if inc_addr:
                addr += bytes_to_read
            length -= bytes_to_read
        return read_bytes


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class Device:
    """A class to define a Device that communicates over the
    I2C Bus using a defined memory map.
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 addr_bytes: int, data_bytes: int,
                 mmap_periph: 'MemoryMapPeriph',
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

    def setup(self):
        """Hook for subsystem setup

        Implementation should perform any one-time setup procedures, followed
        by a call to ``update`` with default settings."""

    def apply_settings(self, settings=None):
        """Hook for subsystem hardware update

        Implementation should update hardware according to settings object."""

    def enable(self):
        """Hook for actions taken right before starting scan controller"""

    def disable(self):
        """Hook for actions to take right after stopping scan controller
        E.g., to put the LCM in standby mode."""

    def disconnect(self):
        """Hook to disconnect the hardware

        Implementation should disconnect any hardware that was connected in
        ``connect``."""

    def cleanup(self):
        self.disable()
        self.disconnect()

    @classmethod
    @contextlib.contextmanager
    def open(cls, *args, **kwargs) -> ContextManager:
        """Instantiates resource and initializes
        resource communications within a context block.

        Appropriately handles exceptions raised while
        manipulating returned ContextManager object.

        Args:
            *args: passed directly to the underlying system constructor
            *kwargs: passed directly to the underlying system constructor

        Returns:
            (ContextManager[LidarResource]) context manager for system

        Typical use:

        .. code:: python
            from cobra_system_control.cobra import Cobra

            # calls cob.disable() at block end during unhandled exception
            with Cobra.open() as cob:

        """
        system = cls(*args, **kwargs)
        system.connect()  # connects to hardware
        system.setup()  # puts system in a good initial state

        try:
            yield system
        except KeyboardInterrupt:
            log.info('Aborting script early due to keyboard interrupt')
        except Exception as e:
            log.error('An unexpected exception was raised: %s', e)
            raise e
        finally:
            system.cleanup()

    @classmethod
    def __init_subclass__(cls, **kwargs):
        # automatically expose all subclasses of Device to Pyro for development
        cls_ = Pyro5.api.expose(cls)
        cls_ = Pyro5.api.behavior(instance_mode='single')(cls_)
        return cls_


# These are Dummy Classes that should act like
# a device class but shouldn't do anything
# Do not need to call super().__init__()
# Need to accept any number of args for any method call
# pylint: disable=super-init-not-called
# pylint: disable=unused-argument
class DummyObject(Device):
    """A dummy class that should act like
    a device class but shouldn't do anything
    Do not need to call super().__init__()
    Need to accept any number of args for any method call
    """
    def __init__(self, *args, **kwargs):
        pass

    def __getattr__(self, attr):
        def func(*args, **kwargs):
            pass
        return func


class DummyDevice(Device):
    """A dummy class that should act like
    a device class but shouldn't do anything
    Do not need to call super().__init__()
    Need to accept any number of args for any method call
    """
    def __init__(self, *args, **kwargs):
        # add some nested objects that might get called
        self.dac = DummyObject()
        self.bus = DummyObject()
        self.i2c = DummyObject()

    def __getattr__(self, attr):
        def func(*args, **kwargs):
            pass
        return func
# pylint: enable=super-init-not-called
# pylint: enable=unused-argument
