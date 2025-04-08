"""
file: temp_sensor.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provides a driver for the temperature sensor
IC on the Compute Module (NCB)
"""
from cobra_system_control.device import Device


class Tmp1075(Device):
    """A class to define a driver for the
    TI TMP1075 Temperature sensor.
    """
    def __init__(self, bus: 'I2CBus', device_addr: int):
        super().__init__(bus, device_addr, 1, 2, None, True, True)
        self.device_addr = device_addr
        self.temp_data_nbits = 12
        self.temp_data_nfrac = 4

    def get_pointer_register(self, cmd_idx: str):
        command_addr = {
            'read_temp': 0x0,
            'config': 0x1,
            'low_limit': 0x2,
            'hi_limit': 0x3,
            'device_id': 0xf,
        }
        try:
            return command_addr[cmd_idx]
        except KeyError as exc:
            raise KeyError(f"Invalid Tmp1075 register {cmd_idx}") from exc

    def _write(self, register: str, value: int):
        """Frames are 4 bytes
        Frame1 [24:31]
            7: 1
            6: 0
            1-5: device address
            0: 0
        Frame2 [16:23]
            4-7: 0
            0-3: pointer register
        Frame3 [8:15]
            0-7: Data bits 8-15
        Frame4 [0:7]
            0-7: Data bits 0-7
        """
        wrwd = 0
        wrwd |= 1 << 31
        wrwd |= self.device_addr << 25
        wrwd |= self.get_pointer_register(register) << 16
        wrwd |= value
        self.i2c.write(self.device_addr, wrwd)

    def _read(self, register: str):
        """Writes the register pointer then
        reads the data (the bus handles all this)
        """
        return self.i2c.read(self.get_pointer_register(register))

    def setup(self):
        # read the device id
        did = self._read('device_id')
        assert (did >> 8) == 0x75, f'Device id {did} does not match 0x75'
        # set to the slowest conversion rate
        #self._write('config', 0b11 << 13)

    def read_temperature(self):
        rdata = self._read('read_temp')
        return self.convert_int2temp(rdata >> 4)

    def convert_int2temp(self, rdata):
        """Converts the read two's comp integer
        to a temperature value. s7.4 format
        """
        if rdata < 2**(self.temp_data_nbits-1):
            a = rdata
        else:
            a = rdata - 2**self.temp_data_nbits
        return a / 2**self.temp_data_nfrac
