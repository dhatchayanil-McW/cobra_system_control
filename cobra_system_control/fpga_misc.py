"""
file: fpga_misc.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provides classes for miscellaneous FPGA peripherals.
"""
from cobra_system_control.device import Device


class FpgaDbg(Device):
    """A class to interface with the FPGA Debug peripheral
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph'):
        super().__init__(bus, device_addr, 2, 1, memmap_periph,
                         addr_bigendian=True, data_bigendian=True)

    def setup(self):
        self.write_fields(dbg_out_en=1)


class FpgaGpio(Device):
    """A class to interface with the FPGA GPIO peripheral
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph'):
        super().__init__(bus, device_addr, 2, 1, memmap_periph,
                         addr_bigendian=True, data_bigendian=True)


class ISP(Device):
    """A class to interface with the FPGA ISP peripheral
    """
    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph'):
        super().__init__(bus, device_addr, 2, 1, memmap_periph,
                         addr_bigendian=True, data_bigendian=True)
