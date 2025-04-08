"""
file: spi_flash_dump.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file is a helper script to dump all SPI flash contents to file.

Note the hardcoded memory locations in the script may need to change
if the spi flash memory is reallocated.
"""

import sys
import time
import argparse

from cobra_system_control.cobra import Cobra
from cobra_system_control.cobra_log import log
import cobra_system_control.w25q32jw_const as wb

FLASH_ADDR_BEGIN = 0x00_0000
FLASH_ADDR_END   = 0x40_0000

# auto detect if int or hex
def auto_int(x):
    return int(x, 0)

def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-s', '--system',
                        type=int,
                        help='M series system number, <25,30>')
    parser.add_argument('-c', '--compute',
                        type=str,
                        default='nxp',
                        help='compute type <nxp>')
    parser.add_argument('-b', '--addr_begin',
                        type=auto_int,
                        default=FLASH_ADDR_BEGIN,
                        help=f'SPI flash address begin <0x{FLASH_ADDR_BEGIN:06x}>')
    parser.add_argument('-e', '--addr_end',
                        type=auto_int,
                        default=FLASH_ADDR_END,
                        help=f'SPI flash address end <0x{FLASH_ADDR_END:06x}>')
    parser.add_argument('-o', '--output',
                        type=str,
                        default='spi_dump.bin',
                        help='output filename')
    return parser.parse_args(argv)

def main(args):
    try:
        c = Cobra(f'm{args.system}', args.compute)
        sf = c.sen.spi_flash

        # Init and setup communications
        c.cam_i2c_bus.connect()
        c.cmb_i2c_bus.connect()

        c.compute.setup()
        c.compute.select_i2c_gpio.enable()

        c.cmb_sensor_v_dac.connect()
        c.cmb_sensor_v_dac.setup()
        c.cmb_sensor_v_dac.set_voltage(0)
        c.cmb_sensor_v_dac.enable()
        time.sleep(0.5)
        c.cmb_sensor_v_dac.set_voltage(3.4)
        time.sleep(0.1)
        c.sen.connect()
        c.sen.debug.setup()

        c.compute.clk_sync_gpio.sync()

        # Read from SPI flash
        max_chunk = args.addr_end // wb.W25Q32JW_FAST_READ_SIZE
        chunks = range(max_chunk)

        with open(args.output, "wb") as f:
            for idx, ck in enumerate(chunks):
                addr = args.addr_begin + (ck * wb.W25Q32JW_FAST_READ_SIZE)
                log.info(f'Download progress: {int(idx / len(chunks) * 100):.0f}%')
                f.write(sf.qspi.fast_read_data(addr, length=wb.W25Q32JW_FAST_READ_SIZE))

    except KeyboardInterrupt:
        log.debug('Aborting script early due to keyboard interrupt')
    except Exception as e:
        log.error('An unexpected exception was raised: %s', e)
        raise e
    finally:
        c.disable()
        c.disconnect()
        c.cam_i2c_bus.disconnect()  # needed to avoid segv

if __name__ == "__main__":
    _args = parse_args(sys.argv[1:])
    main(_args)
