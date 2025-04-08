import argparse
import sys
import time

import numpy as np

from cobra_system_control.device import I2CBus
from cobra_system_control.dacs import TiDac6578, SensorVDac, LcmVDac
from cobra_system_control.laser import LaserVldaDac
from cobra_system_control.compute import CpuGpio


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--sen',
                        action='store_true',
                        help='Power on the sensor v dac')
    parser.add_argument('--lcm',
                        action='store_true',
                        help='Power on the lcm v dac')
    parser.add_argument('--vlda',
                        action='store_true',
                        help='Power on the vlda v dac')
    return parser.parse_args(argv)


def main(args):
    cmb_i2c_bus = I2CBus(4)

    #dac_addr = 0x48
    #dac = TiDac6578(i2cbus, dac_addr, None, 5, 1)
    #dac.connect()
    #dac.setup()

    if args.sen:
        gpio = CpuGpio(4, 19)
        dac = SensorVDac(TiDac6578(cmb_i2c_bus, 0x48, None, 3.3, 1), 4, 'nxp', CpuGpio(4, 19))
        vrange = np.linspace(2, 4, 5)
    elif args.vlda:
        gpio = CpuGpio(4, 17)
        dac = LaserVldaDac(TiDac6578(cmb_i2c_bus, 0x48, None, 3.3, 1), 0, 'nxp', CpuGpio(4, 17))
        vrange = np.arange(10, 25)
    elif args.lcm:
        gpio = CpuGpio(4, 18)
        dac = LcmVDac(TiDac6578(cmb_i2c_bus, 0x48, None, 3.3, 1), 2, 'nxp', CpuGpio(4, 18))
        vrange = np.arange(8, 22)

    #gpio.connect()
    #gpio.setup()
    #gpio.enable()
    cmb_i2c_bus.connect()
    dac.connect()
    dac.setup()
    dac.enable()

    for v in vrange:
        print(f'Setting dac to voltage {v}')
        dac.set_voltage(v)
        time.sleep(2)
        print(f' Read voltage {dac.get_voltage()}')
        # field = int(v * 2**dac.dac_bits / dac.dac_full_scale)
        # print(f'Writing voltage {v}, field {field} to channel {dac_chan}')
        # dac.dac_write('write_update', dac_chan, field)
        # print('read', dac.dac_read(dac_chan))
    print('Disabling with GPIO')
    dac.disable()
    print('Disabled')


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    main(args_)
