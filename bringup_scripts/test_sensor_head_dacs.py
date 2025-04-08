import argparse
import sys
import time

import numpy as np

from cobra_system_control.cobra import Cobra


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vlda',
                        action='store_true',
                        help='choose the sh vlda dac')
    parser.add_argument('--ci',
                        action='store_true',
                        help='choose the sh ci dac')
    parser.add_argument('--field',
                        action='store_true',
                        help='Write fields. Otherwise inputs will be to write voltages')
    parser.add_argument('--loop', action="store_true",
                        )
    return parser.parse_args(argv)


def main(sh, args):
    sh.enable()

    if args.vlda:
        sh.cmb_laser_vlda_dac.enable()
        sh.cmb_laser_vlda_dac.set_voltage(30)
        vdac = sh.sh_laser_vlda_dac
        dac_str = 'vlda'
    elif args.ci:
        vdac = sh.laser_ci_dac
        dac_str = 'ci'
    time.sleep(10)
    if args.loop and args.ci:
        print('Cannot loop with the ci selection')
        return
    if args.loop:
        for v in reversed(np.arange(0, 28, 2)):
            sh.set_laser_vlda_combined(v)
            time.sleep(5)
            rcomb = sh.get_laser_vlda_combined()
            rset = sh.sh_laser_vlda_dac.get_voltage()
            radc = sh.fpga_adc.get_mon_vlda()
            rf = sh.sh_laser_vlda_dac.get_field()
            print(f'set {v}, get combined {rcomb:.2f}, get voltage {rset: .2f}, adc {radc:.2f}, field {rf}')
    else:
        while True:
            if args.field:
                resp = input('Press s for setup or provide field integer. "x" to exit  ')
            else:
                resp = input('Press s for setup or provide voltage float  "x" to exit  ')

            if resp == 's':
                print('Setting up')
                vdac.setup()
                vdac.enable()
            elif resp == 'x':
                print('Exiting')
                break
            else:
                if args.field:
                    try:
                        val = int(resp)
                        print(f'Setting DAC chan {vdac.chan_idx} value to {int(resp)}')
                        print('Expecting voltage', vdac.voltage_from_field(val))
                        vdac.dac.dac_write(None, vdac.chan_idx, int(resp))
                        if args.vlda:
                            print('Expecting combined VLDA voltage', sh.get_laser_vlda_combined())
                        else:
                            print('Expecting CI voltage', sh.laser_ci_dac.get_voltage())
                        time.sleep(0.5)
                        print('VLDA ADC', sh.fpga_adc.get_mon_vlda())
                    except ValueError:
                        print('Bad value input')
                        continue
                else:
                    try:
                        val = float(resp)
                        print(f'Setting {dac_str} DAC chan {vdac.chan_idx} value to voltage {float(resp)}')
                        if args.vlda:
                            sh.set_laser_vlda_combined(val)
                            print('Expecting combined voltage', sh.get_laser_vlda_combined())
                        else:
                            sh.laser_ci_dac.set_voltage(val)


                        time.sleep(0.5)
                        if args.vlda:
                            print('VLDA ADC', sh.fpga_adc.get_mon_vlda())
                    except ValueError:
                        print('Bad value input')
                        continue


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    with Cobra.open(system_type="m30", board_type="nxp") as c:
        main(c.sen, args_)
