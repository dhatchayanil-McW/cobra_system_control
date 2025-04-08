import argparse
import pickle
import sys
import time

from cobra_system_control.cobra import Cobra


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--recalibrate',
                        action='store_true',
                        help='Recalibrate the ADC')
    parser.add_argument('-a', '--append',
                        type=str,
                        help='append a str')
    return parser.parse_args(argv)


def main(sh, args):
    vref_hi_cnt = []
    vref_lo_cnt = []
    vref_hi_volt = []
    vref_lo_volt = []
    fpga_die_temp = []
    laser_temp = []
    cal_gain = []
    cal_offset = []
    counter = 0

    while True:
        try:
            for _ in range(10):
                time.sleep(1)

            if counter == 10:
                sh.start()

            if args.recalibrate:
                sh.fpga_adc.calibrate()

            cal_gain.append(sh.fpga_adc._cal_gain)
            cal_offset.append(sh.fpga_adc._cal_offset)
            vref_hi_cnt.append(sh.fpga_adc.read_fields('mon_vref_hi'))
            vref_hi_volt.append(sh.fpga_adc.get_mon_vref_hi())
            vref_lo_cnt.append(sh.fpga_adc.read_fields('mon_vref_lo'))
            vref_lo_volt.append(sh.fpga_adc.get_mon_vref_lo())
            fpga_die_temp.append(sh.fpga_adc.get_mon_fpga_temp())
            laser_temp.append(sh.fpga_adc.get_mon_laser_temp())

            counter += 1

        except KeyboardInterrupt:
            with open(f'cal_gain_recal_{args.recalibrate}_{args.append}.pkl', 'wb') as f:
                pickle.dump(cal_gain, f)
            with open(f'cal_offset_recal_{args.recalibrate}_{args.append}.pkl', 'wb') as f:
                pickle.dump(cal_offset, f)
            with open(f'vref_hi_cnt_recal_{args.recalibrate}_{args.append}.pkl', 'wb') as f:
                pickle.dump(vref_hi_cnt, f)
            with open(f'vref_hi_volt_recal_{args.recalibrate}_{args.append}.pkl', 'wb') as f:
                pickle.dump(vref_hi_volt, f)
            with open(f'vref_lo_cnt_recal_{args.recalibrate}_{args.append}.pkl', 'wb') as f:
                pickle.dump(vref_lo_cnt, f)
            with open(f'vref_lo_volt_recal_{args.recalibrate}_{args.append}.pkl', 'wb') as f:
                pickle.dump(vref_lo_volt, f)
            with open(f'fpga_die_temp_recal_{args.recalibrate}_{args.append}.pkl', 'wb') as f:
                pickle.dump(fpga_die_temp, f)
            with open(f'laser_temp_recal_{args.recalibrate}_{args.append}.pkl', 'wb') as f:
                pickle.dump(laser_temp, f)
            sh.stop()
            return


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    with Cobra.open() as c_:
        main(c_.sen, args_)
