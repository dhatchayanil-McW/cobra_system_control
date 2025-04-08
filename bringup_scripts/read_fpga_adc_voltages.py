"""Script to help compare voltages measured by the FPGA ADC
to those captured with a scope
"""
import time

import numpy as np

from cobra_system_control.cobra import Cobra
from cobra_system_control.fpga_adc import enum_external_monitors_m30


amux_selects = [0, 4, 8, 12, 16, 17, 18, 19,
                20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
                30, 31, 32,]


def main(sen):
    time.sleep(10)
    sen.fpga_adc.calibrate()
    print('**SINGLE**')
    for i in enum_external_monitors_m30():
        print(f'{i:>30}, {sen.read_fpga_adc_voltage(i):>10.3f}')

    sen.fpga_adc.write_fields(amux_sel_ovr_en=1)
    print('/n**AVERAGE**')

    output_str = ""
    output_str += 'measurement, fpga adc measured, scope measured, difference\n'

    for i, j in zip(enum_external_monitors_m30(), amux_selects):
        sen.fpga_adc.write_fields(amux_sel_ovr=j)
        time.sleep(3)
        rdata = []
        for _ in range(10):
            rdata.append(sen.read_fpga_adc_voltage(i))
            time.sleep(.007)
        print(f'{i:>25}, {np.mean(rdata):>10.3f}')
        measured = input(f'Enter scope value for {i} in mV   ')
        measured = float(measured)
        measured /= 1000  # convert to volts
        output_str += f'{i:>25}: {np.mean(rdata):>10.5f}, {measured:>10.5f}, {np.mean(rdata)-measured:>10.5f}\n'
        input(f'Finished with {i}?')
    print(output_str)
    sen.fpga_adc.write_fields(amux_sel_ovr_en=0)


if __name__ == "__main__":
    with Cobra.open(system_type='m30', board_type='nxp') as c_:
        main(c_.sen)
