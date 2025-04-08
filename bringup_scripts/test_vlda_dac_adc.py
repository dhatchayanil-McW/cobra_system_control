import time
from cobra_system_control.cobra import Cobra


if __name__ == "__main__":

    with Cobra.remote() as c:
        vldas = [15,16,17,18,19,20]
        for i in vldas:

            c.sen.cmb_laser_vlda_dac.set_voltage(i)
            time.sleep(2)
            print(f'cmb_laser_vlda_dac.get_voltage() = {c.sen.cmb_laser_vlda_dac.get_voltage()} V')
            time.sleep(2)
            print(f'fpga_adc.get_mon_vlda() = {c.sen.fpga_adc.get_mon_vlda()} V')
            time.sleep(2)
