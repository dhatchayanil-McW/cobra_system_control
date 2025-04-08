import time
from cobra_system_control.cobra import Cobra
import pprint


def main(c_):
    c_.cmb_laser_vlda_dac.set_voltage(24)
    c_.cmb_laser_vlda_dac.enable()
    c_.cmb_lcm_v_dac.set_voltage(18)
    c_.cmb_lcm_v_dac.enable()
    while True:
        pprint.pprint(c_.cmb_adc.get_mon_all_channels())
        print('temperature', c_.temp_sensor.read_temperature())
        time.sleep(2)


if __name__ == "__main__":
    with Cobra.open() as c_:
        main(c_)
