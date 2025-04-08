import numpy as np

from cobra_system_control.cobra import Cobra
from cobra_system_control.sensor_head import SensorHead


def main(sh: SensorHead):
    cal_data = sh.get_cal_data()

    cal_data.a2a.update_group(
        vfxp=dict(wave_nm=905.0,
                  ps_c_0=48.44,
                  ps_c_1=-0.23821,
                  ps_c_2=0.0002052,
                  ps_c_3=-0.000000285
        )
    )

    cal_data.info.update_group(
        vfxp=dict(sensor_sn=999)
    )

    cal_data.cam.update_group(
        vfxp=dict(fx=325.00000000,
                  fy=325.00000000,
                  cx=319.5,
                  cy=239.5,
                  k1=-0.20193899, k2=0.09003341,
                  p1=-0.10790229, p2=0.05483031, k3=0,
        )
    )

    cal_data.dyn.update_group(
        vfxp=dict(pga_gain=10,
                  doff_diff_adu=0)
    )

    cal_data.range0807.update_group(
        vfxp=dict(pw_laser_f0_shrink   = 0.00000000,
                  pw_laser_f0_expand   = 1.00000000,
                  dlay_laser_f0_coarse = 0.00000000,
                  dlay_laser_f0_fine   = 0.00000000,
                  dlay_mg_f0_coarse    = 4.00000000,
                  dlay_mg_f0_fine      = 1.00000000,
                  pw_laser_f1_shrink   = 0.00000000,
                  pw_laser_f1_expand   = 1.00000000,
                  dlay_laser_f1_coarse = 0.00000000,
                  dlay_laser_f1_fine   = 0.00000000,
                  dlay_mg_f1_coarse    = 4.00000000,
                  dlay_mg_f1_fine      = 4.00000000,
                  sync_laser_lvds_mg   = 1.00000000)
    )

    cal_data.range0908.update_group(
        vfxp=dict(pw_laser_f0_shrink   = 0.00000000,
                  pw_laser_f0_expand   = 1.00000000,
                  dlay_laser_f0_coarse = 0.00000000,
                  dlay_laser_f0_fine   = 0.00000000,
                  dlay_mg_f0_coarse    = 4.00000000,
                  dlay_mg_f0_fine      = 1.00000000,
                  pw_laser_f1_shrink   = 0.00000000,
                  pw_laser_f1_expand   = 1.00000000,
                  dlay_laser_f1_coarse = 0.00000000,
                  dlay_laser_f1_fine   = 0.00000000,
                  dlay_mg_f1_coarse    = 4.00000000,
                  dlay_mg_f1_fine      = 4.00000000,
                  sync_laser_lvds_mg   = 1.00000000)
    )

    cal_data.range_tmp.update_group(
        vfxp=dict(rng_offset_mm_0807=-5.67,
                  mm_per_volt_0807=1.66,
                  mm_per_celsius_0807=3.91,
                  rng_offset_mm_0908=15.2,
                  mm_per_volt_0908=4.35,
                  mm_per_celsius_0908=0.67,)
    )

    cal_data.lcm.update_group(
        vfxp=dict(lcm_sn=999,
                  lc_settle_us=100,
                  lc_settle_temp=30,
                  thermistor_res=5000,
                  thermistor_temp=30,
                  lcm_layout=2,
                  lcm_driver=1,
                  lcm_package=1,
                  lcm_interface=1,
                  lcm_eta=35,)
    )

    sh.set_cal_data(cal_data)


if __name__ == "__main__":
    with Cobra.remote() as c:
        main(c.sen)
