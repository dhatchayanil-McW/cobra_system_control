import glob
import time
import os

import numpy as np
from cobra_system_control.cobra import Cobra
from cobra_system_control.start_row_calibration import RowCalTable


def test_setup():
    with Cobra.open(system_type="m25", board_type="agx") as c:
        sen = c.sen

        print('writing')
        sen.scan.write_fields(rowcal_adjust_en=1)
        sen.scan.write_fields(rowcal_temp_ovr_en=1)
        sen.scan.write_fields(rowcal_temp_ovr=153)

        print('read back')
        print(sen.scan.read_fields('rowcal_adjust_en'))
        print(sen.scan.read_fields('rowcal_temp_ovr_en'))
        print(sen.scan.read_fields('rowcal_temp_ovr'))

        fixed_thresh = [25, 50, 100, 200]
        # Write a predetermined Rowcal Threshold map
        thresholds = np.zeros((256, 4)).astype(int)
        for j, t in enumerate(thresholds):
            thresholds[j, :] = fixed_thresh
        thresholds[0, :] = [1,2,3,4]
        thresholds[255, :] = [255, 56, 253, 252]
        rt = RowCalTable.build(thresholds)
        sen.row_cal.write_calibration_values(rt)

        for i in range(4):
            print(f'tr_0_{i}', sen.row_cal.read_fields(f'thresh_0_{i}'))
            print(f'tr_255_{i}', sen.row_cal.read_fields(f'thresh_255_{i}'))


def test_realtime():
    with Cobra.open() as c:
        sen = c.sen
        sen.scan.write_fields(rowcal_adjust_en=1)
        print(sen.row_cal.rowcal_table)
        print(sen.cal_data.a2a)
        print(sen.cal_data.cam)

        print('applying_settings')
        sen.apply_settings(
            angles=[-40, -40, -40,],
            ci_v=[0],
        )

        while True:
            os.system('rm -f /run/lumotive/cobra*.bin')
            time.sleep(0.5)
            sen.start()
            #print(sen.scan_params.scan_table)
            time.sleep(2)

            fids = sorted(glob.glob('/run/lumotive/cobra*.bin'))
            # Use the latest file
            fimg = np.fromfile(fids[-1], dtype=np.uint16)
            fimg &= 0xfffc
            fmeta = list(fimg[0:640*3].copy() >> 4)
            print(f"Order : {sen.scan_params.scan_table[0].steering_idx: >4.0f} "
                  f"Set Srow : {sen.scan_params.scan_table[0].rwin0_s-4: >4.0f} "
                  f"Read Srow : {fmeta[1]: > 4.0f} "
                  f"Temp : {sen.fpga_adc.get_mon_laser_temp(): >8.3f} C ")
            sen.stop()


if __name__ == "__main__":
    test_realtime()
