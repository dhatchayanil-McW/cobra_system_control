"""This script helps to profile the latency of the Python call stack when changing settings from the UI.
The typical sequence is stop(), apply_settings(), stop().

Some calls are broken out for additional information such as
- Creating the RandomAccessScanning object that gets fed to apply settings
- Writing the scan table.

"""
import time
from cobra_system_control.cobra import Cobra
from cobra_system_control.random_access_scanning import RandomAccessScanning


def test_times(sen):
    sen.stop()
    n = 10
    start = time.time()
    for _ in range(n):
        ras = RandomAccessScanning(
            angle_range=[[-45,45]],
            fps_multiple=[1],
            power_index=[1],
            inte_time_index=[1],
            max_range_index=[2],
            binning=[2],
            user_tag=[0],
            roi_mapping=sen.roi_mapping,
            roi_rows=8,
            )
        time.sleep(1)
    end = time.time()
    print('Average time for creating RAS object'
          f' is {(end-start-n)/n:.3f}s')

    start = time.time()
    for _ in range(n):
        scan_tab = sen.apply_settings(**ras.appset_dict)
        time.sleep(1)
    end = time.time()
    print('Average time for applying settings'
          f' is {(end-start-n)/n:.3f}s')

    start = time.time()
    for _ in range(n):
        sen.write_scan_table(scan_tab)
        time.sleep(1)
    end = time.time()
    print('Average time for writing scan table'
          f' is {(end-start-n)/n:.3f}s')

    total = 0
    for _ in range(n):
        start = time.time()
        sen.start()
        time.sleep(1)
        end = time.time()
        total += (end-start-1)
        sen.stop()
        time.sleep(1)
    print('Average time for starting lidar'
          f' is {total/n:.3f}s')

    start = time.time()
    for _ in range(n):
        sen.stop()
        time.sleep(2)
    end = time.time()
    print('Average time for stopping lidar'
          f' is {(end-start-2*n)/n:.3f}s')


if __name__ == "__main__":
    with Cobra.remote() as c_:
        test_times(c_.sen)
