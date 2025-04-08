import argparse
from pathlib import Path
import sys

import numpy as np


pixels = {
    0: 'p_raw_sensor_mode',
    1: 'p_meta_start_row',
    2: 'p_meta_num_rows',
    3: 'p_meta_mod_freq0',
    4: 'p_meta_mod_freq1',
    5: 'p_raw_npulse_group_f0',
    6: 'p_raw_npulse_group_f1',
    7: 'p_raw_inte_burst_length_f0',
    8: 'p_raw_inte_burst_length_f1',
    9: 'p_raw_roi_id',
    10: 'p_raw_blob1',
    11: 'p_raw_blob2',
    12: 'p_raw_blob3',
    13: 'p_raw_blob4',
    14: 'fov_bitmask',
    15: 'start_stop_0',
    16: 'start_stop_1',
    17: 'start_stop_2',
    18: 'start_stop_3',
    19: 'start_stop_4',
    20: 'start_stop_5',
    21: 'start_stop_6',
    22: 'start_stop_7',
    23: 'SCAN_ROI_CNT',
    24: 'TS_STAMP0',
    25: 'TS_STAMP1',
    26: 'TS_STAMP2',
    27: 'TS_STAMP3',
    28: 'TS_STAMP4',
    29: 'TS_STAMP5',
    30: 'TS_STAMP6',
    31: 'ADC_MEAS_CURRS_LCM',
    32: 'ADC_MEAS_CURRS_LASER',
    33: 'ADC_MEAS_ADC_18_0',
    34: 'ADC_MEAS_ADC_VLDA',
    35: 'ADC_MEAS_LASER_THERM',
    36: 'ADC_MEAS_ADC_1_2',
    37: 'ADC_MEAS_ADC_VRAMP',
    38: 'ADC_MEAS_ADC_VREF_2_5',
    39: 'd0_0',

    48: 'rtd_output',
    49: 'reduce_mode',
    50: 'sensor_sn',
    51: 'test_mode',
    52: 'quant_mode',
    53: 'mipi_raw_mode',
    54: 'hdr_threshold',

}


fov = {64 + i + (j * 10): f'{d}_{j%9}' for i, d in enumerate([
    'user_tag', 'bin_x', 'bin_y', 's_rows',
    'n_rows', 'n_rois',
    'rtd_algorithm',
    'snr_threshold', 'frame_average, nn_level',
    ]) for j in range(8)
       }

pixels.update(fov)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_file",
                        type=str,
                        help="Specify path to a ROI binary")
    return parser.parse_args(argv)


def get_metadata(f):
    data = np.fromfile(f, dtype=np.uint16)
    return data[0:1920] >> 4


def main(args):
    fid = args.input_file   # '/home/rossuthoff/swdl_test_0_01_0000.bin'
    data = get_metadata(fid)

    for p in range(144):
        name = pixels.get(p)
        if name is None:
            continue
        print(f'pixel{p:>5} : {name:>30s}  :  {data[p]}')


def arrayed_main():
    fs = [
        Path(Path.home(), 'hdr1p3_test', 'hdr_crash_0_01_0000.bin'),
        Path(Path.home(), 'hdr1p3_test', 'hdr_crash_0_01_0001.bin'),
        # Path(Path.home(), 'hdr1p3_test', 'hdr_test_0_01_0002.bin'),
        # Path(Path.home(), 'hdr1p3_test', 'hdr_test_0_01_0003.bin'),
        # Path(Path.home(), 'hdr1p3_test', 'hdr_test_0_01_0004.bin'),
        # Path(Path.home(), 'hdr1p3_test', 'hdr_test_0_01_0005.bin'),
        # Path(Path.home(), 'hdr1p3_test', 'hdr_test_0_01_0006.bin'),
        ]
    fs_metadata = []
    for f in fs:
        d = get_metadata(f)
        fs_metadata.append(d)

    print([str(f)[27:51] for f in fs])

    for p in range(144):
        name = pixels.get(p)
        if name is None:
            continue
        print(f'pixel{p:>3} : {name:>28s}  : ',end='')
        for d in fs_metadata:
            print(f'{d[p]:>6}   | ', end='')
        print('')


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    #main(args_)
    arrayed_main()
