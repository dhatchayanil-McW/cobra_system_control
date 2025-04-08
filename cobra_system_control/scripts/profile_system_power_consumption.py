"""
file: profile_system_power_consumption.py

Copyright (C) 2024 Lumotive, Inc. All rights reserved.

This file helps to profile the power consumption of both
the Sensor Head and the Compute Module using the
ADC measurements on the Compute Module.
NUM_MEASURES are collected at 1second intervals and averaged.
"""
import argparse
import glob
import time
import sys

import numpy as np
import pandas as pd

from cobra_system_control.cobra import Cobra
from cobra_system_control.fpga_adc import get_mon_all

NUM_MEASURES = 500


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-c', '--collect', action='store_true',
        help='Collect power consumption data')
    parser.add_argument(
        '-a', '--analyze', action='store_true',
        help='Analyze a group of collected power data. Pickles must be in same folder as this script.')
    return parser.parse_args(argv)


def profile_power(c: Cobra):
    """Loops through the NCB ADC measurements and
    appends to a list and converts to pandas dataframe.
    Iterates and gets the mean of each ADC net for printing.
    """
    adc_measures = []

    for _ in range(NUM_MEASURES):
        rval = {}
        rval.update(c.cmb_adc.get_mon_all_channels())
        rval.update(get_mon_all(c.sen.fpga_adc))
        adc_measures.append(rval)
        time.sleep(.15)

    df = pd.DataFrame(adc_measures)

    # Drop some columns that are less interesting
    df = df.drop(columns=[
        'amb_det_1', 'amb_det_2', 'v1p2', 'vmgh', 'vcc', 'vccio0', 'vccaux',
        'vref_lo', 'vref_hi', 'v2p8', 'v9p0', 'laser_temp', 'die_temp', 'pcb_temp',
        'lcm_temp', 'lcm_current_fine',
        # These are measured on the NCB already with currents
        'v21p0', 'v24p0',
    ])
    df = df.reindex(sorted(df.columns), axis=1)

    if df['v3p3_volts_cb'].mean() > 5:
        df['v3p3_volts_cb'] = df['v3p3_volts_cb'].div(2)
        df['v3p3_power_cb'] = df['v3p3_power_cb'].div(2)
    df['sensor_head_power'] = df.v3p3_power_cb + df.v24p0_power_cb + df.v21p0_power_cb
    df['ncb_power'] = df.vin_power_cb - df.sensor_head_power

    print(df.describe(percentiles=[]).transpose())
    df.to_pickle(f'./{c.sen.sensor_id}_scanning_power.pkl')


def collect_scanning_power_measurement(c: Cobra):
    """Collects power consumption values during SCANNING
    state and saves to pickle.
    """
    print(f"SN {c.sen.sensor_id}")
    print("\n Profiling SCANNING power")
    c.sen.start()
    time.sleep(20)
    profile_power(c)
    c.sen.stop()


def analyze_scanning_power_measurement():
    """Loads saved pickles and calculates statistics
    across all sensors measured.
    """
    fids = glob.glob('./*.pkl')
    print(fids)
    v3p3_power_cb = []
    v21p0_power_cb = []
    v24p0_power_cb = []
    sensor_head_power = []
    ncb_power = []

    for fid in fids:
        df = pd.read_pickle(fid)
        v3p3_power_cb.append(df.v3p3_power_cb.mean())
        v21p0_power_cb.append(df.v21p0_power_cb.mean())
        v24p0_power_cb.append(df.v24p0_power_cb.mean())
        sensor_head_power.append(df.sensor_head_power.mean())
        ncb_power.append(df.ncb_power.mean())

    print('Measurement', 'Mean', 'STD')
    print('v3p3_power_cb', np.mean(v3p3_power_cb), np.std(v3p3_power_cb))
    print('v21p0_power_cb', np.mean(v21p0_power_cb), np.std(v21p0_power_cb))
    print('v24p0_power_cb', np.mean(v24p0_power_cb), np.std(v24p0_power_cb))
    print('sensor_head_power', np.mean(sensor_head_power), np.std(sensor_head_power))
    print('ncb_power', np.mean(ncb_power), np.std(ncb_power))


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    if args_.collect:
        with Cobra.remote() as c_:
            collect_scanning_power_measurement(c_)
    elif args_.analyze:
        analyze_scanning_power_measurement()
