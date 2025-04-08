"""
file: start_lidar.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

Helper script to start the lidar to collect depth measurements.
Provides command-line arguments for many of the args in the
apply_settings() api. The sensor can also be started using the
apply_random_access_scan_settings() api using the --ras argument.
"""
import argparse
import sys

import numpy as np

from cobra_system_control.cobra import Cobra
from cobra_system_control.functional_utilities import add_host_arg
from cobra_system_control.metadata import print_virtual_sensor_metadata
from cobra_system_control.random_access_scanning import InteTimeIdxMappedOv
from cobra_system_control.sensor_head import (
    SensorHead, DEFAULT_RTD_ALGORITHM_COMMON, DEFAULT_RTD_ALGORITHM_GRID_MODE,
    DEFAULT_RTD_ALGORITHM_STRIPE_MODE, DEFAULT_DSP_MODE, DEFAULT_FRAME_RATE,
    DEFAULT_INTE_TIME_US, DEFAULT_ROI_ROWS,
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    add_host_arg(parser)
    start_stop_group = parser.add_mutually_exclusive_group(required=True)
    start_stop_group.add_argument('-s', '--start',
                                  action='store_true',
                                  help='starts lidar')
    start_stop_group.add_argument('-x', '--stop',
                                  action='store_true',
                                  help='stops lidar')

    parser.add_argument('-a', '--angle', type=float, default=45.0,
                        help='specify the half angle for VIRTUAL_SENSOR')
    parser.add_argument('--step', type=float, default=1.0,
                        help='angular step')
    parser.add_argument('-o', '--single', action='store_true',
                        help='Treat the angle as a single angle instead of a range')
    parser.add_argument('-b', '--binning', type=int, default=2,
                        help='Specify the binning mode')
    parser.add_argument('-c', '--ci', type=float, default=None,
                        help='set laser ci')
    parser.add_argument('-n', '--nn', default=0, type=int,
                        help='specifies amount of NN thresholding (0-7)')
    parser.add_argument('-p', '--print', action='store_true',
                        help='Print scan table')
    parser.add_argument('--snr', default=1.0, type=float,
                        help='specify the snr threshold')
    parser.add_argument('--rtdc', type=int, default=DEFAULT_RTD_ALGORITHM_COMMON, help='set rtd_algo')
    parser.add_argument('--rtdg', type=int, default=DEFAULT_RTD_ALGORITHM_GRID_MODE, help='Set Grid Mode algorithms')
    parser.add_argument('--rtds', type=int, default=DEFAULT_RTD_ALGORITHM_STRIPE_MODE, help='Set Stripe Mode algorithms')
    parser.add_argument('--nomask', action='store_true',
                        help='Tell R2D to do no range masking')
    parser.add_argument('--pol0', type=int, default=None, help='set pol_cnt_tc0')
    parser.add_argument('--pol1', type=int, default=None, help='set pol_cnt_tc1')
    parser.add_argument('--trig', type=int, default=None, help='set scan trig')
    parser.add_argument('--fetch', type=int, default=None, help='set scan fetch')
    parser.add_argument('--tp0', type=int, default=None, help='set tp1 period 0')
    parser.add_argument('--tp1', type=int, default=None, help='set tp1 period 1')
    parser.add_argument('--inte', type=int, default=DEFAULT_INTE_TIME_US, help='set inte time us')
    parser.add_argument('--rows', type=int, default=DEFAULT_ROI_ROWS, help='set the number of rows')
    parser.add_argument('--ras', action='store_true', help='Use RandomAccessScanning entry point')
    parser.add_argument('--nortd', action='store_true', help='Turn off R2D when saving frames')
    parser.add_argument('--framerate', type=int, default=DEFAULT_FRAME_RATE, help='set frame rate in Hz for a single ROI')
    parser.add_argument('--dsp', type=int, default=DEFAULT_DSP_MODE, help='Set the dsp_mode; 0: Camera Mode, 1: Lidar Mode')

    parser.add_argument('--hdr', type=int, default=4095, help='Set HDR threshold')
    parser.add_argument('--hdrinte', type=int, default=5, help='Set the HDR integration time in us')
    parser.add_argument('--hdrci', type=float, default=1.5, help='Set the HDR laser ci voltage')

    parser.add_argument('--laserpower', type=int, default=100, help='Set RAS laser power percent')
    parser.add_argument('--hdrlaser', type=int, default=25, help='Set RAS HDR laser power percent')
    return parser.parse_args(argv)


def main(sen: SensorHead, args: argparse.Namespace):
    if args.start:
        if args.single:
            a = [args.angle]
            print(f'Setting to angle {args.angle}')
        else:
            a = np.arange(-1 * args.angle, args.angle + 1, args.step)
            print(f'Setting steering to range({-1 * args.angle}, {args.angle+1}, {args.step})')
        sen.stop()

        if args.ras:
            sen.apply_random_access_scan_settings(
                angle_range=[[-1 * args.angle, args.angle, args.step]],
                fps_multiple=1,
                power_percent=args.laserpower,
                inte_time_us=args.inte,
                binning=args.binning,
                snr_threshold=args.snr,
                nn_level=args.nn,
                dsp_mode=args.dsp,
                rtd_algorithm_common=args.rtdc | int(args.nomask << 1),
                rtd_algorithm_grid_mode=args.rtdg,
                rtd_algorithm_stripe_mode=args.rtds,
                hdr_threshold=args.hdr,
                hdr_laser_power_percent=args.laserpower,
                hdr_inte_time_us=args.hdrinte,
                )

        else:
            sen.apply_settings(
                angles=a,
                roi_rows=args.rows,
                inte_time_s=InteTimeIdxMappedOv.MAP[InteTimeIdxMappedOv.OPTIONS.index(args.inte)],
                ci_v=args.ci,
                snr_threshold=args.snr,
                nn_level=args.nn,
                binning=args.binning,
                rtd_algorithm_common=args.rtdc | int(args.nomask << 1),
                rtd_algorithm_grid_mode=args.rtdg,
                rtd_algorithm_stripe_mode=args.rtds,
                frame_rate_hz=args.framerate,
                tp1_period_us=(args.tp0, args.tp1),
                pol_cnt=(args.pol0, args.pol1),
                scan_trigger_delay=args.trig,
                scan_fetch_delay=args.fetch,
                disable_rawtodepth=args.nortd,
                dsp_mode=args.dsp,
                hdr_threshold=args.hdr,
                hdr_ci_v=args.hdrci,
                hdr_inte_time_s=InteTimeIdxMappedOv.MAP[InteTimeIdxMappedOv.OPTIONS.index(args.hdrinte)],
            )

        sen.start()

        if args.print:
            print(sen.metabuff.static_metadata)
            print_virtual_sensor_metadata(sen.metabuff.virtual_sensor_metadata)
            print(f"{sen.debug.read_fields('git_sha'):#010x}")
            print(sen.scan_params.scan_table)
            srows = []
            for i in sen.scan_params.scan_table:
                srows.append(i.rwin0_s)
    if args.stop:
        sen.stop()


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    with Cobra.remote(args_.host) as c:
        main(c.sen, args_)
