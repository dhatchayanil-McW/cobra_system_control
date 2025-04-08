import argparse
import sys

import numpy as np

from cobra_system_control.cobra import Cobra
from cobra_system_control.sensor_head import SensorHead
from cobra_system_control.utility import add_host_arg
from cobra_system_control.metadata import FovMetadata, StaticMetadata, print_fov_metadata


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
    parser.add_argument('-p', '--print',
                        action='store_true',
                        help='Print scan table')
    parser.add_argument('-a', '--angle',
                        default=45,
                        type=float,
                        help='FOV half angle')
    parser.add_argument('-r', '--ras',
                        action='store_true',
                        help='Use random access scan')
    parser.add_argument('-i', '--interleave',
                        action='store_true',
                        help="Interleave the scan")
    # parser.add_argument('-q', '--seq',
    #                     action='store_true',
    #                     help="Sequential average the scan")
    return parser.parse_args(argv)


def main(sen: SensorHead, args: argparse.Namespace):
    if args.start:
        binning = [2, 4, 2, 4, 4, 2, 2, 4]
        sen.stop()
        print('interleave =', args.interleave)
        # print('sequential avg =', args.seq)
        if args.ras:
            sen.apply_random_access_scan_settings(
                angle_range=[
                    [-12, 5],
                    [-5, 5],
                    [10, 20],
                    ],
                fps_multiple=[1,3,1],
                frame_average=[1],
                power_index=[1],
                inte_time_index=[1],
                max_range_index=[2],
                bin_x=[2], #binning,
                bin_y=[2], #binning,
                snr_threshold=[3],
                nn_level=[0],
                user_tag=[1], #0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8],
                interleave=args.interleave,
                #sequential_average=args.seq,
                )
        else:
            fovs = [0, 1, 2, 3, 4, 5, 6, 7]
            a = np.arange(-1 * args.angle, args.angle + 1, 1)
            o, r = sen.roi_mapping(angles=a, roi_rows=20, trim_duplicates=True)
            ci_v = 2.2
            ssf = [0] * len(o)
            fovm = FovMetadata.empty_array()
            fbitmask = 0
            for idx, i in enumerate(fovs):
                ssf[0] |= 0b01 << (i*4)
                ssf[-1] |= 0b10 << (i*4)
                fbitmask |= 1 << i
                fovm[i] = FovMetadata(
                    user_tag=44 * (i+1),
                    bin_x=binning[idx],
                    bin_y=binning[idx],
                    s_rows=min(r),
                    n_rows=max(r) - min(r) + 20,
                    n_rois=len(o),
                    rtd_algorithm=0,
                    snr_threshold=500,
                    #frame_average=i + 2,
                    nn_level=i,
                )
            statm = StaticMetadata(
                rtd_output=0,
                reduce_mode=1,
                sensor_sn=sen.serial_number(),
                test_mode=0,
                quant_mode=0,
                )
            print(fbitmask, bin(fbitmask))
            print(statm)
            print_fov_metadata(fovm)


            sen.apply_settings(
                orders=o,
                s_rows=r,
                ci_v=ci_v,
                roi_rows=20,
                loopback=True,
                start_stop_flags=ssf,
                fov_bitmask=fbitmask,
                fov_metadata=fovm,
                static_metadata=statm,

            )
        print(sen.metabuff.static_metadata)
        print_fov_metadata(sen.metabuff.fov_metadata)
        print('Scan table len', len(sen.scan_params.scan_table))

        sen.start()

        if args.print:
            print(f"{sen.debug.read_fields('git_sha'):#010x}")
            print(sen.scan_params.scan_table)
    if args.stop:
        sen.stop()


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    with Cobra.remote(args_.host) as c:
        main(c.sen, args_)
