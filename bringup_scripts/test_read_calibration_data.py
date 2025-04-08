import argparse
import random
import sys

from cobra_system_control.cobra import Cobra


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',
                        type=str,
                        default=None,
                        help='Hostname for remote object location')
    return parser.parse_args(argv)


def main(args: argparse.Namespace, c: Cobra):
    cal_data = c.sen.get_cal_data()
    print(cal_data)



if __name__ == "__main__":
    _args = parse_args(sys.argv[1:])
    with Cobra.remote(hostname=_args.host) as c_:
        main(_args, c_)
