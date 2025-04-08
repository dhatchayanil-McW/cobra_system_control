import argparse
import sys
from cobra_system_control.cobra import *
import time


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-n', '--num',
                        type=int,
                        default=1,
                        help='Number of times to cycle')
    return parser.parse_args(argv)


def main(args):
    for _ in range(args.num):
        with Cobra.open() as c:
            #print('opened cobra and connected')
            sen = c.sen
            gs = sen.debug.read_fields('git_sha')
            sid = sen.isp.read_fields('sensor_id')
            print(f'git sha = {gs:#010x}, sensor_id = {sid:#08x}')
            time.sleep(1)


if __name__ == "__main__":
    _args = parse_args(sys.argv[1:])
    main(_args)
