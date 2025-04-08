import argparse
import sys

from cobra_system_control.cobra import Cobra


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-i', '--sel',
                        type=int,
                        default=6,
                        help='Set debug sel')
    return parser.parse_args(argv)


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    with Cobra.remote() as c:
        c.sen.debug.write_fields(dbg_sel=args_.sel)
