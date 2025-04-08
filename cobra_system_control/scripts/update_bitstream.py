import argparse
from pathlib import Path
import sys

from cobra_system_control.mcs_updater import update_fpga, update_lcm
from cobra_system_control.cobra import Cobra


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-f', '--dofpga',
                        action='store_true',
                        help='Overwrite the fpga bitstream')
    parser.add_argument('-l', '--dolcm',
                        action='store_true',
                        help='Overwrite the lcm tables')
    return parser.parse_args(argv)


def main(args: argparse.Namespace):
    with Cobra.open(system_type="m30", board_type="nxp") as c:
        sf = c.sen.spi_flash
        if args.dofpga:
            update_fpga(sf, str(c.fpga_bin_path))
        if args.dolcm:
            update_lcm(sf, str(c.sen.lcm_assembly_lcm_bin_path))
        else:
            pass


if __name__ == "__main__":
    _args = parse_args(sys.argv[1:])
    main(_args)
