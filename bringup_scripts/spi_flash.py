"""Compare SPI Flash partitions to MCS file
"""
import argparse
import sys

from cobra_system_control.cobra import Cobra
from cobra_system_control.sensor_head import SensorHead


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-c', '--check',
        default=False,
        action='store_true',
        help='Read and verify all partitions.',
    )
    return parser.parse_args(argv)


def main(sen: SensorHead, args):
    sha = sen.debug.read_fields('git_sha')
    print(f'git sha = {sha:08x}', flush=True)

    # Compare
    if args.check:
        print()
        #for partition in [sen.spi_flash.bitstream_jump_table]:
        for partition in sen.spi_flash.partitions:
            mismatch = partition.read_and_verify()
            if len(mismatch) == 0:
                print(f"Partition '{partition.mmp.name}' matched!", flush=True)
            else:
                print(f"Partition '{partition.mmp.name}' DID NOT match!")
                for addr, m in sorted(mismatch.items()):
                    print(f"{addr:#011_x}, {m[0]:#x}, {m[1]:#x}")
            print()


if __name__ == "__main__":
    args_ = parse_args(sys.argv[1:])
    with Cobra.open() as c:
        try:
            main(c.sen, args_)
        finally:
            c.stop()
