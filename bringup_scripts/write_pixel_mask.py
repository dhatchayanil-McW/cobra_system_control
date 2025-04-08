import argparse
import sys
from cobra_system_control.cobra import *
import numpy as np

from cobra_system_control.pixel_mask import (
    encode_mask, decode_mask, decode_diff,
    encode_diff)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-n', '--num',
                        type=int,
                        default=1,
                        help='Number of times to cycle')
    return parser.parse_args(argv)


def circle_mask():
    # make a circle
    img = np.zeros((480, 640), dtype=np.uint8)
    for r in range(img.shape[0]):
        for c in range(img.shape[1]):
            if int(np.sqrt((r - 240) ** 2 + (c - 320) ** 2)) < 200:
                img[r, c] = 1

    # the underlying open cv algorithm ends up determining almost all of these
    # i don't really care what they are, as long as they are in the
    # expected range and give me the exact same mask after decode

    s_row, s_col, n_elements, ba, img_shape = encode_mask(img)
    return int(s_row), int(s_col), int(n_elements), ba, img_shape


def main(args):
    srow, scol, nelements, ba, shape = circle_mask()
    print(srow, scol, nelements, ba, shape)

    with Cobra.open() as c:
        #print('opened cobra and connected')
        sen = c.sen
        gs = sen.debug.read_fields('git_sha')
        print(f'git sha = {gs:#010x}')

        pcd = sen.get_pixel_mask_cal()
        print(pcd)
        print(pcd.pixel_mask)
        pcd.pixel_mask.update_group(
            vfxp=dict(start_row=srow,
                      start_col=scol,
                      n_elements=nelements),
            vbytes=dict(array=ba),
        )
        sen.set_pixel_mask_cal(pcd)


if __name__ == "__main__":
    _args = parse_args(sys.argv[1:])
    main(_args)
