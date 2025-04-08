"""
file: pixel_mapping.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

Functions related to pixel mapping of RX, converting
pixel location to far-field angle.

20220325 Pixel Mapping Table coordinate convention changed
to ISO 80000-2:2019. Details can be seen at
 https://en.wikipedia.org/wiki/Spherical_coordinate_system
(Physics convention).

The mapping table is super-sampled to be able to handle both the
native VGA resolution and the various levels of binning.
Therefore, there are (480*2-1) x (640*2-1) entries in the
mapping table versus 480x640 physical pixels.

The camera calibration functions will mostly continue to
follow the OpenCV coordinates for x-prime, x-prime-prime calculations.

From OpenCV to ISO 80000
xyz ==> yzx
"""
import csv
from numbers import Number
from pathlib import Path
from typing import Tuple

import numpy as np

from cobra_system_control.itof import N_COLS, N_ROWS
from cobra_system_control import remote
from cobra_system_control.cobra_log import log


def uv_rect2xyp(uv: np.ndarray, kinv: np.ndarray) -> np.ndarray:
    """Converts the uv pixel array to the xprime, yprime array
    """
    xyone = kinv @ uv.T
    xyone = xyone.T
    return np.stack(
        [xyone[:, 0] / xyone[:, 2],
         xyone[:, 1] / xyone[:, 2]], axis=1)


def xypp2uv(xypp: np.ndarray, k: np.ndarray) -> np.ndarray:
    """Converts (xpp, ypp) to (u,v) coordinates
    using the K matrix
    """
    k_s = np.reshape(k, (3, 3))
    uv = k_s @ np.stack([xypp[:, 0], xypp[:, 1],
                         np.ones_like(xypp[:, 0])])
    return uv.T


def xypp2XYZ(xypp: np.ndarray, depth: float) -> np.ndarray:
    """Converts xypp to (X,Y,Z)
    when a range is known
    """
    rrange = np.sqrt(xypp[:, 0] ** 2
                     + xypp[:, 1] ** 2
                     + np.ones_like(xypp[:, 0]) ** 2)
    mult = depth / rrange
    X = xypp[:, 0] * mult
    Y = xypp[:, 1] * mult
    return np.stack([X, Y, np.ones_like(Y)*mult], axis=1)


def xypp2theta_phi(xypp: np.ndarray) -> np.ndarray:
    """Converts (X,Y,Z) or (xpp, ypp, 1) to
    (r, theta, phi) for creating a mapping table

    Physics convention
    r**2 = x**2 + y**2 + z**2
    theta = arccos(z/r)
    phi = arctan(y/x)

    From OpenCV to ISO 80000
    x -> -y
    y -> -z
    z -> x
    Because the coordinates are in camera coordinates which
    are rotated 180 deg from world coordinates.

    So
    zpp = -ypp
    ypp = -xpp
    xpp = ones

    BUT when setting a steering angle of -45 degrees, the system
    steers up since the angles are LCM centric. This is flipped
    from the camera's perspective.Therefore, we need
    to flip the y-coordinate again with another -1 *.
    """
    zpp = xypp[:, 1]
    ypp = -1 * xypp[:, 0]
    xpp = np.ones_like(xypp[:, 0])

    r = np.sqrt(xpp**2 + ypp**2 + zpp**2)

    theta_phi = np.zeros_like(xypp)
    theta_phi[:, 0] = np.arccos(zpp / r)
    theta_phi[:, 1] = np.arctan2(ypp, xpp)
    return theta_phi


def rtp2XYZ(theta_phi: np.array, r: float) -> np.ndarray:
    """Converts (r, theta, phi) to XYZ in ISO 80000 convention
    """
    x = r * np.cos(theta_phi[:, 1]) * np.sin(theta_phi[:, 0])
    y = r * np.sin(theta_phi[:, 1]) * np.sin(theta_phi[:, 0])
    z = r * np.cos(theta_phi[:, 0])
    return np.stack([x, y, z], axis=1)


@remote.register_for_serialization
class PixelMapping:
    """A class to calculated the mapping between pixel location and
    far-field angle based on the results of the intrinsic
    camera calibration.
    """
    def __init__(
            self, fx: Number, fy: Number, cx: Number, cy: Number,
            k1: Number, k2: Number, k3: Number, p1: Number, p2: Number,
            n_rows: int = N_ROWS, n_cols: int = N_COLS,
            write_check: bool = True,
    ):
        if k3 == 0:
            self.fisheye = True
            self.dist = np.array([k1, k2, p1, p2]).astype(np.float32)
        else:
            raise ValueError(f'Could not figure out the calibration type from '
                             f' k3={k3}, p1={p1}, p2={p2}')

        self.intrinsic = np.array(
                [fx,  0, cx,
                 0,  fy, cy,
                 0,   0,  1]
            )
        self.n_rows = n_rows
        self.n_cols = n_cols
        self.xypp = None

        self.write_check = write_check

    def generate_mapping_arrays(self) -> Tuple[np.ndarray]:
        return self.generate_fisheye_mapping_arrays()

    def undistortPoints(self, uv_array, K, D):
        """Generates UV to theta/phi mapping from the intrinsic
        calibration coefficients and distortion values

        Behavior matches that of cv2.fisheye.undistortPoints

        if input is (0,0), output is (0,0)
        """
        uv_linear = uv_array.flatten().astype(np.float64)
        # 600,600, enforced double-precision math
        f = [float(K[0, 0]), float(K[1, 1])]
        c = [float(K[0, 2]), float(K[1, 2])]
        k = D.astype(np.float64)
        # RR is eye(3,3)
        # isEps = True  # default values are used:
        # (criteria.type ==3 TermCriteria::EPS == 2)
        pi = np.array([uv_linear[0::2], uv_linear[1::2]], dtype=np.float64)
        pw = np.array([(pi[0] - c[0])/f[0],
                       (pi[1] - c[1])/f[1]], dtype=np.float64)

        theta_d = np.sqrt(pw[0]*pw[0] + pw[1]*pw[1])
        theta_d = np.clip(theta_d, -0.5 * np.pi,  0.5 * np.pi)
        theta = theta_d.copy()

        epsilon = 1.0e-9

        nonzeros = theta_d > epsilon
        zeros = theta_d <= epsilon
        converged = zeros.copy()
        iters = 10

        while not np.all(converged) and (iters > 0):
            theta2 = theta*theta
            theta4 = theta2*theta2
            theta6 = theta4*theta2
            theta8 = theta6*theta2
            k0_theta2 = k[0] * theta2
            k1_theta4 = k[1] * theta4
            k2_theta6 = k[2] * theta6
            k3_theta8 = k[3] * theta8
            theta_fix = (
                (theta
                 * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8)
                 - theta_d)
                / (1 + 3*k0_theta2 + 5*k1_theta4 + 7*k2_theta6 + 9*k3_theta8))
            eps_idcs = np.logical_or(nonzeros, nonzeros)
            theta[eps_idcs] = theta[eps_idcs] - theta_fix[eps_idcs]
            converged = np.logical_or(zeros, np.abs(theta_fix) < epsilon)
            iters -= 1

        scale = np.ones(theta.size, dtype=np.float64)
        scale[nonzeros] = np.tan(theta[nonzeros]) / theta_d[nonzeros]
        theta_flipped = np.logical_or(np.logical_and(theta_d < 0, theta > 0),
                                      np.logical_and(theta_d > 0, theta < 0))
        fi = np.array([pw[0], pw[1]]) * scale
        fi[0][theta_flipped] = -1.0
        fi[1][theta_flipped] = -1.0
        fi[0][zeros] = 0.0
        fi[1][zeros] = 0.0

        dst = np.zeros_like(uv_linear, dtype=uv_array.dtype)
        dst[0::2] = fi[0]
        dst[1::2] = fi[1]
        return np.reshape(dst, uv_array.shape)

    def generate_fisheye_mapping_arrays(self) -> Tuple[np.ndarray]:
        u = np.arange(0, self.n_cols, dtype=np.float32)
        v = np.arange(0, self.n_rows, dtype=np.float32)
        U, V = np.meshgrid(u, v)
        uv_array = np.stack((U, V), axis=2)
        k = np.reshape(self.intrinsic, (3, 3)).astype(np.float32)
        # uout = cv2.fisheye.undistortPoints(uv_array, k, self.dist)
        uout = self.undistortPoints(uv_array, k, self.dist)
        self.xypp = np.reshape(uout, (-1, 2))
        theta_phi_array = xypp2theta_phi(self.xypp)

        # this is the format for the other functions
        uv_array = np.stack([U.ravel(), V.ravel(),
                             np.ones_like(U).ravel()], axis=1)
        return uv_array, theta_phi_array

    def generate_a2a_arrays(self) -> Tuple[np.ndarray, np.ndarray]:
        """Creates (pixel y, theta) array for use with a2a calibration

        Subtracts pi/2 from the values to reference theta from the XY plane
        """
        uv, theta_phi = self.generate_mapping_arrays()
        pixels_y = np.arange(self.n_rows)
        pixel_x = self.n_cols//2
        angles = theta_phi[:, 0] - np.pi/2
        angle = angles[np.where(uv[:, 0] == pixel_x)]
        return pixels_y, angle

    def write_a2a_mapping_file(self, path: Path):
        """Writes the (pixel y, phi) mapping table
        to a CSV
        """
        pixel_y, angle = self.generate_a2a_arrays()
        with open(Path(path).resolve(), 'w',
                  newline='', encoding='utf8') as f:
            fidwr = csv.writer(f, delimiter=',')
            for idx in range(self.n_rows):
                fidwr.writerow((
                    pixel_y[idx].astype(int),
                    (np.rad2deg(angle[idx]) * 3600).astype(int)
                ))

    def write_mapping_table_file(self, path: Path):
        """Writes the (pixel x, pixel y, theta, phi) mapping table
        to a binary file.
        """
        def write_the_table(path, mapping_array):
            log.debug('Writing pixel mapping table')
            with open(Path(path).resolve(), 'wb') as fid:
                mapping_array.tofile(fid)

        uv, theta_phi = self.generate_mapping_arrays()
        theta_phi_arcsec = (np.rad2deg(theta_phi) * 3600).astype(int)
        # combine the arrays into one
        mapping_array = np.concatenate(
            (uv[:, 0:2], theta_phi_arcsec), axis=1).astype(np.int32)
        p = Path(path).resolve()

        if self.write_check:
            if p.exists():
                arr = np.fromfile(p, dtype=np.int32)
                arr = np.reshape(arr, (-1, 4))
                try:
                    np.testing.assert_equal(arr, mapping_array)
                except AssertionError:
                    write_the_table(p, mapping_array)
            else:
                write_the_table(p, mapping_array)
        else:
            write_the_table(p, mapping_array)


DEFAULT_PIXEL_MAPPING = PixelMapping(
    fx=325, fy=325, cx=N_COLS // 2 - 0.5, cy=N_ROWS // 2 - 0.5,
    k1=-0.20193899, k2=0.09003341, p1=-0.10790229, p2=0.05483031, k3=0,
)


""" Supersampled mapping table. Here, there is a mapping location
at the nominal pixel locations and halfway between. This results
in pixel grid that is sized  (480*2-1) * (640*2-1).
The grid is centered on pixel 480, 640.
Each new pixel is 1/2 the size of the original pixel in both the
x- and y-dimension.
"""
SUPERSAMPLED_PIXEL_MAPPING = PixelMapping(
    fx=325*2, fy=325*2, cx=N_COLS-1, cy=N_ROWS-1,
    k1=-0.20193899, k2=0.09003341, p1=-0.10790229, p2=0.05483031, k3=0,
    n_rows=(N_ROWS*2-1), n_cols=(N_COLS*2-1)
)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        prog='pixel_mapping.py', description='Generates Mapping Tables')
    parser.add_argument(
        '--supersampled',
        help='Path to supersampled mapping table',
        default='./resources/supersampled_mapping_table.bin')
    parser.add_argument(
        '--default',
        help='Path to default mapping table',
        default='./resources/default_mapping_table.bin')
    args = parser.parse_args()
    print(f'input files: \n\t{args.supersampled}\n\t{args.default}')
    ss = SUPERSAMPLED_PIXEL_MAPPING
    ss.write_mapping_table_file(args.supersampled)
    dd = DEFAULT_PIXEL_MAPPING
    dd.write_mapping_table_file(args.default)
    print(dd.generate_a2a_arrays())
