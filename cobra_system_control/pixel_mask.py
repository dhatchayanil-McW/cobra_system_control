"""
file: pixel_mask.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provides utilities to create a default
pixel mask. The pixel mask sets imager
pixels that are outside the TX field of illumination
to zero range inside RawToDepth.
"""
import numpy as np

from cobra_system_control.itof import N_ROWS, N_COLS


def create_default_pixel_mask(cx, cy):
    """Creates default pixel mask determined from Zemax ray
    tracing results

    M30 RX Lens image circle is 1.365mm radius for 120 deg FOV
    with +/- 25um focus range.

    Masked pixels outside the circle should be value
    Kept pixels inside the circle should be value
    """
    # Radius in number of pixels
    # Pixels are 5um
    pixel_radius = int(1.365e-3 / 5e-6)

    # Center of ellipse from inputs
    u = np.arange(N_COLS)-cx
    v = np.arange(N_ROWS)-cy
    U, V = np.meshgrid(u, v)
    R = np.sqrt(U**2 + V**2)
    # Need to make the center pixel not get removed since
    # it equals zero
    R[R == 0] = 0.01
    # Then filter
    R[R > pixel_radius] = 0
    R[R != 0] = 1
    return R
