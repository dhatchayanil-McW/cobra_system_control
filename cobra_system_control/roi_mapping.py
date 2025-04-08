"""
file: roi_mapping.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provides the RoiMapping class that converts
the angle selection to an imager start row
and an LCM steering order through the
A2A Calibration coefficients and the
PixelMapping table.
"""
import dataclasses as dc
from typing import Sequence, List, Tuple
from numbers import Number

import numpy as np

from numpy.polynomial.polynomial import polyval

from cobra_system_control.itof import N_ROWS
from cobra_system_control.metasurface import LcmAssembly
from cobra_system_control.pixel_mapping import PixelMapping
from cobra_system_control import remote

import cobra_system_control.exceptions as cobex


def myround(x: Number, base: int):
    if not isinstance(x, Number):
        raise ValueError(
            f'myround value must be a number but {x} is {type(x)}')
    if not isinstance(base, int):
        raise ValueError(
            f'myround base must be an int but {base} is type {type(base)}')
    return (base * np.round(float(x) / base)).astype(np.uint16)


@remote.register_for_serialization
@dc.dataclass
class RoiMapping:
    """A class to calculate the mapping between LCM steering
    order and the ITOF pixel row using the coefficients from
    the A2A calibration. This ensures that the correct pixels
    are read during scanning.
    """
    a2a_coefficients: Sequence[Number]
    pixel_mapping: PixelMapping
    lcm_assembly: LcmAssembly
    pixel_mapping_a2a_arrays: np.ndarray = dc.field(init=False)

    def __post_init__(self):
        self.pixel_mapping_a2a_arrays = self.pixel_mapping.generate_a2a_arrays()

    def __call__(self, *,
                 angles: np.ndarray = None,
                 s_rows: np.ndarray = None,
                 roi_rows: int = 8,
                 trim_duplicates: bool = False) -> Tuple[List[int], List[int]]:
        """Given an array of angles and the target ROI height, compute all
        sensor starting rows and LCM steering orders

        20230503: A2A was updated to fit to the peak pixel instead of the
        start row for a 20 row ROI.

        Since we have this, we can determine the
        """
        a2a_row_num = 20
        if self.a2a_coefficients is None:
            raise cobex.CalibrationError(
                'A2A coefficients unavailable, cannot compute ROI and order '
                'mappings.')

        if all(x is None for x in (angles, s_rows)):
            raise cobex.CalibrationError(
                'RoiMapping args angles and s_rows cannot both be None')

        if all(x is not None for x in (angles, s_rows)):
            raise cobex.CalibrationError(
                'RoiMapping args angles and s_rows cannot both be provided')

        if angles is not None:
            # get the starting rows from intrinsic calibration
            v = self.pixel_mapping_a2a_arrays[0]
            a_rad = self.pixel_mapping_a2a_arrays[1]
            a_deg = np.rad2deg(a_rad)
            s_rows = []
            for angle in angles:
                idx = np.argmin(abs(a_deg - angle))
                # keep this at 20 row since that's what A2A is done with
                s_rows.append(int(v[idx]) - a2a_row_num // 2)

        # clamp the starting rows in case any are out of bounds
        s_rows = np.clip(s_rows, 0, (N_ROWS-1) - roi_rows)

        # evaluate the LCM steering angle to hit the given center row
        lcm_angles = polyval(s_rows, self.a2a_coefficients)

        orders = np.array(
            [self.lcm_assembly.angle_to_order(a) for a in lcm_angles])

        if roi_rows != 480:
            # Adjust srow by the delta between a2a cal at 20 rows and roi_rows
            s_rows = np.array([x + (a2a_row_num-roi_rows)//2 for x in s_rows])
            # clip again and match order length
            s_rows = np.clip(s_rows, 0, N_ROWS)
            a_s_rows = s_rows[s_rows >= 0]
            a_orders = orders[s_rows >= 0]
            b_s_rows = a_s_rows[a_s_rows < (N_ROWS - roi_rows)]
            b_orders = a_orders[a_s_rows < (N_ROWS - roi_rows)]
            s_rows = b_s_rows
            orders = b_orders
        else:
            s_rows = np.clip(s_rows, 0, N_ROWS)

        min_order, *_, max_order = (
            self.lcm_assembly.order_ov.nonzero_sorted_orders())
        orders = np.clip(orders, min_order, max_order)

        if trim_duplicates:
            return self.duplicate_trimmer(orders, s_rows)
        else:
            assert len(orders) == len(s_rows)
            return orders.tolist(), s_rows.tolist()

    def duplicate_trimmer(self, orders, s_rows):
        """Iterates through the order and start row lists.
        if an order is already counted, this deletes the duplicate.
        This could cause unintended behavior at the edges.
        """
        new_o = []
        new_s = []
        for o, s in zip(orders, s_rows):
            if o not in new_o:
                new_o.append(o)
                new_s.append(s)
        assert len(new_o) == len(new_s)
        return new_o, new_s
