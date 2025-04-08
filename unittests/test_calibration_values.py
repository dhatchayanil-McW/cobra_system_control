from collections import defaultdict
from contextlib import nullcontext as does_not_raise

import numpy as np
import pytest as pt


import cobra_system_control.exceptions as cobex
from cobra_system_control.calibration_data import (
    CalItem, get_cal_hash)
from cobra_system_control.calibration_data import (
    CalGroup, CamGroup,
    DynGroup,
    RangeCalTemperatureGroup
)
from cobra_system_control.numerical_utilities import FxpFormat


@pt.mark.parametrize("fxpformat, nitems, val, vfixed, vdig, vbytes", [
    pt.param(FxpFormat(True,   4,  2), 1,             -1.749,  np.array([-1.75]),        np.array([9]),      bytearray([0x09])),
    pt.param(FxpFormat(True,   4,  2), 1,              1.999,  np.array([+1.75]),        np.array([7]),      bytearray([0x07])),
    pt.param(FxpFormat(True,   4,  2), 1,              0.000,  np.array([+0.00]),        np.array([0]),      bytearray([0x00])),
    pt.param(FxpFormat(True,  16, 15), 1,              2.000,  np.array([(1 - 2**-15)]), np.array([0x7fff]), bytearray([0x7f, 0xff])),
    pt.param(FxpFormat(True,  16, 15), 1,             -2.000,  np.array([-1.00]),        np.array([0x8000]), bytearray([0x80, 0x00])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([-1.749]),  np.array([-1.75]),        np.array([9]),      bytearray([0x09])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([+1.999]),  np.array([+1.75]),        np.array([7]),      bytearray([0x07])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([+0.000]),  np.array([+0.00]),        np.array([0]),      bytearray([0x00])),
    pt.param(FxpFormat(True,  16, 15), 1, np.array([+2.000]),  np.array([(1 - 2**-15)]), np.array([0x7fff]), bytearray([0x7f, 0xff])),
    pt.param(FxpFormat(True,  16, 15), 1, np.array([-2.000]),  np.array([-1.00]),        np.array([0x8000]), bytearray([0x80, 0x00])),
    pt.param(FxpFormat(True,   4,  2), 2, np.array([-1.749, 1.999]), np.array([-1.75, 1.75]), np.array([9, 7]), bytearray([0x09, 0x07])),
    pt.param(FxpFormat(True,   4,  2), 3, np.array([+1.999, 0.000, -1.749]), np.array([1.75,  0.00, -1.75]), np.array([7, 0, 9]), bytearray([0x07, 0x00, 0x09])),
])
def test_calibration_item_float_conversion(
        fxpformat, nitems, val, vfixed, vdig, vbytes):

    class View(CalGroup):
        hash = CalItem(0x00, FxpFormat(False, 16, 0), 1)
        c = CalItem(0x20, fxpformat, nitems)

    view = View(bytearray(80))  # initialize with 0s
    view.update_group(vfxp=dict(c=val))

    np.testing.assert_array_equal(view.c.vfxp, vfixed)
    np.testing.assert_array_equal(view.c.vdig, vdig)
    np.testing.assert_array_equal(view.c.vbytes, vbytes)
    assert isinstance(view.c.vfxp,      np.ndarray), (
        f'view.c.vfxp is {type(view.c.vfxp)}')
    assert isinstance(view.c.vfxp[0],   (int, np.float32, np.float64)), (
        f'view.c.vfxp[0] is {type(view.c.vfxp[0])}')
    assert isinstance(view.c.vbytes,    bytearray), (
        f'view.c.vbytes is {type(view.c.vbytes)}')
    assert isinstance(view.c.vbytes[0], (int, np.int32, np.int64)), (
        f'view.c.vbytes[0] is {type(view.c.vbytes[0])}')
    assert isinstance(view.c.vdig,      np.ndarray), (
        f'view.c.vdig is {type(view.c.vdig)}')
    assert isinstance(view.c.vdig[0],   (int, np.int32, np.int64)), (
        f'view.c.vdig[0] is {type(view.c.vdig[0])}')


@pt.mark.parametrize("fxpformat, nitems, vfixed, vdig, val", [
    pt.param(FxpFormat(True,   4,  2), 1, np.array([-1.75]),        np.array([9]),      bytearray([0x09])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([+1.75]),        np.array([7]),      bytearray([0x07])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([+0.00]),        np.array([0]),      bytearray([0x00])),
    pt.param(FxpFormat(True,  16, 15), 1, np.array([(1 - 2**-15)]), np.array([0x7fff]), bytearray([0x7f, 0xff])),
    pt.param(FxpFormat(True,  16, 15), 1, np.array([-1.00]),        np.array([0x8000]), bytearray([0x80, 0x00])),
    pt.param(FxpFormat(True,   4,  2), 2, np.array([-1.75, 1.75]),  np.array([9, 7]),   bytearray([0x09, 0x07])),
    pt.param(FxpFormat(True,   4,  2), 3, np.array([+1.75,  0.00, -1.75]), np.array([7, 0, 9]), bytearray([0x07, 0x00, 0x09])),
])
def test_calibration_item_byte_conversion(
        fxpformat, nitems, vfixed, vdig, val):

    class View:

        def __init__(self, ba):
            self.ba = ba
        c = CalItem(0x0, fxpformat, nitems)

    view = View(val)
    np.testing.assert_array_equal(view.c.vfxp, vfixed)
    np.testing.assert_array_equal(view.c.vdig, vdig)
    assert isinstance(view.c.vfxp,      np.ndarray), (
        f'c.vfxp is {type(view.c.vfxp)}')
    assert isinstance(view.c.vfxp[0],   (int, np.float32, np.float64)), (
        f'c.vfxp[0] is {type(view.c.vfxp[0])}')
    assert isinstance(view.c.vbytes,    bytearray), (
        f'c.vbytes is {type(view.c.vbytes)}')
    assert isinstance(view.c.vbytes[0], (int, np.int32, np.int64)), (
        f'c.vbytes[0] is {type(view.c.vbytes[0])}')
    assert isinstance(view.c.vdig,      np.ndarray), (
        f'c.vdig is {type(view.c.vdig)}')
    assert isinstance(view.c.vdig[0],   (int, np.int32, np.int64)), (
        f'c.vdig[0] is {type(view.c.vdig[0])}')


@pt.mark.parametrize("fxpformat, nitems,  vfixed, val, vbytes", [
    pt.param(FxpFormat(True,   4,  2), 1, np.array([-1.75]),        9,                  bytearray([0x09])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([+1.75]),        7,                  bytearray([0x07])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([+0.00]),        0,                  bytearray([0x00])),
    pt.param(FxpFormat(True,  16, 15), 1, np.array([(1 - 2**-15)]), 0x7fff,             bytearray([0x7f, 0xff])),
    pt.param(FxpFormat(True,  16, 15), 1, np.array([-1.00]),        0x8000,             bytearray([0x80, 0x00])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([-1.75]),        np.array([9]),      bytearray([0x09])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([+1.75]),        np.array([7]),      bytearray([0x07])),
    pt.param(FxpFormat(True,   4,  2), 1, np.array([+0.00]),        np.array([0]),      bytearray([0x00])),
    pt.param(FxpFormat(True,  16, 15), 1, np.array([(1 - 2**-15)]), np.array([0x7fff]), bytearray([0x7f, 0xff])),
    pt.param(FxpFormat(True,  16, 15), 1, np.array([-1.00]),        np.array([0x8000]), bytearray([0x80, 0x00])),
    pt.param(FxpFormat(True,   4,  2), 2, np.array([-1.75, 1.75]),  np.array([9, 7]),   bytearray([0x09, 0x07])),
    pt.param(FxpFormat(True,   4,  2), 3, np.array([+1.75,  0.00, -1.75]), np.array([7, 0, 9]), bytearray([0x07, 0x00, 0x09])),
])
def test_calibration_item_dig_conversion(
        fxpformat, nitems, vfixed, val, vbytes):

    class View(CalGroup):
        hash = CalItem(0x00, FxpFormat(False, 16, 0), 1)
        c = CalItem(0x20, fxpformat, nitems)

    view = View(bytearray(80))  # initialize with 0s
    view.update_group(vdig=dict(c=val))

    np.testing.assert_array_equal(view.c.vfxp, vfixed)
    np.testing.assert_array_equal(view.c.vbytes, vbytes)
    assert isinstance(view.c.vfxp,      np.ndarray), (
        f'view.c.vfxp is {type(view.c.vfxp)}')
    assert isinstance(view.c.vfxp[0],   (int, np.float32, np.float64)), (
        f'view.c.vfxp[0] is {type(view.c.vfxp[0])}')
    assert isinstance(view.c.vbytes,    bytearray), (
        f'view.c.vbytes is {type(view.c.vbytes)}')
    assert isinstance(view.c.vbytes[0], (int, np.int32, np.int64)), (
        f'view.c.vbytes[0] is {type(view.c.vbytes[0])}')
    assert isinstance(view.c.vdig,      np.ndarray), (
        f'view.c.vdig is {type(view.c.vdig)}')
    assert isinstance(view.c.vdig[0],   (int, np.int32, np.int64)), (
        f'view.c.vdig[0] is {type(view.c.vdig[0])}')


@pt.mark.parametrize("wargs, rdata, error", [
    pt.param((bytearray([0xe3]), ), bytearray([0x69, 0xf2]), does_not_raise()),
    pt.param((bytearray([0xa1]), bytearray([0xb2])), bytearray([0xda, 0xb4]), does_not_raise()),
    pt.param(('foo',), None, pt.raises(TypeError)),
    pt.param((bytearray([0xa1]), bytearray([0xb2])), None, pt.raises(TypeError)),
])
def test_get_cal_hash(wargs, rdata, error):
    if rdata is None:
        with error:
            rval = get_cal_hash(wargs)
    else:
        val = bytearray([])
        for ba in wargs:
            val.extend(ba)
        rval = get_cal_hash(val)
        assert rval == rdata


@pt.mark.parametrize('ba, expected', [
    pt.param(bytearray([0xFF] * 100), False),
    pt.param(bytearray([0x00] * 100), False),
    pt.param(bytearray([0x01] * 100), True),
])
def test_cam_cal_group_loaded(ba, expected):
    cam_group = CamGroup(ba)
    assert cam_group.is_loaded is expected


@pt.mark.parametrize('pga_gain, doff, hhash, expected', [
    pt.param(10, 100, 57215, True),
    pt.param(9, 100, 57215, False),
    pt.param(101, 100, 57215, False),
])
def test_dyn_cal_group_valid(pga_gain, doff, hhash, expected):
    ba = bytearray(100)
    dyn = DynGroup(ba)
    dyn.update_group(vdig=dict(
        doff_diff_adu=doff,
        pga_gain=pga_gain)
    )
    # spoof the hash
    ba[DynGroup.hash.addr_offset: DynGroup.hash.addr_offset
       + DynGroup.hash.nbytes_total] = int.to_bytes(
           hhash, 2, byteorder='big', signed=False)

    assert dyn.hash.vdig[0] == hhash
    assert dyn.is_valid is expected


@pt.mark.parametrize('new_value_dict, error', [
    pt.param(dict(pga_gain=10, doff_diff_adu=100), None),
    pt.param(dict(pga_gain=10), pt.raises(cobex.CalibrationError)),
    pt.param(dict(doff_diff_adu=100), pt.raises(cobex.CalibrationError)),
])
def test_dyn_cal_group_assignment(new_value_dict, error):
    ba = bytearray(100)
    dyn = DynGroup(ba)
    with error or does_not_raise():
        dyn.update_group(vdig=new_value_dict)
        assert dyn.pga_gain.vdig[0] == new_value_dict['pga_gain']
        assert dyn.doff_diff_adu.vdig[0] == new_value_dict['doff_diff_adu']


def test_cal_size_bytes(cal_data):
    expected_size = 0
    for g in cal_data.groups():
        for i in g.all_items():
            expected_size = max(expected_size, i.span[1] - 1)

    assert cal_data.size_bytes() == RangeCalTemperatureGroup.mm_per_celsius_0908.span[1]


def test_cal_empty(cal_data):
    cal_data = cal_data.empty()
    assert len(cal_data.ba) == cal_data.size_bytes()


def test_cal_data_address(cal_data):
    assert cal_data.ADDRESS_BASE == 0x25_0000
    assert cal_data.MAX_OFFSET == 0x00_0fff


@pt.mark.parametrize('num_vals, error', [
    pt.param(None, None),
    pt.param(10, pt.raises(cobex.CalibrationSizeError)),
])
def test_cal_instantiate_cal(num_vals, error, cal_data):
    nv = int(cal_data.size_bytes() if num_vals is None else num_vals)
    ba = bytearray([0] * nv)
    with error or does_not_raise():
        cal_data(ba)


def test_cal_data_unique_addresses(cal_data):

    d = defaultdict(int)
    for g in cal_data.groups():
        for i in g.all_items():
            for addr in range(i.addr_offset, i.addr_offset + i.nbytes_total):
                d[addr] += 1

    for addr, val in d.items():
        assert val == 1, f'address {addr:#04x} mapped twice (possible overlap in cal item)'
