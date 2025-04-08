from contextlib import nullcontext as does_not_raise

import pytest as pt

from cobra_system_control.numerical_utilities import FxpFormat


fxp_test_inputs = (
    'signed, n_bits, n_frac, init_error, n_int_expected, '
    'expected_fxp_limits, expected_dig_limits')
fxp_test_params = ([
    pt.param(True, 0, 0, pt.raises(ValueError), 0, None, None),
    pt.param(True, 10, 9, None, 0, (-1, 0.998046875), (0, 1024)),
    pt.param(False, 8, 0, None, 8, (0, 255), (0, 256))
])


@pt.mark.parametrize(
    fxp_test_inputs, [
        *fxp_test_params,
    ])
def test_fxp_format(signed, n_bits, n_frac, init_error, n_int_expected,
                    expected_fxp_limits, expected_dig_limits):
    with init_error or does_not_raise():
        fxp = FxpFormat(signed, n_bits, n_frac)
        assert (signed, n_bits, n_frac) == fxp.format


@pt.mark.parametrize(
    fxp_test_inputs, [
        *fxp_test_params,
    ])
def test_n_int(signed, n_bits, n_frac, init_error, n_int_expected,
               expected_fxp_limits, expected_dig_limits):
    if n_bits < 1:
        pt.skip("Failed Init No Test")
    fxp = FxpFormat(signed, n_bits, n_frac)
    assert n_int_expected == fxp.n_int


@pt.mark.parametrize(
    fxp_test_inputs, [
        *fxp_test_params,
    ])
def test_fxp_limits(signed, n_bits, n_frac, init_error, n_int_expected,
                    expected_fxp_limits, expected_dig_limits):
    if n_bits < 1:
        pt.skip("Failed init no test")
    fxp = FxpFormat(signed, n_bits, n_frac)
    assert expected_fxp_limits[0] == fxp.fxp_min
    assert expected_fxp_limits[1] == fxp.fxp_max


@pt.mark.parametrize(
    fxp_test_inputs, [
        *fxp_test_params,
    ])
def test_dig_limits(signed, n_bits, n_frac, init_error, n_int_expected,
                    expected_fxp_limits, expected_dig_limits):
    if n_bits < 1:
        pt.skip("Failed init no test")
    fxp = FxpFormat(signed, n_bits, n_frac)
    assert expected_dig_limits[0] == fxp.dig_min
    assert expected_dig_limits[1] == fxp.dig_max


@pt.mark.parametrize(
    fxp_test_inputs, [
        *fxp_test_params,
    ])
def test_equality(signed, n_bits, n_frac, init_error, n_int_expected,
                  expected_fxp_limits, expected_dig_limits):
    if n_bits < 1:
        pt.skip("Failed init no test")
    fxp = FxpFormat(signed, n_bits, n_frac)
    fxp_2 = FxpFormat(signed, n_bits, n_frac)
    assert fxp == fxp_2
