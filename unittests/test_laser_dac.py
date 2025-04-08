import math
import time

import numpy as np
import pytest as pt

from cobra_system_control.laser import (
    ci_max_by_system,
    ci_percentage_array,
    LaserPowerPercentMappedOvFactory,
    LASER_PERCENT_OPTIONS,
)


@pt.mark.parametrize('whoami, pcb_rev, exp', [
    pt.param('m30', 1, 1.8),
    pt.param('m30', 2, 2.075),
    pt.param('m30', 3, 1.8),
])
def test_ci_max_by_system(whoami, pcb_rev, exp):
    ci_max = ci_max_by_system(whoami, pcb_rev)
    assert ci_max == exp


def test_ci_percentage_array():
    ci_max = 1
    slope = 1
    offset = 0
    ci_array = ci_percentage_array(ci_max, slope, offset)
    assert list(np.asarray(LASER_PERCENT_OPTIONS)/100) == list(ci_array)


@pt.mark.parametrize('whoami, pcb_rev, exp', [
    pt.param('m30', 1, 1.8),
    pt.param('m30', 2, 2.075),
    pt.param('m30', 3, 1.8),
])
def test_laser_power_cls_factory(whoami, pcb_rev, exp):
    factory = LaserPowerPercentMappedOvFactory()
    lppmo = factory(whoami, pcb_rev)
    assert lppmo.OPTIONS[0] == 1
    assert lppmo.OPTIONS[-1] == 100
    assert math.isclose(lppmo.MAP[-1], exp, abs_tol=1e-5, rel_tol=1e-8)


def test_laser_ci_full_scale(laser_ci_dac):
    assert laser_ci_dac.dac.dac_full_scale == 4.88


@pt.mark.parametrize('ci_v', [
    pt.param(0),
    pt.param(1.5),
    pt.param(3.3),
])
def test_laser_ci_field_conversion(laser_ci_dac, ci_v):
    field = laser_ci_dac.field_from_voltage(ci_v)
    v = laser_ci_dac.voltage_from_field(field)
    assert ci_v == pt.approx(v, abs=laser_ci_dac.dac.tolerance)


@pt.mark.parametrize('vlda_v', [
    pt.param(13),
    pt.param(18),
    pt.param(22),
])
def test_sh_laser_field_conversion(sh_laser_vlda_dac, vlda_v):
    field = sh_laser_vlda_dac.field_from_voltage(vlda_v)
    v = sh_laser_vlda_dac.voltage_from_field(field)
    assert vlda_v == pt.approx(v, abs=sh_laser_vlda_dac.dac.tolerance)


@pt.mark.parametrize('vlda_v', [
    pt.param(13),
    pt.param(22),
])
def test_cmb_laser_field_conversion(cmb_laser_vlda_dac, vlda_v):
    field = cmb_laser_vlda_dac.field_from_voltage(vlda_v)
    v = cmb_laser_vlda_dac.voltage_from_field(field)
    assert vlda_v == pt.approx(v, abs=cmb_laser_vlda_dac.dac.tolerance)


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize('ci_v', [
    pt.param(0),
    pt.param(0.5),
    pt.param(1),
    pt.param(1.5),
    pt.param(1.8),
])
def test_dac_linearity(ci_v, laser_ci_dac):
    laser_ci_dac.set_voltage(ci_v)
    return_val = laser_ci_dac.get_voltage()
    assert ci_v == pt.approx(return_val, abs=laser_ci_dac.dac.tolerance)
    time.sleep(0.3)
