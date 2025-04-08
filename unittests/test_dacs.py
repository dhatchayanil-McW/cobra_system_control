import time

import pytest as pt


@pt.mark.parametrize('ito_v', [
    pt.param(-9),
    pt.param(0),
    pt.param(4.5),
    pt.param(9),
])
def test_ito_field_conversion(ito_dac, ito_v):
    field = ito_dac.field_from_voltage(ito_v)
    v = ito_dac.voltage_from_field(field)
    assert ito_v == pt.approx(v, abs=ito_dac.dac.tolerance)


def test_ito_full_scale(ito_dac):
    assert ito_dac.dac.dac_full_scale == 2.44


@pt.mark.skip('ITO doesnt work this way now')
@pt.mark.parametrize('vito', [
    -9, -4.5, 0, 4.5, 9
])
def test_ito_dac_linearity(mock, vito, system_type, sensor_head):
    if mock:
        pt.skip("Integration only")

    sensor_head.lcm_ctrl.write_fields(gpio_pwr_en=1)
    sensor_head.ito_dac.enable()
    sensor_head.ito_dac.set_voltage(vito)
    time.sleep(3)

    rdac = sensor_head.ito_dac.get_voltage()
    assert vito == pt.approx(rdac, abs=sensor_head.ito_dac.dac.tolerance), f'set {vito}, dac returned {rdac}'
    r9v = sensor_head.fpga_adc.get_mon_9v_lcm()
    # assert r9v == pt.approx(9, abs=0.1), f'9V reference is off, {r9v}'
    # Vsense is 9 + (-2.05 * ((1.5 * 9 - 3.57 * DAC) -9)
    dac_v = sensor_head.ito_dac.field_unshifted_from_voltage(vito) / 2**sensor_head.ito_dac.dac.dac_bits * sensor_head.ito_dac.dac.dac_full_scale
    mito = r9v + (-2.05 * ((1.5 * r9v - 3.57 * dac_v) - r9v))
    # Some are calcualting out to negative. So let's abs it
    mito = abs(mito)
    radc = sensor_head.fpga_adc.get_mon_vsense_ito()

    assert mito == pt.approx(radc, abs=0.5), f'set vito {vito:.3f}, expected mito {mito:.3f}, read {radc:.3f}, dac_v {dac_v:.3f}, 9v {r9v:.3f}'
    sensor_head.lcm_ctrl.write_fields(gpio_pwr_en=0)
