import pytest as pt
import logging
import time

from cobra_system_control.cobra_log import log
log.setLevel(logging.DEBUG)


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize(
    'caller, m30_expected, tol', [
        pt.param('get_mon_18v_lcm', 18, 0.3),
        pt.param('get_mon_vlda', 20, 2),
        pt.param('get_mon_1p2v', 1.2, 0.2),
        pt.param('get_mon_vcc', 1.0, 0.1),
        pt.param('get_mon_vccaux', 1.8, 0.1),
        pt.param('get_mon_vccio0', 1.8, 0.1),
        pt.param('get_mon_vref_hi', 1.22, 0.05),
        pt.param('get_mon_vref_lo', 1.22, 0.05),
        pt.param('get_mon_24v', 24, 1),
        pt.param('get_mon_9v_lcm', 9, 0.2),
        pt.param('get_mon_2p8v', 2.8, 0.2),
        pt.param('get_mon_vmgh', 1.6, 0.1),

        pt.param('get_mon_vlda_currs', 0, 0.08),
        pt.param('get_mon_lcm_currs_fine', 0, 0.3),
        pt.param('get_mon_lcm_currs_coarse', 0, 0.3),
        pt.param('get_mon_laser_temp', 30, 10),
        pt.param('get_mon_lcm_temp', 35, 15),
        pt.param('get_mon_pcb_temp', 30, 15),
        pt.param('get_mon_fpga_temp', 38, 12),

        pt.param('get_mon_amb_det_1', 1, 20),
        pt.param('get_mon_amb_det_2', 1, 20),
        pt.param('get_mon_csense_ito', 1, 30),
        pt.param('get_mon_vsense_ito', 1, 30),
        pt.param('get_mon_21v_raw', 21, 3),

        # pt.param('get_mon_lcm_present',  None, 0.05),
    ])
def test_fpga_getters(caller, m30_expected, tol, sensor_head, system_type):
    expected = m30_expected

    # make sure some of the voltages are on!
    sensor_head.laser_ci_dac.setup()
    sensor_head.laser_ci_dac.set_voltage(0.01)

    if (('21' in caller) or ('lcm' in caller)):
        sensor_head.cmb_lcm_v_dac.set_voltage(21)
        sensor_head.cmb_lcm_v_dac.enable()
        sensor_head.cmb_lcm_v_dac.set_voltage(21)
        sensor_head.lcm_ctrl.write_fields(gpio_pwr_en=1)

    if (('vlda' in caller) or ('24' in caller)):
        sensor_head.cmb_laser_vlda_dac.set_voltage(50)
        sensor_head.cmb_laser_vlda_dac.enable()
        sensor_head.set_laser_vlda_combined(expected)
        sensor_head.debug.write_fields(vlda_en=1)

    if 'ito' in caller:
        sensor_head.ito_dac.setup()
        sensor_head.ito_dac.set_voltage(expected / 2)

    time.sleep(1)

    res = getattr(sensor_head.fpga_adc, caller)()
    if res == -1:
        pt.xfail('Measurement not enabled')
    elif caller == 'get_mon_9v_lcm':
        # 9V is always half of 18V
        expected = sensor_head.fpga_adc.get_mon_18v_lcm()
        assert res == pt.approx(expected/2, abs=tol)
    elif isinstance(expected, str):
        assert res == expected
    else:
        assert res == pt.approx(expected, abs=tol)

    sensor_head.laser_ci_dac.set_voltage(0.0)
    sensor_head.debug.write_fields(vlda_en=0)
    sensor_head.lcm_ctrl.write_fields(gpio_pwr_en=0)
    sensor_head.cmb_lcm_v_dac.disable()
    sensor_head.cmb_laser_vlda_dac.disable()


@pt.mark.usefixtures('integration_only')
def test_fpga_adc_calibration(sensor_head):
    """Calibrates N times in a row and verifies we can
    get the right reference voltage results.
    """
    for _ in range(5):
        sensor_head.fpga_adc.calibrate()
        assert sensor_head.fpga_adc.get_mon_vref_hi() == pt.approx(1.22, abs=0.02)
        assert sensor_head.fpga_adc.get_mon_vref_lo() == pt.approx(1.22, abs=0.02)
