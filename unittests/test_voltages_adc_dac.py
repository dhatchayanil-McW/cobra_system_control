"""Testing module for the applied voltages from the NCB
to the Sensor Head. Tests the DACs and ADCs for
24V, 21V, 3.3V, and combined VLDA setting.
"""
import logging
import time

import numpy as np
import pytest as pt

from cobra_system_control.cobra_log import log
log.setLevel(logging.DEBUG)


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize("wvolt, ncb_exp, sen_exp", [
    pt.param(13, 13, 13),
    pt.param(19, 19, 19),
    pt.param(24, 24, 24),
    pt.param(50, 'max', 'max'),
])
def test_v24p0(cobra, wvolt, ncb_exp, sen_exp):
    time.sleep(5)
    cobra.sen.sh_laser_vlda_dac.disable()
    cobra.cmb_dac_adc_calibration(cobra.cmb_laser_vlda_dac,
                                  cobra.cmb_adc.v24_voltage,
                                  cobra.sensor_24v_en_gpio)
    cobra.sen.cmb_laser_vlda_dac.enable()

    cobra.cmb_lcm_v_dac.disable()

    cobra.cmb_laser_vlda_dac.set_voltage(wvolt)
    time.sleep(1)

    rncb_dac = cobra.cmb_laser_vlda_dac.get_voltage()
    num = 10
    l_rncb_adc = []
    for _ in range(num):
        l_rncb_adc.append(cobra.cmb_adc.get_mon_all_channels()['v24p0_volts_cb'])
        time.sleep(0.05)
    rncb_adc = np.mean(l_rncb_adc)

    l_rsen_adc = []
    for _ in range(num):
        l_rsen_adc.append(cobra.sen.fpga_adc.get_mon_24v())
        time.sleep(0.05)
    rsen_adc = np.mean(l_rsen_adc)

    msg = (
        f'wvolt {wvolt:0.2f}; '
        f'rncb_adc {rncb_adc:0.2f}; '
        f'rncb_dac {rncb_dac:0.2f}; '
        f'rsen_adc {rsen_adc:0.2f}; '
        f'slope {cobra.cmb_laser_vlda_dac.slope:0.5f}; '
        f'offset {cobra.cmb_laser_vlda_dac.offset:0.5f}; '
    )
    log.info(msg)

    # The min and max voltages are dependent on the on-system calibration
    if ncb_exp == 'max':
        ncb_exp = sen_exp = (
            (0 - cobra.cmb_laser_vlda_dac.offset) / cobra.cmb_laser_vlda_dac.slope)

    assert rncb_adc == pt.approx(ncb_exp, abs=0.3), f'diff {rncb_adc - ncb_exp:0.2f} outside 0.3 tol; {msg}'
    assert rncb_dac == pt.approx(ncb_exp, abs=0.1), f'diff {rncb_dac - ncb_exp:0.2f} outside 0.1 tol; {msg}'
    assert rsen_adc == pt.approx(sen_exp, abs=0.5), f'diff {rsen_adc - sen_exp:0.2f} outside 0.5 tol; {msg}'

    cobra.sen.sh_laser_vlda_dac.disable()
    cobra.cmb_laser_vlda_dac.disable()


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize("wvolt, ncb_exp, sen_exp", [
    pt.param(13, 13, 13),
    pt.param(16, 16, 16),
    pt.param(20, 20, 20),
    pt.param(50, 'max', 'max'),
])
def test_v21p0(cobra, wvolt, ncb_exp, sen_exp):
    cobra.cmb_dac_adc_calibration(cobra.cmb_lcm_v_dac,
                                  cobra.cmb_adc.v21_voltage,
                                  cobra.sensor_21v_en_gpio)
    cobra.sen.cmb_laser_vlda_dac.enable()

    cobra.sen.sh_laser_vlda_dac.disable()
    cobra.sen.cmb_lcm_v_dac.disable()
    # Turn on just on CMB side
    cobra.sensor_21v_en_gpio.enable()

    cobra.cmb_lcm_v_dac.set_voltage(wvolt)
    time.sleep(1)

    rncb_dac = cobra.cmb_lcm_v_dac.get_voltage()

    num = 10
    l_rncb_adc = []
    for _ in range(num):
        l_rncb_adc.append(cobra.cmb_adc.get_mon_all_channels()['v21p0_volts_cb'])
    rncb_adc = np.mean(l_rncb_adc)

    l_rsen_adc = []
    for _ in range(num):
        l_rsen_adc.append(cobra.sen.fpga_adc.get_mon_21v_raw())
        time.sleep(0.05)
    rsen_adc = np.mean(l_rsen_adc)

    msg = (
        f'wvolt {wvolt:0.2f}; '
        f'rncb_adc {rncb_adc:0.2f}; '
        f'rncb_dac {rncb_dac:0.2f}; '
        f'rsen_adc {rsen_adc:0.2f}; '
        f'slope {cobra.cmb_lcm_v_dac.slope:0.5f}; '
        f'offset {cobra.cmb_lcm_v_dac.offset:0.5f}; '
    )
    log.info(msg)

    # The min and max voltages are dependent on the on-system calibration
    if ncb_exp == 'max':
        ncb_exp = sen_exp = (
            (0 - cobra.cmb_lcm_v_dac.offset) / cobra.cmb_lcm_v_dac.slope)

    assert rncb_adc == pt.approx(ncb_exp, abs=0.3), f'diff {rncb_adc - ncb_exp:0.2f} outside 0.3 tol; {msg}'
    assert rncb_dac == pt.approx(ncb_exp, abs=0.1), f'diff {rncb_dac - ncb_exp:0.2f} outside 0.1 tol; {msg}'
    assert rsen_adc == pt.approx(sen_exp, abs=0.5), f'diff {rsen_adc - sen_exp:0.2f} outside 0.5 tol; {msg}'

    cobra.sensor_21v_en_gpio.disable()
    cobra.sen.sh_laser_vlda_dac.disable()
    cobra.sen.cmb_laser_vlda_dac.disable()


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize("wvolt, ncb_exp", [
    pt.param(3.3, 3.3),
    pt.param(3.6, 3.6),
])
def test_read_v3p3(cobra, wvolt, ncb_exp):
    cobra.cmb_sensor_v_dac.set_voltage(wvolt)
    time.sleep(1)

    rncb_dac = cobra.cmb_sensor_v_dac.get_voltage()


    num = 10
    l_rncb_adc = []
    for _ in range(num):
        l_rncb_adc.append(cobra.cmb_adc.get_mon_all_channels()['v3p3_volts_cb'])
    rncb_adc = np.mean(l_rncb_adc)

    if rncb_adc > 5:
        pt.xfail('Rev1 NCB cannot measure 3.3V rail voltage properly')

    msg = (
        f'wvolt {wvolt:0.2f}; '
        f'rncb_adc {rncb_adc:0.2f}; '
        f'rncb_dac {rncb_dac:0.2f}; '
        'Rev 1 NCB cannot measure 3.3V rail voltage properly'
    )

    tol = 0.2
    assert rncb_adc == pt.approx(ncb_exp, abs=tol), f'diff {rncb_adc - ncb_exp:0.2f} outside {tol} tol; {msg}'
    assert rncb_dac == pt.approx(ncb_exp, abs=tol), f'diff {rncb_dac - ncb_exp:0.2f} outside {tol} tol; {msg}'
    cobra.cmb_sensor_v_dac.set_voltage(3.4)


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize('vlda_set, sen_exp', [
    pt.param(13, 13),
    pt.param(15, 15),
    pt.param(18, 18),
    pt.param(22, 22),
    pt.param(23, 23),
    ])
@pt.mark.usefixtures('integration_only')
def test_get_set_laser_vlda_combined(
        vlda_set, sen_exp, cobra
):
    cobra.sen.sh_laser_vlda_dac.disable()
    cobra.sen.cmb_lcm_v_dac.disable()
    time.sleep(4)

    cobra.sen.cmb_laser_vlda_dac.enable()
    cobra.cmb_dac_adc_calibration(cobra.cmb_laser_vlda_dac,
                                  cobra.cmb_adc.v24_voltage,
                                  cobra.sensor_24v_en_gpio)

    cobra.sen.sh_laser_vlda_dac.enable()
    cobra.sen.set_laser_vlda_combined(20)
    time.sleep(1)
    cobra.sen.sh_vlda_dac_adc_calibration()

    cobra.sen.set_laser_vlda_combined(vlda_set)
    time.sleep(1)
    rcombined = cobra.sen.get_laser_vlda_combined()

    rncb_dac = cobra.sen.cmb_laser_vlda_dac.get_voltage()

    num = 10
    l_rncb_adc = []
    for _ in range(num):
        l_rncb_adc.append(cobra.cmb_adc.get_mon_all_channels()['v24p0_volts_cb'])
        time.sleep(0.05)
    rncb_adc = np.mean(l_rncb_adc)

    l_rsen_adc = []
    for _ in range(num):
        l_rsen_adc.append(cobra.sen.fpga_adc.get_mon_24v())
        time.sleep(0.05)
    rsen_adc_24 = np.mean(l_rsen_adc)

    l_rsen_adc = []
    for _ in range(num):
        l_rsen_adc.append(cobra.sen.fpga_adc.get_mon_vlda())
        time.sleep(0.05)
    rsen_adc_vlda = np.mean(l_rsen_adc)

    rsen_dac = cobra.sen.sh_laser_vlda_dac.get_voltage()

    ncb_exp = cobra.cmb_laser_vlda_dac.voltage_from_field(
        cobra.cmb_laser_vlda_dac.field_from_voltage(50))

    msg = (
        f'rcombined {rcombined:0.2f}; '
        f'rncb_dac {rncb_dac:0.2f}; '
        f'rncb_adc {rncb_adc:0.2f}; '
        f'rsen_dac {rsen_dac:0.2f}; '
        f'rsen_adc_vlda {rsen_adc_vlda:0.2f}; '
        f'rsen_adc_24 {rsen_adc_24:0.2f}; '
        f'ncb slope {cobra.cmb_laser_vlda_dac.slope:0.4f}; '
        f'ncb offset {cobra.cmb_laser_vlda_dac.offset:0.4f} '
        f'sen slope {cobra.sen.sh_laser_vlda_dac.slope:0.4f} '
        f'sen offset {cobra.sen.sh_laser_vlda_dac.offset:0.4f} '
    )
    log.info(msg)

    assert rcombined == pt.approx(sen_exp, abs=0.2), f'diff {rcombined - sen_exp:0.2f} outside 0.2 tol; {msg}'
    assert rsen_dac == pt.approx(sen_exp, abs=0.2), f'diff {rsen_dac - sen_exp:0.2f} outside 0.2 tol; {msg}'
    assert rsen_adc_vlda == pt.approx(sen_exp, abs=0.2), f'diff {rsen_adc_vlda - sen_exp:0.2f} outside 0.2 tol; {msg}'

    assert rncb_dac == pt.approx(ncb_exp, abs=0.1), f'diff {rncb_dac - ncb_exp:0.2f} outside 0.1 tol; {msg}'
    assert rncb_adc == pt.approx(ncb_exp, abs=0.2), f'diff {rncb_adc - ncb_exp:0.2f} outside 0.2 tol; {msg}'
    assert rsen_adc_24 == pt.approx(ncb_exp, abs=0.5), f'diff {rsen_adc_24 - ncb_exp:0.2f} outside 0.5 tol; {msg}'

    cobra.sen.sh_laser_vlda_dac.disable()
    cobra.sen.cmb_laser_vlda_dac.disable()
