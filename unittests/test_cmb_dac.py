import pytest as pt


def test_check_cmb_vlda_payload(cmb_laser_vlda_dac):
    with pt.raises(ValueError):
        cmb_laser_vlda_dac.dac.check_payload_size(
            (2**cmb_laser_vlda_dac.dac.dac_bits + 1)
            << cmb_laser_vlda_dac.dac.bit_shift)


def test_check_sh_vlda_payload(sh_laser_vlda_dac):
    with pt.raises(ValueError):
        sh_laser_vlda_dac.dac.check_payload_size(
            (2**sh_laser_vlda_dac.dac.dac_bits + 1)
            << sh_laser_vlda_dac.dac.bit_shift)


def test_check_sensorv_payload(sensor_v_dac):
    with pt.raises(ValueError):
        sensor_v_dac.dac.check_payload_size(
            (2**sensor_v_dac.dac.dac_bits + 1) << sensor_v_dac.dac.bit_shift)


@pt.mark.parametrize("voltage, field", [
    pt.param(12.36, 2**16-1),
    pt.param(16.67, (2**16-1)//2),
    pt.param(20.97, 0),
    ])
def test_lcmv_field_conversion(
        cmb_lcm_v_dac, voltage, field, board_type, system_type):
    if board_type == 'nxp':
        msg = (f'dac field from voltage {voltage}, '
               f'{cmb_lcm_v_dac.field_from_voltage(voltage)}, '
               f'voltage from field {cmb_lcm_v_dac.voltage_from_field(field)}')
        assert cmb_lcm_v_dac.field_from_voltage(voltage) == pt.approx(field, abs=1000), msg
        assert cmb_lcm_v_dac.voltage_from_field(field) == pt.approx(voltage, abs=0.25), msg


@pt.mark.parametrize("voltage, field", [
    pt.param(10.9, 2**16-1),
    pt.param(17.45, (2**16-1)//2),
    pt.param(24, 0),
    ])
def test_vlda_field_conversion(cmb_laser_vlda_dac, voltage, field, board_type):
    laser_vlda_dac = cmb_laser_vlda_dac
    if board_type == 'nxp':
        msg = (f'dac field from voltage {voltage}, '
               f'{laser_vlda_dac.field_from_voltage(voltage)}, '
               f'voltage from field {laser_vlda_dac.voltage_from_field(field)}')
        assert laser_vlda_dac.field_from_voltage(voltage) == pt.approx(field, abs=1000), msg
        assert laser_vlda_dac.voltage_from_field(field) == pt.approx(voltage, abs=0.25), msg


@pt.mark.parametrize("voltage, field", [
    pt.param(2.75, 2**16-1),
    pt.param(3.35, (2**16-1)//2),
    pt.param(3.95, 0),
    ])
def test_rxv_field_conversion(sensor_v_dac, voltage, field, board_type):
    if board_type == 'nxp':
        msg = (f'dac field from voltage {voltage}, '
               f'{sensor_v_dac.field_from_voltage(voltage)}, '
               f'voltage from field, {sensor_v_dac.voltage_from_field(field)}')
        assert sensor_v_dac.field_from_voltage(voltage) == pt.approx(field, abs=1000), msg
        assert sensor_v_dac.voltage_from_field(field) == pt.approx(voltage, abs=0.25), msg
