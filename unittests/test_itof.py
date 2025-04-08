from contextlib import nullcontext as does_not_raise
import random

import numpy as np
from unittest.mock import MagicMock

import pytest as pt
from cobra_system_control import itof as itof_


def test_itof_defaults():
    assert itof_.NPULSE == 63
    assert itof_.DPULSE == 63
    assert itof_.DLASER == 12
    assert itof_.DUMMY_BURSTS == 12
    assert itof_.SUBFRAME_DEAD_TIME_US == 5.25
    assert itof_.LASER_PREHEAT == 0
    assert itof_.DUMMY_PREHEAT == 1
    assert itof_.MAX_LOCAL_DC == 0.5
    assert itof_.PLL2_FREQ_HZ == 1e9


def test_itof_read_adc_bits(mock, itof):
    if mock:
        itof.read_fields.side_effect = lambda f: {
            'adc_bits': 156}.get(f)
    bits = itof.read_fields('adc_bits')
    assert bits == 156


def test_itof_pll2(mock, itof):
    if mock:
        itof.read_fields.side_effect = lambda f: {
            'pll2_div_n_lo': 250,
            'pll2_div_n_hi': 0,
            'pll2_div_i': 2,
            'pll2_div_a': 1,
            'pll2_div_b': 1}.get(f)

    rval = itof.read_fields('pll2_div_n_lo')
    assert rval == 250, f'{rval}, {rval:#06x} != 250, {250:#06x}'
    rval = itof.read_fields('pll2_div_n_hi')
    assert rval == 0, f'{rval}, {rval:#06x} != 0, {0:#06x}'
    rval = itof.read_fields('pll2_div_i')
    assert rval == 2, f'{rval}, {rval:#06x} != 2, {2:#06x}'
    rval = itof.read_fields('pll2_div_a')
    assert rval == 1, f'{rval}, {rval:#06x} != 1, {1:#06x}'
    rval = itof.read_fields('pll2_div_b')
    assert rval == 1, f'{rval}, {rval:#06x} != 1, {1:#06x}'


def test_read_write(mock, itof):
    # Try a bunch of reads and writes to register
    for _ in range(100):
        wdata = random.randint(0, 0xffff)
        if mock:
            itof.read_fields.return_value = wdata

        itof.write_fields(rwin0_s=wdata)
        rdata = itof.read_fields('rwin0_s')
        assert rdata == wdata, (
            f' rdata {rdata}, {rdata:#06x} != '
            f'wdata {wdata}, {wdata:#06x}')


def test_mod_freq_int_ov():
    assert itof_.ModFreqIntOv.OPTIONS == range(3, 11)

    for i in itof_.ModFreqIntOv.OPTIONS:
        assert itof_.ModFreqIntOv(i).clk_freq_hz == pt.approx(1/(i*10**-9), rel=0.01)
        assert itof_.ModFreqIntOv(i).laser_freq_hz == pt.approx(1/(i*10**-9)/3, rel=0.01)
        assert itof_.ModFreqIntOv(i).clk_period_s == pt.approx(i * 10**-9, rel=0.01)
        assert itof_.ModFreqIntOv(i).field == i - 3


def test_npulse_ov():
    assert itof_.NPulseOv.OPTIONS == range(0, 2 ** 12 - 1, 3)
    assert itof_.NPulseOv(258).fields == (2, 1)
    assert itof_.NPulseOv(258).lo == 2
    assert itof_.NPulseOv(258).hi == 1
    assert itof_.NPulseOv(258) == itof_.NPulseOv.from_fields(*itof_.NPulseOv(258).fields)


def test_dpulse_ov():
    assert itof_.DPulseOv.OPTIONS == range(0, 2 ** 6 - 1, 3)


def test_dlaser_ov():
    assert itof_.DLaserOv.OPTIONS == range(0, 2 ** 6 - 1, 3)


def test_pleco_mode():
    assert itof_.PlecoMode.DMFD.kind == 'dmfd'
    assert itof_.PlecoMode.DMFD.n_subframes == 6
    assert itof_.PlecoMode.DMFD.n_taps == 3
    assert itof_.PlecoMode.DMFD.n_freq == 2

    mode_names = [itof_.PlecoMode(e).kind for e in itof_.PlecoMode]
    assert 'dmfd' in mode_names

    # these are currently unsupported
    assert 'smfd' not in mode_names
    assert 'video' not in mode_names


def test_inte_time_s_bv():
    assert itof_.InteTimeSBv.MAX_LASER_INTE_SECONDS == 20e-6 + 1e-12
    assert itof_.InteTimeSBv.LIMITS == (1e-6 - 1e-12, itof_.InteTimeSBv.MAX_LASER_INTE_SECONDS)


def test_start_row():
    assert itof_.StartRow.OPTIONS == range(480)
    val = random.choice(range(480))
    assert itof_.StartRow(val).field == val + 4
    assert itof_.StartRow(val).value == val


def test_roi_rows():
    assert itof_.RoiRows.OPTIONS == (6, 8, 20, 480)
    for i in itof_.RoiRows.OPTIONS:
        assert itof_.RoiRows(i).field == i + 4
        assert itof_.RoiRows(i).value == i


def test_fov_rows_ov():
    assert itof_.FovRowsOv.OPTIONS == range(1, 480+1)
    val = random.choice(range(1, 480+1))
    assert itof_.FovRowsOv(val).field == val
    assert itof_.FovRowsOv(val).value == val


def test_delay_ns_bv():
    assert itof_.DelayNsBv.MAX_COARSE == 9
    assert itof_.DelayNsBv.MAX_FINE == 9
    assert itof_.DelayNsBv.GATE_COARSE_NS == 0.650
    assert itof_.DelayNsBv.GATE_FINE_NS == 0.170
    assert itof_.DelayNsBv.TOLERANCE == itof_.DelayNsBv.GATE_FINE_NS

    assert itof_.DelayNsBv.from_fields(4, 3).value == pt.approx(3.11, rel=1e-6)
    assert itof_.DelayNsBv.from_fields(7, 1).value == pt.approx(4.72, rel=1e-6)

    # self checking loop
    delays = np.arange(*itof_.DelayNsBv.LIMITS, 0.1)
    for d in delays:
        # this is the delay we want
        delay_bv = itof_.DelayNsBv(d)

        # the associated register settings
        c, f = delay_bv.fields

        # ensure that making a new delay with these settings
        # is within numerical precision
        assert delay_bv == itof_.DelayNsBv.from_fields(c, f)


def test_pga_gain_bv():
    assert itof_.PgaGainOv.OPTIONS == range(0, 2 ** 5 - 1)

    for real_gain, reg in zip(np.arange(1.0, 4.1, .1), range(0, 31)):
        assert itof_.PgaGainOv(reg).gain == pt.approx(real_gain, abs=1e-6)
        assert itof_.PgaGainOv(reg).field == reg
        assert pt.approx(itof_.PgaGainOv.from_gain(real_gain).field,
                         abs=1e-6) == reg


@pt.mark.parametrize(
    's_row, roi_rows, freq, inte_time_s, error',
    [
        pt.param(0, 480, (6, 5), (14e-6, 14e-6), None, id='Typical'),
        pt.param(0, 480, (6, 5), 14e-6, None, id='Expand inte time to tuple'),
        pt.param(0, 480, (7, 5), 14e-6, pt.raises(ValueError), id='Incorrect freq spec'),
        pt.param(0, 480, (6, 5), 1e-9, pt.raises(ValueError), id='Integration time too small'),
        pt.param(461, 20, (6, 5), 14e-6, pt.raises(ValueError), id='Rows out of bounds'),
        pt.param(460, 20, (6, 5), 14e-6, None, id='Rows in bounds'),
        #pt.param(0, 20, (7, 6), 15e-6, pt.raises(cobex.CalibrationError), id='Mod freq calibration not completed'),
    ])
def test_frame_settings(s_row, roi_rows, freq, inte_time_s, error):
    mode = itof_.PlecoMode.DMFD
    error = error or does_not_raise()
    with error:
        fs = itof_.FrameSettings(start_row=s_row,
                                 roi_rows=roi_rows,
                                 mod_freq_int=freq,
                                 inte_time_s=inte_time_s,
                                 )

        assert isinstance(fs.inte_time_s, tuple)
        assert len(fs.inte_time_s) == 2
        assert fs.metadata_size == 640 * itof_.N_TAPS
        assert fs.frame_size == (
            640 + roi_rows * 640 * mode.n_subframes) * itof_.N_TAPS


@pt.mark.parametrize('roi_rows, freqs, inte_time, min_frm_len', [
    (8, (8, 7), 15e-6, 63),
    (8, (8, 7), 5e-6, 52),
    (8, (9, 8), 15e-6, 64),
    (8, (9, 8), 5e-6, 53)
])
def test_frame_settings_timing(roi_rows, freqs, inte_time, min_frm_len):
    fs = itof_.FrameSettings(0, roi_rows, inte_time_s=inte_time, mod_freq_int=freqs)
    assert fs.min_frm_length == min_frm_len


def test_setup(mock, itof, system_type):

    golden_writes = {
        'group_hold': 0,
        'inte_laser_state_ir': 0,
        'apc_en': 0,
        'so_freq_en': 0,  # hw by default
        'adc_error': 4452,
        'test_bias_continue': 0,
        'ld_xemo_tdig_out': 1,
        'i_ramp_ota': 9,
        'i_ramp_bias': 99,
        'ldo_ctrl_en': 80,
        'vramp_st_lo': 43,
        'vramp_st_hi': 41,
        'vdrn_low': 42515,
        'vsg_m_bg_adjust': 13,
        'vtg_m_bg_adjust': 0,
        'ebd_size_v': 0,
        'af_vld_line': 696,
        'rwin1_l': 0x0,
        'rwin1_s': 0x0,
        'rwin2_l': 0x0,
        'rwin2_s': 0x0,
        'rwin3_l': 0x0,
        'rwin3_s': 0x0,
        'win_eb_en': 0,
        'deep_sleep_en': 0,
        'pll3_div_n_lo': 240,
        'pll3_div_n_hi': 0,
        'pll3_div_i': '1/3',
        'pll3_div_b': '1/2',
        'lane_num': '4x',
        'dphy_p0_tx_time_t_lpx': 6,
        'dphy_p0_tx_time_t_clk_prepare': 6,
        'dphy_p0_tx_time_t_clk_zero': 30,
        'dphy_p0_tx_time_t_clk_pre': 0,
        'dphy_p0_tx_time_t_hs_prepare': 5,
        'dphy_p0_tx_time_t_hs_zero': 12,
        'dphy_p0_tx_time_t_hs_sot': 0,
        'dphy_p0_tx_time_t_hs_eot': 8,
        'dphy_p0_tx_time_t_clk_eot': 8,
        'dphy_p0_tx_time_t_clk_post': 12,
        'phya_xstb_d3_config': 1,
        'phya_xstb_d2_config': 1,
        'pga_gain': 10,
        'sync_laser_lvds_mg': 42,
        'frm_num_hi': 0,
        'frm_num_lo': 1,
        'laser_high_z_idle': 0,
    }
    golden_writes['flip_v'] = 1
    golden_writes['flip_h'] = 1

    golden_bare_writes = {33: 17,
                          569: 1,
                          578: 178,
                          579: 107,
                          580: 177,
                          599: 99,
                          600: 80,
                          608: 244,
                          609: 26,
                          611: 90,
                          612: 12,
                          706: 14}

    if mock:
        actual_writes, actual_bare_writes = {}, {}
        itof.write_fields = MagicMock(
            side_effect=lambda **f: actual_writes.update(**f))
        itof._itof_spi_write = MagicMock(
            side_effect=lambda k, v: actual_bare_writes.update({k: v}))
        itof.setup()
    else:
        itof.setup()
        actual_writes = {
            k: itof.read_fields(
                k, use_mnemonic=isinstance(golden_writes[k], str))
            for k in golden_writes.keys()}
        actual_bare_writes = {k: itof._itof_spi_read(k)
                              for k in golden_bare_writes.keys()}
    assert actual_writes == golden_writes
    assert actual_bare_writes == golden_bare_writes


@pt.mark.parametrize('kwargs', [
    pt.param({'roi_rows': 480, }, id='typical full frame'),
    pt.param({'roi_rows': 20, }, id='typical roi'),
    # Not valid option RoiRows    pt.param({'roi_rows': 1, }, id='test row'),
    pt.param({'dlaser': (12, 9), }, id='change dlaser'),
    pt.param({'dummy_bursts': (10, 10), }, id='change dummy bursts'),
    pt.param({'inte_time_s': 14e-6, }, id='change inte time'),
    pt.param({'inte_time_s': (10e-6, 14e-6), }, id='specify diff inte times'),
])
def test_apply_frame_settings(mock, itof, kwargs):
    fs = itof_.FrameSettings(0, **kwargs)
    golden_writes = {
        'group_hold': 0,
        'frm_num_lo': fs.n_frames_capt.fields[0],
        'frm_num_hi': fs.n_frames_capt.fields[1],
        'mod_opt': fs.pleco_mode.kind,
        'mod_freq0_opt': fs.mod_freq_int[0].field,
        'mod_freq1_opt': fs.mod_freq_int[1].field,
        'laser_preheat_length_f0': 0,
        'laser_preheat_length_f1': 0,
        'dum_preheat_length_f0': 1,
        'dum_preheat_length_f1': 1,
        'inte_burst_length_f0': fs.inte_burst_length[0],
        'inte_burst_length_f1': fs.inte_burst_length[1],
        'inte_total_burst_length_f0': fs.inte_total_burst_length[0],
        'inte_total_burst_length_f1': fs.inte_total_burst_length[1],
        'dpulse_f0': fs.dpulse[0].field,
        'dpulse_f1': fs.dpulse[1].field,
        'npulse_f0_lo': fs.npulse[0].fields[0],
        'npulse_f0_hi': fs.npulse[0].fields[1],
        'npulse_f1_lo': fs.npulse[1].fields[0],
        'npulse_f1_hi': fs.npulse[1].fields[1],
        'dlaser_off_group_f0': fs.dlaser[0].field,
        'dlaser_off_group_f1': fs.dlaser[1].field,
        'bin_mode': 0,
        'rwin0_s': fs.start_row + 4,
        'rwin0_l': fs.roi_rows + 4,
        'cwin0_s': 8,
        'cwin0_s_div8': 8,
        'cwin0_l_div8': 640,
        'min_frm_length': fs.min_frm_length,
        'sub_frm_line_num': fs.sub_frm_line_num,
        'rd_line_max': fs.rd_line_max,
        'mipi_max_line': fs.mipi_max_line,
        'data_pix_num': fs.data_pix_num,
    }

    if mock:
        actual_writes = {}
        itof.write_fields = MagicMock(
            side_effect=lambda **f: actual_writes.update(**f))
        itof.apply_frame_settings(fs)
    else:
        itof.apply_frame_settings(fs)
        actual_writes = {k: itof.read_fields(k, use_mnemonic=isinstance(golden_writes[k], str))
                         for k in golden_writes.keys()}
    assert actual_writes == golden_writes
# pylint: disable=protected-access

@pt.mark.parametrize('check_limits', [
    pt.param(True, id='check limits before trig'),
    pt.param(False, id='do not check limits before trig'),
])
def test_soft_trigger(mock: bool, itof: itof_.Itof, check_limits: bool):
    if not mock:
        pt.skip('No easy test to know if gToF was triggered (afaik).')
    # order is important, so make sure the sequence is correct
    golden_writes = [{'so_freq_en': 1},
                     {'freq_trig': 0},
                     {'freq_trig': 1},
                     {'freq_trig': 0},
                     {'so_freq_en': 0}]
    actual_writes = []
    itof.write_fields = MagicMock(
        side_effect=lambda **f: actual_writes.append({**f}))
    itof._check_valid_timing = MagicMock()
    itof.soft_trigger(check_limits=check_limits)
    assert actual_writes == golden_writes
    if check_limits:
        assert itof._check_valid_timing.called
    else:
        assert not itof._check_valid_timing.assert_not_called()


@pt.mark.parametrize('npulse, dpulse, burst_len, error', [
    pt.param(63, 63, 37, None, id='typical'),  # 14 us
    pt.param(75, 51, 100, pt.raises(RuntimeError), id='inte time too high'),
    pt.param(75, 50, 37, pt.raises(RuntimeError), id='duty cycle too high'),
])
def test_check_valid_timing(mock: bool, itof: itof_.Itof,
                            npulse, dpulse, burst_len, error):
    """Pleco boots into an unsafe state. By default, the software trigger will
    check for this state in case the user did not properly call
     setup / apply settings / apply_frame_settings """
    if not mock and error is not None:
        pt.skip('Dangerous to run this on hardware.')
    d = {}
    itof.write_fields = MagicMock(side_effect=lambda **f: d.update(f))
    itof.write_fields(dpulse_f0=dpulse, dpulse_f1=dpulse,
                      npulse_f0_lo=npulse, npulse_f1_lo=npulse,
                      npulse_f0_hi=0, npulse_f1_hi=0,
                      inte_burst_length_f0=burst_len,
                      inte_burst_length_f1=burst_len,
                      mod_freq0_opt=itof_.ModFreqIntOv(6).field,
                      mod_freq1_opt=itof_.ModFreqIntOv(5).field,
                      dlaser_off_group_f0=12,
                      dlaser_off_group_f1=12,)

    itof.read_fields = MagicMock(side_effect=lambda f: d.get(f))
    error = error or does_not_raise()
    with error:
        itof._check_valid_timing()
