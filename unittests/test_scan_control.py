from contextlib import nullcontext as does_not_raise
import random
import time

import pytest as pt

from cobra_system_control.itof import FrameSettings, LaserMgSync
from cobra_system_control.metasurface import LM10OrderOv
from cobra_system_control.random_access_scanning import (
    InteTimeIdxMappedOv, BinningOv, FrameRateOv, MaxRangeIdxMappedOv,
    STRIPE_MODE_FLAGS, DspMode,
)
from cobra_system_control.scan_control import (
    Scan, ScanEntry, ScanTable,
    SnrThresholdBv, get_scan_time_constants,
    get_extra_scan_delay_margin_time_us,
    SCAN_TABLE_SIZE,
)
from cobra_system_control.sensor_head import make_start_stop_flag_array, SensorHead

from unittests.conftest import make_scan_init_kwargs


def verify_fifo_status(mock: bool, scan: Scan, expected_count: int):
    """Helper method used by test_scan_fifo"""
    fifo_depth = 2 ** (scan.get_size('fifo_count') - 1)
    if mock:
        scan.read_fields.side_effect = lambda f: {
            'fifo_count': expected_count,
            'fifo_empty': int(expected_count == 0),
            'fifo_full': int(expected_count == fifo_depth)
        }.get(f)
    assert scan.read_fields('fifo_count') == expected_count
    assert scan.read_fields('fifo_empty') == int(expected_count == 0)
    assert scan.read_fields('fifo_full') == int(expected_count == fifo_depth)


def test_snr():
    assert SnrThresholdBv._NFRAC == 3
    assert SnrThresholdBv.TOLERANCE == 0
    assert SnrThresholdBv.LIMITS == (0, 511.875)

    assert SnrThresholdBv(2.0).field == (2.0 / 2**(-1*SnrThresholdBv._NFRAC))
    assert SnrThresholdBv.from_field(500).value == (500 * 2**(-1 * SnrThresholdBv._NFRAC))


def test_scan_entry_instantiation(laser_ci_dac):
    # make sure all properties are accounted for in __init__
    init_kwargs, regs = make_scan_init_kwargs(laser_ci_dac)
    with does_not_raise():
        se = ScanEntry(**init_kwargs)

    # ensure the routing was correct for all __init__ args -> descriptors
    for name, _ in ScanEntry.memmap():
        assert getattr(se, name) == regs[name]


@pt.mark.parametrize(
    'rows, mod_freq, inte, hdri, binn, frate', [
        pt.param(nr, mf, inte, hdri, binn, frate)
        for nr in [8, 480]
        for mf in [25.2, 32.4]
        for inte in [8, 13, 20]
        for hdri in [5, 10, 15]
        for binn in [1, 2]
        for frate in [None, 300, 500, 960]
    ])
def test_scan_entry_build(sensor_head, rows, mod_freq, inte, hdri, binn, frate, ffuncs):
    roi_sel = random.randint(0, 2 ** ScanEntry.roi_sel.size - 1)
    order = LM10OrderOv(random.choice(LM10OrderOv.OPTIONS))
    ci_v = random.uniform(0, sensor_head.ci_max)
    hdr_ci_v = random.uniform(0, sensor_head.ci_max/2)
    ci_v_field_unshifted = sensor_head.laser_ci_dac.field_unshifted_from_voltage(ci_v)
    hdr_ci_v_field_unshifted = sensor_head.laser_ci_dac.field_unshifted_from_voltage(hdr_ci_v)

    frame = FrameSettings(
        start_row=0,
        roi_rows=rows,
        mod_freq_int=MaxRangeIdxMappedOv.MAP[mod_freq],
        inte_time_s=InteTimeIdxMappedOv(inte).mapped,
        hdr_inte_time_s=InteTimeIdxMappedOv(hdri).mapped,
    )

    flags = random.choice(range(0b11111111111111 + 1))
    virtual_sensor_bitmask = random.choice(range(0b11111111 + 1))

    # with error or does_not_raise():
    ito_phase_fields, ito_toggle_fields, tp1_fields, pol_cnt_tc, scan_fetch_delay, scan_trigger_delay = get_scan_time_constants(
        ffuncs, frame,
        binn, frate,
        None,
        (None, None),
        (None, None),
        None, None,
    )

    se = ScanEntry.build(ffuncs,
                         roi_sel,
                         order,
                         ci_v_field_unshifted,
                         hdr_ci_v_field_unshifted,
                         frame,
                         virtual_sensor_bitmask,
                         flags,
                         binn,
                         frate,
                         None,
                         (None, None),
                         (None, None),
                         None, None,
                         )

    assert se.roi_sel == roi_sel
    assert se.roi_id == order.value

    assert se.virtual_sensor_bitmask == virtual_sensor_bitmask

    assert se.start_stop_flags == flags

    assert se.min_frm_length == frame.min_frm_length

    assert se.laser_ci_hdr == hdr_ci_v_field_unshifted
    assert se.laser_ci == ci_v_field_unshifted

    assert se.npulse_group_f1 == frame.npulse[1].field
    assert se.npulse_group_f0 == frame.npulse[0].field
    assert se.mod_freq1_opt == frame.mod_freq_int[1].field
    assert se.mod_freq0_opt == frame.mod_freq_int[0].field

    assert se.sensor_mode is frame.pleco_mode.field
    assert se.rwin0_l == frame.roi_rows.field
    assert se.rwin0_s == frame.start_row.field
    assert se.dpulse_group_f1 == frame.dpulse[1].field
    assert se.dpulse_group_f0 == frame.dpulse[0].field

    assert se.sync_laser_lvds_mg == LaserMgSync(1).field
    assert se.inte_burst_length_f1 == frame.inte_burst_length[1]
    assert se.inte_burst_length_f0 == frame.inte_burst_length[0]

    assert se.inte_burst_length_f1_hdr == frame.hdr_inte_burst_length[1]
    assert se.inte_burst_length_f0_hdr == frame.hdr_inte_burst_length[0]

    assert se.steering_idx == order.field
    assert se.pol_cnt_tc_1 == pol_cnt_tc[1]
    assert se.pol_cnt_tc_0 == pol_cnt_tc[0]

    assert se.tp1_period_1 == tp1_fields[1]
    assert se.tp1_period_0 == tp1_fields[0]

    assert se.ito_phase_tc_1 == ito_phase_fields[1]
    assert se.ito_phase_tc_0 == ito_phase_fields[0]

    assert se.ito_toggle_tc_1 == ito_toggle_fields[1]
    assert se.ito_toggle_tc_0 == ito_toggle_fields[0]

    assert se.scan_fetch_delay == scan_fetch_delay
    assert se.scan_trigger_delay == scan_trigger_delay


def test_scan_entry_addr(laser_ci_dac):
    # check that we are writing to the correct memory location
    init_kwargs, regs = make_scan_init_kwargs(laser_ci_dac)
    se = ScanEntry(**init_kwargs)
    assert se.addr == regs['roi_sel'] << 6


def test_scan_entry_data(laser_ci_dac):
    # ensure roi_sel is not in the data
    init_kwargs, _ = make_scan_init_kwargs(laser_ci_dac, lambda x: 0)
    ## CI was hardcoded to something small to make sure that the laser is safe
    init_kwargs['laser_ci'] = 0
    init_kwargs['roi_sel'] = 0
    # window size was set to non-zero
    init_kwargs['rwin0_l'] = 0

    assert all(word == 0 for word in ScanEntry(**init_kwargs).data_words)

    # ensure 8 bytes of data are returned
    init_kwargs, _ = make_scan_init_kwargs(laser_ci_dac)
    se = ScanEntry(**init_kwargs)
    assert len(se.data_words) == ScanEntry.DATA_WORDS


@pt.mark.parametrize(
    'ptrs, error', [
        pt.param([*range(0, 512)], None, id='use all ptrs'),
        pt.param([*range(100, 200)], None, id='different starting ptr'),
        pt.param([*range(0, 50), *range(51, 100)], pt.raises(ValueError),
                 id='non-contiguous error'),
    ])
def test_scan_table(ptrs, error, laser_ci_dac):
    entries = []
    for ptr in ptrs:
        init_kwargs, _ = make_scan_init_kwargs(laser_ci_dac)
        init_kwargs['roi_sel'] = ptr
        entries.append(ScanEntry(**init_kwargs))

    with error or does_not_raise():
        scan_table = ScanTable(entries)
        assert scan_table.valid_ptr_range == (min(ptrs), max(ptrs))


@pt.mark.parametrize("dsp_mode", [
    pt.param(DspMode.CAMERA_MODE, id="grid"),
    pt.param(DspMode.LIDAR_MODE, id="stripe"),
])
def test_scan_table_build(laser_ci_dac, ffuncs, dsp_mode):
    orders = [LM10OrderOv(o) for o in (90, 320, 180)]
    ci_v = [v for v in (1.0, 1.5, 2.0)]
    ci_field_unshifted = [laser_ci_dac.field_unshifted_from_voltage(x) for x in ci_v]
    hdr_ci_field_unshifted = [laser_ci_dac.field_unshifted_from_voltage(x) for x in ci_v]
    frames = [FrameSettings(s, 8) for s in (0, 15, 30)]
    bitmask = [random.choice(range(0b11111111 + 1)) for _ in range(3)]
    binning = [BinningOv(2), BinningOv(2), BinningOv(2)]
    frame_rate = [FrameRateOv(300), FrameRateOv(850), FrameRateOv(960)]

    flags = make_start_stop_flag_array(None, len(orders), dsp_mode)

    table = ScanTable.build(
        field_funcs=ffuncs,
        orders=orders,
        ci_v_fields_unshifted=ci_field_unshifted,
        hdr_ci_v_fields_unshifted=hdr_ci_field_unshifted,
        frames=frames,
        virtual_sensor_bitmask=bitmask,
        binning=binning,
        frame_rate=frame_rate,
        start_stop_flags=flags,
    )

    assert len(table.scan_entries) == 3
    for i, entry in enumerate(table.scan_entries):
        assert i == entry.roi_sel
    if dsp_mode == 0:
        assert table.scan_entries[0].start_stop_flags == 0b00010001000100010001000100010001
        assert table.scan_entries[1].start_stop_flags == 0b00
        assert table.scan_entries[2].start_stop_flags == 0b00100010001000100010001000100010
    elif dsp_mode == 1:
        for entry in table.scan_entries:
            assert entry.start_stop_flags == STRIPE_MODE_FLAGS

    assert table.scan_entries[0].pol_cnt_tc_0 in [0, 1]
    assert table.scan_entries[0].pol_cnt_tc_1 == 2
    assert table.scan_entries[1].pol_cnt_tc_0 == 0
    assert table.scan_entries[1].pol_cnt_tc_1 == 2
    assert table.scan_entries[2].pol_cnt_tc_0 == 0
    assert table.scan_entries[2].pol_cnt_tc_1 == 2


@pt.mark.parametrize(
    'ram_msb', [pt.param(x) for x in [0, 1]])
def test_scan_params_valid_ptr_range(sensor_head: SensorHead, ram_msb):
    sen = sensor_head
    sen.scan_ram_msb = not ram_msb
    #assert sen.scan_params.valid_ptr_range is None

    num_ptrs = 100
    first_ptr = random.randint(0, (2 ** 9 - 1) - num_ptrs)
    all_ptrs = list(range(first_ptr, first_ptr + num_ptrs))
    random.shuffle(all_ptrs)

    entries = []
    for ptr in all_ptrs:
        init_kwargs, _ = make_scan_init_kwargs(sen.laser_ci_dac)
        init_kwargs['roi_sel'] = ptr
        entries.append(ScanEntry(**init_kwargs))

    table = ScanTable(entries)
    sen.write_scan_table(table)

    assert sen.scan_ram_msb == ram_msb
    msg = f'scan params valid ptr range:{sen.valid_scan_table_pointer_range}; table range {table.valid_ptr_range}'
    assert (sen.valid_scan_table_pointer_range[0] % 512) == table.valid_ptr_range[0], msg
    assert (sen.valid_scan_table_pointer_range[1] % 512) == table.valid_ptr_range[1], msg
    #assert scan_params.valid_ptr_range[0] == (table.valid_ptr_range[0] + ram_msb * SCAN_TABLE_SIZE), msg
    #assert scan_params.valid_ptr_range[1] == (table.valid_ptr_range[1] + ram_msb * SCAN_TABLE_SIZE), msg

    #sen.disconnect()
    #assert scan_params.valid_ptr_range is None
    sen.scan.write_fields(reset='all_reset')


@pt.mark.parametrize(
    'ram_msb', [pt.param(x) for x in [0, 1]])
def test_scan_roi_mem(
        mock: bool, sensor_head: SensorHead,
        ram_msb,
):
    """Checks scan controller for correct updates to itof, dac, and lcm"""
    if mock:
        pt.skip('integration only')
    sen = sensor_head
    # Set the scan param RAM partition
    sensor_head.scan_ram_msb = not ram_msb
    # reset scan controller
    sen.scan.write_fields(reset='all_reset')
    sen.scan.write_fields(scan_loopback=False)
    assert sen.scan.read_fields('fifo_count') == 0
    roi_cnt_begin = sen.scan.read_fields('scan_roi_cnt')

    # Make sure HDR is off
    sen.isp.write_fields(hdr_sat_limit=4095)

    # make a new scan entry
    random_init_kwargs, _ = make_scan_init_kwargs(sen.laser_ci_dac)

    entry = ScanEntry(**random_init_kwargs)
    table = ScanTable((entry,))

    # write these to the fpga
    sen.write_scan_table(table)

    assert ram_msb == sen.scan_ram_msb

    npops = random.choice(range(3, 8))
    # need to adjust the fifo_wdata by the ram_msb chosen
    for i in range(npops):
        sen.scan.write_fields(fifo_wdata=(entry.roi_sel+ram_msb*SCAN_TABLE_SIZE) & 0xFF)
        sen.scan.write_fields(fifo_wdata=(entry.roi_sel+ram_msb*SCAN_TABLE_SIZE) >> 8)
        sen.scan.write_fields(fifo_wdata=(entry.roi_sel+ram_msb*SCAN_TABLE_SIZE) & 0xFF)
        sen.scan.write_fields(fifo_wdata=(entry.roi_sel+ram_msb*SCAN_TABLE_SIZE) >> 8)
        print(f'roi cnt begin = {roi_cnt_begin}, npops={i}, scan cnt = {sen.scan.read_fields("scan_roi_cnt")}')
        time.sleep(0.5)
        print(f'during pops, read roi sel {sen.scan.read_fields("scan_params_roi_sel")}')

    assert (entry.roi_sel + 512*sen.scan_ram_msb) == sen.scan.read_fields('scan_params_roi_sel')
    msg = f'roi cnt begin = {roi_cnt_begin}, npops={npops}, scan cnt = {sen.scan.read_fields("scan_roi_cnt")}'
    assert roi_cnt_begin + npops == sen.scan.read_fields('scan_roi_cnt'), msg
    assert entry.roi_id == sen.scan.read_fields('param_roi_id')
    assert entry.laser_ci == sen.scan.read_fields('param_dac_ci')

    # itof checks
    assert sen.itof.read_fields('mod_opt') == entry.sensor_mode
    assert sen.itof.read_fields('mod_freq0_opt') == entry.mod_freq0_opt
    assert sen.itof.read_fields('mod_freq1_opt') == entry.mod_freq1_opt
    assert (sen.itof.read_fields('npulse_f0_lo')
            + (sen.itof.read_fields('npulse_f0_hi') << 8)
            == entry.npulse_group_f0)
    assert (sen.itof.read_fields('npulse_f1_lo')
            + (sen.itof.read_fields('npulse_f1_hi') << 8)
            == entry.npulse_group_f1)
    assert sen.itof.read_fields('dpulse_f0') == entry.dpulse_group_f0
    assert sen.itof.read_fields('dpulse_f1') == entry.dpulse_group_f1
    assert sen.itof.read_fields('rwin0_s') == entry.rwin0_s
    assert sen.itof.read_fields('rwin0_l') == entry.rwin0_l
    assert sen.itof.read_fields('bin_mode') == 0
    assert sen.itof.read_fields(
        'inte_burst_length_f0') == entry.inte_burst_length_f0
    assert sen.itof.read_fields(
        'inte_burst_length_f1') == entry.inte_burst_length_f1
    assert sen.itof.read_fields('min_frm_length') == entry.min_frm_length

    # delays and nov_sel checks
    assert sen.itof.read_fields('sync_laser_lvds_mg') == entry.sync_laser_lvds_mg

    # dac spi check
    assert entry.laser_ci == sen.laser_ci_dac.dac.dac_read(sen.laser_ci_dac.chan_idx)

    sen.scan.write_fields(reset='all_reset')


def test_scan_fifo(mock: bool, scan: Scan):
    if mock:
        pt.skip('Integration only')
    scan.write_fields(scan_hold_at_idle=1)
    scan.write_fields(reset='all_reset')

    # check initial conditions
    verify_fifo_status(mock, scan, expected_count=0)

    # fill fifo
    wdata = []
    fifo_depth = 2 ** (scan.get_size('fifo_count') - 1)
    fifo_wdata_size = scan.get_size('fifo_wdata')
    for i in range(fifo_depth):
        wd = random.randint(0, 2 ** fifo_wdata_size - 1)
        scan.write_fields(fifo_wdata=wd)
        wdata.append(wd)
        gold_count = i + 1
        verify_fifo_status(mock, scan, expected_count=gold_count)

    # drain fifo
    for i, wd in enumerate(wdata):
        scan.write_fields(fifo_ren_override=1)
        assert scan.read_fields('fifo_rdata') == wd
        gold_count = fifo_depth - 1 - i
        verify_fifo_status(mock, scan, expected_count=gold_count)

    # check fifo reset
    scan.write_fields(fifo_wdata=0xff)
    verify_fifo_status(mock, scan, expected_count=1)
    scan.write_fields(reset='fifo_reset')
    verify_fifo_status(mock, scan, expected_count=0)

    # remove hold and reset
    scan.write_fields(scan_hold_at_idle=0)  # just in case

    assert scan.read_fields('scan_hold_at_idle') == 0
    scan.write_fields(reset='all_reset')


def make_md_init_kwargs(
        metadata, func=lambda r: random.randint(0, 2**r.size - 1)):
    regs = {name: func(reg) for name, reg in metadata.memmap()}
    init_kwargs = regs.copy()
    return init_kwargs, regs


@pt.mark.parametrize('inte_time_us, at_least', [
    pt.param(5, 17),
    pt.param(6, 11),
    pt.param(7, 9),
    pt.param(8, 8),
    pt.param(9, 3),
    pt.param(10, 0),
])
def test_get_extra_scan_delay_margin_time(inte_time_us, at_least):
    tup = (inte_time_us * 1e-6, inte_time_us * 1e-6)
    extra = get_extra_scan_delay_margin_time_us(tup)
    assert extra >= at_least
