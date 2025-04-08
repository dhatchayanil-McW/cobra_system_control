# pylint: disable=protected-access
import dataclasses as dc
import inspect
import random
import time
from contextlib import nullcontext as does_not_raise
from unittest.mock import MagicMock

import numpy as np
import pytest as pt

from cobra_system_control.calibration_data import CalData
import cobra_system_control.exceptions as cobex
import cobra_system_control.fe_ctl as fe_ctl
from cobra_system_control.metadata import StaticMetadata, PerVirtualSensorMetadata
from cobra_system_control.metasurface import (
    LcmController, LcmAssembly)
from cobra_system_control.numerical_utilities import SignalVec, btop_raw12
from cobra_system_control.random_access_scanning import NnLevelOv
from cobra_system_control.scan_control import BinningOv, SnrThresholdBv
from cobra_system_control.sensor_head import SensorHead, DEFAULT_ROI_ROWS
from cobra_system_control.spi_flash import (
    pages_from_size, reads_from_size, sectors_from_size)
from cobra_system_control.state import State, StateError

from unittests.test_random_access_scanning import (
    ras_inputs, ras_errors, ras_arg_tests)


def get_default_args(func):
    signature = inspect.signature(func)
    return {
        k: v.default
        for k, v in signature.parameters.items()
        if v.default is not inspect.Parameter.empty
    }


TXN_ERROR = pt.raises(StateError)


@pt.mark.usefixtures('mock_only')
@pt.mark.parametrize('txn, en_state, expected', [
    pt.param(lambda s: s.connect(), State.INITIALIZED, State.CONNECTED),
    pt.param(lambda s: s.connect(), State.CONNECTED, State.CONNECTED),
    pt.param(lambda s: s.connect(), State.READY, TXN_ERROR),
    pt.param(lambda s: s.connect(), State.ENERGIZED, TXN_ERROR),
    pt.param(lambda s: s.connect(), State.SCANNING, TXN_ERROR),

    pt.param(lambda s: s.setup(), State.INITIALIZED, TXN_ERROR),
    pt.param(lambda s: s.setup(), State.CONNECTED, State.READY),
    pt.param(lambda s: s.setup(), State.READY, State.READY),
    pt.param(lambda s: s.setup(), State.ENERGIZED, TXN_ERROR),
    pt.param(lambda s: s.setup(), State.SCANNING, TXN_ERROR),

    pt.param(lambda s: s.apply_settings(orders=0, s_rows=0), State.INITIALIZED, TXN_ERROR),
    pt.param(lambda s: s.apply_settings(orders=0, s_rows=0), State.CONNECTED, State.CONNECTED),
    pt.param(lambda s: s.apply_settings(orders=0, s_rows=0), State.READY, State.READY),
    pt.param(lambda s: s.apply_settings(orders=0, s_rows=0), State.ENERGIZED, State.ENERGIZED),
    pt.param(lambda s: s.apply_settings(orders=0, s_rows=0), State.SCANNING, State.ENERGIZED),

    pt.param(lambda s: s.start(), State.INITIALIZED, TXN_ERROR),
    pt.param(lambda s: s.start(), State.CONNECTED, TXN_ERROR),
    pt.param(lambda s: s.start(), State.READY, TXN_ERROR),
    pt.param(lambda s: s.start(), State.ENERGIZED, State.SCANNING),
    pt.param(lambda s: s.start(), State.SCANNING, State.SCANNING),

    pt.param(lambda s: s.write_scan_fifo(0, 0), State.INITIALIZED, TXN_ERROR),
    pt.param(lambda s: s.write_scan_fifo(0, 0), State.CONNECTED, TXN_ERROR),
    pt.param(lambda s: s.write_scan_fifo(0, 0), State.READY, TXN_ERROR),
    pt.param(lambda s: s.write_scan_fifo(0, 0), State.ENERGIZED, State.SCANNING),
    pt.param(lambda s: s.write_scan_fifo(0, 0), State.SCANNING, State.SCANNING),

    pt.param(lambda s: s.stop(), State.INITIALIZED, State.INITIALIZED),
    pt.param(lambda s: s.stop(), State.CONNECTED, State.CONNECTED),
    pt.param(lambda s: s.stop(), State.READY, State.READY),
    pt.param(lambda s: s.stop(), State.ENERGIZED, State.ENERGIZED),
    pt.param(lambda s: s.stop(), State.SCANNING, State.ENERGIZED),

    pt.param(lambda s: s.enable(), State.INITIALIZED, TXN_ERROR),
    pt.param(lambda s: s.enable(), State.CONNECTED, TXN_ERROR),
    pt.param(lambda s: s.enable(), State.READY, State.ENERGIZED),
    pt.param(lambda s: s.enable(), State.ENERGIZED, State.ENERGIZED),
    pt.param(lambda s: s.enable(), State.SCANNING, TXN_ERROR),
])
def test_sensor_head_txns(sensor_head: SensorHead,
                          txn, en_state, expected):
    # put the sensor_head in the correct starting state
    returns = {'fifo_count': 0,
               'scan_loopback': True,
               'scan_state': 'idle',
               'sensor_id': random.choice(range(100)),
               }
    sensor_head.scan.read_fields = MagicMock(
        side_effect=lambda k, **kwargs: returns.get(k))
    sensor_head.scan.wait_for_scan_idle = MagicMock()
    sensor_head.spi_flash.qspi.fast_read_data.return_value = bytearray([0x00] * 1024)
    sensor_head.load_cal_data = MagicMock()
    sensor_head.isp.read_fields = MagicMock(
        side_effect=lambda k, **kwargs: returns.get(k))
    fe_ctl.fe_send = MagicMock()
    sensor_head.scan_ram_msb = 1

    if en_state in (State.CONNECTED, State.READY, State.ENERGIZED, State.SCANNING):
        sensor_head.connect()
    if en_state in (State.READY, State.ENERGIZED, State.SCANNING):
        sensor_head.setup()
    if en_state in (State.ENERGIZED, State.SCANNING):
        sensor_head.enable()
    if en_state in (State.SCANNING, ):
        returns['scan_state'] = 'not-idle'
        sensor_head.start()
    assert sensor_head.state is en_state

    if isinstance(expected, State):
        if expected is not State.SCANNING:
            returns['scan_state'] = 'idle'
        txn(sensor_head)
        if expected is State.SCANNING:
            returns['scan_state'] = 'not-idle'

        assert sensor_head.state is expected
    else:
        with expected:
            txn(sensor_head)
    sensor_head.stop()


def test_sensor_head_setup():
    pt.skip('Nail down after scan control works reliably across a few units')


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize("angles, nrows, error", [
    pt.param([0], 6, None),
    pt.param([0], 8, None),
    pt.param([0], 20, None),
    pt.param([0], 20, None),
    pt.param([0], 480, None),
])
def test_sensor_head_apply_settings(sensor_head, angles, nrows, error):
    with error or does_not_raise():
        sensor_head.apply_settings(angles=angles, roi_rows=nrows)


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize("angles, freqs, error", [
    pt.param([0], (8, 7), None),
    pt.param([0], [(9, 8)], None),
    pt.param([0, 1], [(9, 8), (9, 8)], None),
    pt.param([0, 1], [(8, 7), (9, 8)], pt.raises(cobex.ScanPatternValueError)),
])
def test_apply_settings_mixed_freqs(sensor_head, angles, freqs, error):
    with error or does_not_raise():
        sensor_head.apply_settings(angles=angles, mod_freq_int=freqs)


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize('use_sm', [
    pt.param(True),
    pt.param(False),
    ])
def test_apply_settings_with_static_metadata(sensor_head, use_sm):
    sensor_head.stop()
    angles = random.choice(range(-40, 40))
    testm = random.choice(range(3))
    rtd = random.choice(range(2))
    hdrt = random.choice(range(4096))
    reducem = 1
    qm = 0
    mipim = 1

    if use_sm:
        sn = random.choice(range(1200))
        system_type = random.choice(range(4096))
        rx_pcb_type = random.choice(range(4096))
        tx_pcb_type = random.choice(range(4096))
        lcm_type = random.choice(range(4096))
        range_cal_offset_mm_lo_0807 = random.choice(range(4096))
        range_cal_offset_mm_hi_0807 = random.choice(range(4096))
        range_cal_mm_per_volt_lo_0807 = random.choice(range(4096))
        range_cal_mm_per_volt_hi_0807 = random.choice(range(4096))
        range_cal_mm_per_celsius_lo_0807 = random.choice(range(4096))
        range_cal_mm_per_celsius_hi_0807 = random.choice(range(4096))
        range_cal_mm_per_volt_lo_0908 = random.choice(range(4096))
        range_cal_mm_per_volt_hi_0908 = random.choice(range(4096))
        range_cal_mm_per_celsius_lo_0908 = random.choice(range(4096))
        range_cal_mm_per_celsius_hi_0908 = random.choice(range(4096))
        range_cal_offset_mm_lo_0908 = random.choice(range(4096))
        range_cal_offset_mm_hi_0908 = random.choice(range(4096))

        adc_cal_gain = random.choice(range(4096))
        adc_cal_offset = random.choice(range(4096))
    else:
        sn = sensor_head.sensor_sn
        system_type = 3

        rx_pcb_type = sensor_head.rx_pcb_rev
        tx_pcb_type = 0
        lcm_type = 2

        if sensor_head.cal_data.range_tmp.is_valid:
            # WARN all these values are shifted by 4 to fit into 12bits until we have a new FPGA image.
            range_cal_offset_mm_0807 = sensor_head.cal_data.range_tmp.rng_offset_mm_0807.vdig[0]
            range_cal_offset_mm_lo_0807 = range_cal_offset_mm_0807 & 0xfff
            range_cal_offset_mm_hi_0807 = (range_cal_offset_mm_0807 >> 12) & 0xf

            range_cal_mm_per_volt_0807 = sensor_head.cal_data.range_tmp.mm_per_volt_0807.vdig[0]
            range_cal_mm_per_volt_lo_0807 = range_cal_mm_per_volt_0807 & 0xfff
            range_cal_mm_per_volt_hi_0807 = (range_cal_mm_per_volt_0807 >> 12) & 0xf

            range_cal_mm_per_celsius_0807 = sensor_head.cal_data.range_tmp.mm_per_celsius_0807.vdig[0]
            range_cal_mm_per_celsius_lo_0807 = range_cal_mm_per_celsius_0807 & 0xfff
            range_cal_mm_per_celsius_hi_0807 = (range_cal_mm_per_celsius_0807 >> 12) & 0xf

            range_cal_offset_mm_0908 = sensor_head.cal_data.range_tmp.rng_offset_mm_0908.vdig[0]
            range_cal_offset_mm_lo_0908 = range_cal_offset_mm_0908 & 0xfff
            range_cal_offset_mm_hi_0908 = (range_cal_offset_mm_0908 >> 12) & 0xf

            range_cal_mm_per_volt_0908 = sensor_head.cal_data.range_tmp.mm_per_volt_0908.vdig[0]
            range_cal_mm_per_volt_lo_0908 = range_cal_mm_per_volt_0908 & 0xfff
            range_cal_mm_per_volt_hi_0908 = (range_cal_mm_per_volt_0908 >> 12) & 0xf

            range_cal_mm_per_celsius_0908 = sensor_head.cal_data.range_tmp.mm_per_celsius_0908.vdig[0]
            range_cal_mm_per_celsius_lo_0908 = range_cal_mm_per_celsius_0908 & 0xfff
            range_cal_mm_per_celsius_hi_0908 = (range_cal_mm_per_celsius_0908 >> 12) & 0xf

        else:
            range_cal_offset_mm_lo_0807 = 0
            range_cal_offset_mm_hi_0807 = 0
            range_cal_mm_per_volt_lo_0807 = 0
            range_cal_mm_per_volt_hi_0807 = 0
            range_cal_mm_per_celsius_lo_0807 = 0
            range_cal_mm_per_celsius_hi_0807 = 0
            range_cal_offset_mm_lo_0908 = 0
            range_cal_offset_mm_hi_0908 = 0
            range_cal_mm_per_volt_lo_0908 = 0
            range_cal_mm_per_volt_hi_0908 = 0
            range_cal_mm_per_celsius_lo_0908 = 0
            range_cal_mm_per_celsius_hi_0908 = 0

        adc_cal_gain_fmt = SignalVec(False, 12, 19)
        adc_cal_gain_fmt.set_float_vec(sensor_head.fpga_adc.cal_gain)
        adc_cal_gain = adc_cal_gain_fmt.get_dig_vec()[0]
        adc_cal_offset_fmt = SignalVec(True, 12, 14)
        adc_cal_offset_fmt.set_float_vec(sensor_head.fpga_adc.cal_offset)
        adc_cal_offset = adc_cal_offset_fmt.get_dig_vec()[0]

    sm = StaticMetadata(
        rtd_output=rtd,
        reduce_mode=reducem,
        sensor_sn=sn,
        quant_mode=qm,
        mipi_raw_mode=mipim,
        test_mode=testm,
        hdr_threshold=hdrt,
        system_type=system_type,
        rx_pcb_type=rx_pcb_type,
        tx_pcb_type=tx_pcb_type,
        lcm_type=lcm_type,

        range_cal_offset_mm_lo_0807=range_cal_offset_mm_lo_0807,
        range_cal_offset_mm_hi_0807=range_cal_offset_mm_hi_0807,
        range_cal_mm_per_volt_lo_0807=range_cal_mm_per_volt_lo_0807,
        range_cal_mm_per_volt_hi_0807=range_cal_mm_per_volt_hi_0807,
        range_cal_mm_per_celsius_lo_0807=range_cal_mm_per_celsius_lo_0807,
        range_cal_mm_per_celsius_hi_0807=range_cal_mm_per_celsius_hi_0807,
        range_cal_offset_mm_lo_0908=range_cal_offset_mm_lo_0908,
        range_cal_offset_mm_hi_0908=range_cal_offset_mm_hi_0908,
        range_cal_mm_per_volt_lo_0908=range_cal_mm_per_volt_lo_0908,
        range_cal_mm_per_volt_hi_0908=range_cal_mm_per_volt_hi_0908,
        range_cal_mm_per_celsius_lo_0908=range_cal_mm_per_celsius_lo_0908,
        range_cal_mm_per_celsius_hi_0908=range_cal_mm_per_celsius_hi_0908,

        adc_cal_gain=adc_cal_gain,
        adc_cal_offset=adc_cal_offset,
    )
    print('sm user tag', sm.random_scan_table_tag)

    if use_sm:
        sensor_head.apply_settings(angles=angles, static_metadata=sm)
    else:
        sensor_head.apply_settings(
            angles=angles,
            test_mode=testm,
            disable_network_stream=rtd,
            hdr_threshold=hdrt,
        )
    rdata = sensor_head.metabuff.get_static_buffer_data()

    if use_sm:
        assert list(sm.data_words) == list(rdata)
    else:
        # Don't take the last two entries which have the randomized tag
        assert list(sm.data_words)[0:-3] == list(rdata)[0:-3], f'{list(sm.data_words)}, {rdata}'


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize('use_fm', [
    pt.param(True),
    pt.param(False),
    ])
def test_apply_settings_with_virtual_sensor_metadata(sensor_head, use_fm):
    sensor_head.stop()
    ut = random.randint(0, 2**PerVirtualSensorMetadata.user_tag.size-1)
    binning = random.choice(BinningOv.OPTIONS)

    algo_comm = random.choice(range(1024))
    algo_grid = random.choice(range(1024))
    algo_stripe = random.choice(range(1024))
    snr = random.uniform(0, 11)
    nn = random.choice(NnLevelOv.OPTIONS)

    order = [random.choice(range(100, 400))]
    srow = [random.choice(range(400))]
    roi_rows = min(480, DEFAULT_ROI_ROWS + 4)

    #roi_rows = min(480, max(srow) - min(srow) + max(roi_rows) + 4)
    fm = PerVirtualSensorMetadata.empty_array()
    fm[0] = PerVirtualSensorMetadata.build(
        ut, BinningOv(binning),
        max(0, min(srow) - 2),
        roi_rows, 1,
        algo_comm, algo_grid, algo_stripe,
        SnrThresholdBv(snr),
        NnLevelOv(nn))

    if use_fm:
        sensor_head.apply_settings(orders=order, s_rows=srow,
                                   virtual_sensor_metadata=fm)
    else:
        sensor_head.apply_settings(orders=order, s_rows=srow,
                                   user_tag=ut, binning=binning,
                                   rtd_algorithm_common=algo_comm,
                                   rtd_algorithm_grid_mode=algo_grid,
                                   rtd_algorithm_stripe_mode=algo_stripe,
                                   snr_threshold=snr,
                                   nn_level=nn)

    for i, f in enumerate(fm):
        rdata = sensor_head.metabuff.get_virtual_sensor_buffer_data(i)
        # if virtual sensor metadata is automatically generated, some of the
        # items in the list will not match
        if use_fm:
            assert list(f.data_words) == list(rdata), f'index {i}, {list(f.data_words)}, {list(rdata)}'
        else:
            assert list(f.data_words)[0:15] == list(rdata)[0:15], f'index {i}, {list(f.data_words)}, {list(rdata)}'
            assert list(f.data_words)[16] == list(rdata)[16], f'index {i}, {list(f.data_words)}, {list(rdata)}'
            assert list(f.data_words)[18::] == list(rdata)[18::], f'index {i}, {list(f.data_words)}, {list(rdata)}'


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize(ras_inputs, [
    *ras_errors,
    *ras_arg_tests,
    ])
def test_apply_random_access_scan_settings(
        virtual_sensor_trip, fps, power, inte, mrange, binning, tag,
        nn, snr, algo_comm, algo_grid, algo_stripe,
        frate, rfrate,
        rangles, rpower, rinte, rfreq_ints,
        rbin,
        rtag, rtotal_roi, rflags, rvirtual_sensor_bitmask, error,
        sensor_head,
):
    with error or does_not_raise():
        sensor_head.apply_random_access_scan_settings(
            angle_range=virtual_sensor_trip,
            fps_multiple=fps,
            laser_power_percent=power,
            inte_time_us=inte,
            max_range_m=mrange,
            binning=binning,
            user_tag=tag,
            snr_threshold=snr,
            nn_level=nn,
            rtd_algorithm_common=algo_comm,
            rtd_algorithm_grid_mode=algo_grid,
            rtd_algorithm_stripe_mode=algo_stripe,
            frame_rate_hz=frate,
        )


def test_sensor_head_start():
    pt.skip('Nail down after scan control works reliably across a few units')


def test_sensor_head_write_scan_fifo():
    pt.skip('Nail down after scan control works reliably across a few units')


def test_sensor_head_stop():
    pt.skip('Nail down after scan control works reliably across a few units')


def test_sensor_head_disconnect():
    pt.skip('Nail down after scan control works reliably across a few units')


def test_serial_number(mock, sensor_head, cal_data):
    if mock:
        sensor_head._read_cal = MagicMock()
        span = cal_data.info.group_type.sensor_sn.addr_offset
        cdata = cal_data.empty()
        cdata.ba[span + 1] = 0xba
        sensor_head._read_cal.return_value = cdata
    else:
        cdata = sensor_head.get_cal_data()
    sn = cdata.info.sensor_sn.vdig[0]
    if mock:
        assert sn == 0xba
    else:
        # setup() called in conftest so caldata was loaded even though the
        # check is overridden at the top of this test
        op0 = sn == 0xff
        op1 = sn == 0x00
        op2 = sn == sensor_head.get_cal_data().info.sensor_sn.vdig[0]
        assert op0 or op1 or op2


@pt.mark.skip('Not loading cal from disk currently')
def test_load_cal_data_from_disk(mock, sensor_head):
    if not mock:
        data = sensor_head.load_cal_data_from_disk()

        assert isinstance(data, (CalData,))
        for d, c in zip(sorted(dc.asdict(data).items()),
                        sorted(dc.asdict(sensor_head.cal_data).items())):

            assert d[0] == c[0]
            if isinstance(c[1], dict):
                if len(c[1]['_vfxp']) > 1:
                    for (dk, dv), (ck, cv) in zip(d[1].items(), c[1].items()):
                        assert dk == ck
                        if isinstance(cv, bytearray):
                            cv = list(cv)
                        if isinstance(dv, (np.ndarray, np.generic)):
                            np.testing.assert_array_equal(dv, cv)
                        else:
                            assert dv == cv


@pt.mark.usefixtures('mock_only')
def test_write_cal_data(mock, sensor_head, cal_data):
    if not mock:
        pt.skip("Don't overwrite the spi flash.")
    sensor_head._write_cal(cal_data.empty())

    erase_call_count = sensor_head.spi_flash.qspi.sector_erase.call_count
    num_sectors = sectors_from_size(cal_data.size_bytes())

    msg0 = (f'sector erase called '
            f'{erase_call_count} times, '
            f'num sectors = {num_sectors}')
    assert erase_call_count == num_sectors, msg0

    page_program_call_count = sensor_head.spi_flash.qspi.page_program.call_count
    num_pages = pages_from_size(cal_data.size_bytes())

    msg1 = (f'page program called '
            f'{page_program_call_count} times, '
            f'num pages = {num_pages}')
    assert page_program_call_count == num_pages, msg1


def test_read_cal_data(mock: bool, sensor_head: SensorHead, cal_data):
    if mock:
        sensor_head.spi_flash.qspi.fast_read_data.return_value = (
            bytearray([0xaf] * 1024))
    cdata = sensor_head._read_cal(cal_data)
    assert len(cdata.ba) == cal_data.size_bytes()
    if mock:
        for byte in cdata.ba:
            assert byte == 0xaf

        read_data_call_count = sensor_head.spi_flash.qspi.fast_read_data.call_count
        num_reads = reads_from_size(cal_data.size_bytes())
        msg0 = (f'fast read data called '
                f'{read_data_call_count} times, '
                f'num reads = {num_reads}')
        assert read_data_call_count == num_reads, msg0


@pt.mark.skip("Tested with the test_mipi tests")
def test_itof_fpga_mipi(mock: bool, sensor_head: SensorHead, lcm_ctrl: LcmController):
    """Checks scan controller for correct updates to itof"""
    nlines = 120
    if mock:
        pt.skip('integration only')
    lcm_ctrl.enable()
    sensor_head.scan.write_fields(reset='all_reset')
    time.sleep(1)
    sensor_head.stop()
    time.sleep(0.4)
    sensor_head.apply_settings(orders=[0], s_rows=[1], loopback=False)
    time.sleep(0.1)
    sensor_head.start()

    done = False
    max_tries = 20
    idx = 0
    while not done:
        sensor_head.isp.write_fields(pkt_fifo_en=0)
        time.sleep(0.1)
        sensor_head.isp.write_fields(pkt_fifo_en=1)
        time.sleep(0.1)
        sensor_head.write_scan_fifo(0, 0)
        time.sleep(1)
        fifo_count = sensor_head.isp.read_fields("mipi_rx_pkt_fifo_count")
        done = fifo_count == nlines + 1
        time.sleep(0.1)
        idx += 1
        if idx == max_tries:
            pt.xfail('Sometimes getting the values into mipi '
                     'fails even if the real test_mipis pass')

    empty = sensor_head.isp.read_fields("mipi_rx_pkt_fifo_empty")
    idx = 0
    while empty != 1:
        sensor_head.isp.write_fields(pkt_fifo_pop=1)
        empty = sensor_head.isp.read_fields("mipi_rx_pkt_fifo_empty")
        count = sensor_head.isp.read_fields("mipi_rx_pkt_fifo_count")
        dt = sensor_head.isp.read_fields("mipi_rx_pkt_fifo_dt")
        wc_lo = sensor_head.isp.read_fields("mipi_rx_pkt_fifo_wc_lo")
        wc_hi = sensor_head.isp.read_fields("mipi_rx_pkt_fifo_wc_hi")
        wc = (wc_hi << 8) | wc_lo
        #print('empty?', empty, count, dt, wc_lo, wc_hi, wc)

        if idx == 0:
            assert dt == 0x00
            assert wc == 0
            assert count == nlines
        elif idx == 121:
            assert dt == 0x01
            assert wc == 0
        else:
            assert dt == 0x2c
            assert wc == 2880
            assert count == nlines - idx
        idx += 1
    assert idx == nlines + 1  # extra +=1 on idx on the last pop
    sensor_head.isp.write_fields(pkt_fifo_en=0)
    sensor_head.stop()
    lcm_ctrl.disable()


@pt.mark.usefixtures('integration_only')
def test_properties(sensor_head):
    sensor_head.apply_random_access_scan_settings(
        angle_range=[(-1, 1, 1)], fps_multiple=1, laser_power_percent=50,
        inte_time_us=5, max_range_m=25.2, user_tag=0x0)
    assert sensor_head.cal_data == sensor_head._cal_data
    assert sensor_head.itof == sensor_head._itof
    assert sensor_head.laser_ci_dac == sensor_head._laser_ci_dac
    assert sensor_head.cmb_laser_vlda_dac == sensor_head._cmb_laser_vlda_dac
    assert sensor_head.cmb_lcm_v_dac == sensor_head._cmb_lcm_v_dac
    assert sensor_head.sh_laser_vlda_dac == sensor_head._sh_laser_vlda_dac
    assert sensor_head.ito_dac == sensor_head._ito_dac
    assert sensor_head.lcm_ctrl == sensor_head._lcm_ctrl
    assert sensor_head.debug == sensor_head._debug
    assert sensor_head.isp == sensor_head._isp
    assert sensor_head.metabuff == sensor_head._metabuff
    assert sensor_head.spi_flash == sensor_head._spi_flash
    assert sensor_head.fpga_adc == sensor_head._fpga_adc
    assert sensor_head.scan == sensor_head._scan
    assert sensor_head.scan_params == sensor_head._scan_params
    assert sensor_head.pixel_mapping == sensor_head._pixel_mapping
    assert sensor_head.super_pixel_mapping == sensor_head._super_pixel_mapping
    assert sensor_head.roi_mapping == sensor_head._roi_mapping
    assert sensor_head.random_access_scan == sensor_head._random_access_scan
    assert sensor_head.cal_data_path == sensor_head._cal_data_path
    assert sensor_head.mapping_table_path == sensor_head._mapping_table_path


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize('ito_v', [
    (9), (-9),
])
def test_setup_ito_v(ito_v, sensor_head):
    # copy logic from setup()
    sensor_head._lcm_assembly = LcmAssembly()
    sensor_head.ito_dac.set_voltage(ito_v)
    rito = sensor_head.ito_dac.get_voltage()
    assert ito_v == pt.approx(rito, abs=0.05)


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize('is_disabled', [
    pt.param(False),
    pt.param(True),
])
def test_disable_range_temp_bit(sensor_head, is_disabled):

    sensor_head.apply_settings(angles=[0], disable_range_temp_correction=is_disabled)
    rdata_bytes = sensor_head.metabuff.get_virtual_sensor_buffer_data(0)
    rdata_pixels = btop_raw12(rdata_bytes)
    rtd_alg = rdata_pixels[6]
    range_temp_cal = (rtd_alg >> 3) & 0x1
    assert range_temp_cal == int(not is_disabled)


def test_is_compatible_calibration_version(sensor_head):
    assert sensor_head.is_compatible_calibration_version(sensor_head.cal_data)
