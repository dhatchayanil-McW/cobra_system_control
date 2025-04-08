"""
"""
import glob
import os
import time

import numpy as np
import pytest as pt

from cobra_system_control import fe_ctl
from cobra_system_control.numerical_utilities import btop_raw12
from cobra_system_control.sensor_head import SensorHead


def get_timestamp_coarse_fine_from_list(tstamp_list: list) -> tuple:
    t = tstamp_list
    tstamp_fine = t[0] + (t[1] << 12) + (t[2] << 24)
    tstamp_fine &= 0xffffffff

    tstamp_coarse = t[2] + (t[3] << 12) + (t[4] << 24) + (t[5] << 36) + (t[6] << 48)
    tstamp_coarse >>= 8
    return tstamp_coarse, tstamp_fine


@pt.mark.usefixtures('integration_only')
@pt.mark.parametrize(
    'nrows, test_mode',
    [
        pt.param(nr, tm)
        for nr in [480, 8]  # removed 6
        for tm in range(1, 4)
    ])
def test_mipi_test_patterns(
        sensor_head: SensorHead,
        nrows: int,
        test_mode: int,
):
    """Checks the test patterns emitted from the FPGA. Note that this test is
    actually quite encompassing since it requires all the normal functionality
    of the system, especially from the FPGA scan controller, Pleco, FPGA ISP,
    GMSL, and Jetson front-end components. That is, when test patterns are
    enabled, the only effect is that the test pattern data are substitued for
    the MIPI data values received from the PLECO.
    """
    # Remove old files
    os.system('rm -f /run/lumotive/cobra*.bin')

    sensor_head.scan.write_fields(rowcal_adjust_en=0)

    # Aliases and constants
    sen = sensor_head
    n_lines = nrows
    n_pixels = 640
    n_taps = 3

    sen.stop()
    time.sleep(5)
    sen.apply_settings(
        orders=[0x123],
        s_rows=[0],
        ci_v=0,
        roi_rows=n_lines,
        loopback=True,
        test_mode=test_mode,
        rtd_algorithm_common=0b1100,
        rtd_algorithm_grid_mode=0b000,
        rtd_algorithm_stripe_mode=0b00,
        disable_rawtodepth=1,
    )

    # Configure the timestamps
    sen.scan.write_fields(scan_tstamp_enable=1)
    sen.scan.write_fields(tstamp_sync_aux_en=1)
    sen.scan.write_fields(tstamp_trigger_ext_en=0)

    # Get scan table information to calculate the timestamp
    # differential later.
    pol_cnt0 = sen.scan_params.scan_table.scan_entries[0].pol_cnt_tc_0
    pol_cnt1 = sen.scan_params.scan_table.scan_entries[0].pol_cnt_tc_1
    tp1_period_0 = sen.scan_params.scan_table.scan_entries[0].tp1_period_0
    tp1_period_1 = sen.scan_params.scan_table.scan_entries[0].tp1_period_1
    roi_time_us = (
        (pol_cnt0+1) * sen.fpga_field_funcs.getv_tp1_period(tp1_period_0, 0) * 2
        + (pol_cnt1+1) * sen.fpga_field_funcs.getv_tp1_period(tp1_period_1, 0) * 2
    )
    scan_fetch_delay = sen.scan_params.scan_table.scan_entries[0].scan_fetch_delay
    scan_trigger_delay = sen.scan_params.scan_table.scan_entries[0].scan_trigger_delay

    rm = sen.isp.read_fields('reduce_mode')
    tm = sen.isp.read_fields('test_mode')
    qm = sen.isp.read_fields('quant_mode')

    reduce_mode = rm
    assert sen.isp.read_fields('roi_aggreg_cnt') == 0
    if n_lines == 480:
        assert rm == 0
    else:
        assert rm == 1
    assert test_mode == tm
    assert qm == 0

    n_subframes = 6 if rm == 0 else 2

    mipi_raw_mode = sen.isp.read_fields('tx_raw16_en')
    assert mipi_raw_mode == 1
    assert sen.isp.read_fields('tx_swap_bytes') == 1
    assert sen.isp.read_fields('tx_raw16_as_rgb888') == 1

    # Special sen.start() so we can wait for the frontend
    # to be ready before starting scanning
    sen.scan.write_fields(scan_lmmi_meta_en=0)
    mode = fe_ctl.fe_get_mode(sen._fe_rows, sen._fe_reduce_mode, sen.aggregate)

    fe_ctl.fe_start_streaming(mode)
    time.sleep(6)

    sen.write_scan_fifo(*sen.scan_params.valid_ptr_range)

    #sen.start()
    time.sleep(10)
    sen.stop()

    # give the frontend time to save the files
    time.sleep(10)

    # # Parse the raw image file from the Frontend
    fids = sorted(glob.glob('/run/lumotive/cobra*.bin'))
    assert len(fids) > 0

    # Use the previous set of aggregated ROIs to avoid tearing
    # Don't choose a wrong index
    # We'll grab two so we can do the timestamp differnce
    next_try = -10
    while True:
        try:
            fimg = np.fromfile(fids[next_try], dtype=np.uint16)
            fimg2 = np.fromfile(fids[next_try+1], dtype=np.uint16)
            assert fimg.shape[0] == (nrows
                                     * n_subframes
                                     * n_pixels
                                     * n_taps
                                     + n_pixels
                                     * n_taps), f'shape is {fimg.shape[0]}'
        except (IndexError, AssertionError):
            next_try += 1
        else:
            break


    fimg &= 0xfffc if (mipi_raw_mode != 0) else 0xfff0
    fmeta = list(fimg[0:n_pixels * n_taps].copy() >> 4)
    fraw = np.reshape(
        fimg[n_pixels * n_taps:],
        newshape=(n_subframes, -1, n_pixels, n_taps)
    )
    fimg2 &= 0xfffc if (mipi_raw_mode != 0) else 0xfff0
    fmeta2 = list(fimg2[0:n_pixels * n_taps].copy() >> 4)

    # Ensure all the metadata matches between the saved ROI and
    # the metadata buffer.

    # The bus times out if you try to read all at once so you can no longer do
    # the commands below. Must read in smaller chunks.
    # # mraw = sen.metabuff.get_all_metadata_buffer_data()
    # # rpix = btop_raw12(mraw)

    rpix = [0] * n_pixels * n_taps

    meta_dyn = sen.metabuff.get_dynamic_buffer_data()
    pix_dyn = btop_raw12(meta_dyn)
    rpix[0:len(pix_dyn)] = pix_dyn

    meta_static = sen.metabuff.get_static_buffer_data()
    pix_static = btop_raw12(meta_static)
    rpix[48:48+len(pix_static)] = pix_static

    for virtual_sensor in range(8):
        mf = sen.metabuff.get_virtual_sensor_buffer_data(virtual_sensor)
        pf = btop_raw12(mf)
        rpix[200+virtual_sensor*32:200+virtual_sensor*32+len(pf)] = pf

    for idx, (rp, fm) in enumerate(zip(rpix, fmeta)):
        # if idx == 1:
        #     # Start row can dither
        #     assert rp >= fm - 2
        #     assert rp <= fm + 2
        if idx == 23:
            # We don't know exactly where SCAN_ROI_CNT
            # will be and it may have wrapped.
            # assert fm <= rp
            continue
        elif idx not in [
                # Some pixels will not match depending when the scan was stopped
                24, 25, 26, 27, 28, 29, 30,  # Timestamps
                31, 32, 33, 34, 35, 36, 37, 38, 39  # ADC measures
        ]:
            assert rp == fm, f'pixel {idx} does not match, {rp}, {fm}'

    assert len(rpix) == len(fmeta)

    # check timestamps between two subsequent ROI
    # Verify the number of ROIs between the two
    num_rois = fmeta2[23] - fmeta[23]
    # fmeta was the ROI right before fmeta2.
    tstamps = fmeta[24:31]
    tstamps2 = fmeta2[24:31]

    tstamp_coarse, tstamp_fine = get_timestamp_coarse_fine_from_list(tstamps)
    tstamp_coarse2, tstamp_fine2 = get_timestamp_coarse_fine_from_list(tstamps2)

    # tstamp_fine should increment every 10ns in this configuration
    tstamp_fine_increment = np.ceil(roi_time_us * num_rois / 0.010)

    def time_absdiff(time1_coarse, time1_fine, time2_coarse, time2_fine):
        """Calculates the absolute difference between two unsigned
        times.
        """
        absdiff_sec = 0
        absdiff_nsec = 0
        if time1_coarse == time2_coarse:
            absdiff_sec = 0
            absdiff_nsec = np.abs(time2_fine - time1_fine)
        else:
            if time1_coarse > time2_coarse:
                if time1_fine < time2_fine:
                    # Borrow
                    absdiff_sec = time1_coarse - time2_coarse - 1
                    absdiff_nsec = 1e9 + time1_fine - time2_fine
                else:
                    # No borrow
                    absdiff_sec = time1_coarse - time2_coarse
                    absdiff_nsec = time1_fine - time2_fine
            else:
                if time1_fine > time2_fine:
                    # Borrow
                    absdiff_sec = time2_coarse - time1_coarse - 1
                    absdiff_nsec = 1e9 + time2_fine - time1_fine
                else:
                    # No borrow
                    absdiff_sec = time2_coarse - time1_coarse
                    absdiff_nsec = time2_fine - time1_fine
        return absdiff_sec, absdiff_nsec

    msg = (f'Timestamp increment estimated to be {tstamp_fine_increment} '
           f'for num_rois {num_rois} with roi_time_us = {roi_time_us:.2f}. '
           f'But actually increment by {tstamp_fine2-tstamp_fine}. '
           f'Tstamp1 = {tstamps}, Tstamp = {tstamps2}. '
           f'tstamp_coarse = {tstamp_coarse}, tstamp_fine = {tstamp_fine}, '
           f'tstamp_coarse2 = {tstamp_coarse2}, tstamp_fine2 = {tstamp_fine2}.'
           f'Scan fetch delay {scan_fetch_delay:.0f}, '
           f'scan_trigger_delay {scan_trigger_delay:.0f}')
    absdiff_sec, absdiff_nsec = time_absdiff(tstamp_coarse, tstamp_fine,
                                             tstamp_coarse2, tstamp_fine2)
    assert absdiff_nsec == pt.approx(tstamp_fine_increment, abs=10), msg

    # Check image data
    def quantize(v, quant_mode=qm):
        """Quantization performed per quant_mode when reduce_mode == 1."""
        return np.clip(np.floor(v / 2**quant_mode), 0, 2**12 - 1).astype(int)

    #for jdx, (fraw, rraw) in enumerate(zip(l_fraw, l_rraw)):
    for subframe in range(n_subframes):
        for line in range(n_lines):
            for pixel in range(n_pixels):
                ftap_a, ftap_b, ftap_c = fraw[subframe, line, pixel, :]

                if test_mode == 1:
                    # line mode
                    if reduce_mode == 0:
                        tap_a_gold = (nrows * subframe + line)
                        tap_b_gold = tap_a_gold
                        tap_c_gold = tap_a_gold
                    elif mipi_raw_mode == 0:
                        tap_a_gold = quantize(3*nrows * subframe + 3 * line + 3*2*nrows)
                        tap_b_gold = tap_a_gold
                        tap_c_gold = tap_a_gold
                    else:
                        tap_a_gold = (3*nrows * subframe + 3 * line + 3*2*nrows)
                        tap_b_gold = tap_a_gold
                        tap_c_gold = tap_a_gold
                elif test_mode == 2:
                    # pixel mode
                    if reduce_mode == 0:
                        tap_a_gold = pixel
                        tap_b_gold = tap_a_gold
                        tap_c_gold = tap_a_gold
                    elif mipi_raw_mode == 0:
                        tap_a_gold = quantize(3 * pixel)
                        tap_b_gold = tap_a_gold
                        tap_c_gold = tap_a_gold
                    else:
                        tap_a_gold = (3 * pixel)
                        tap_b_gold = tap_a_gold
                        tap_c_gold = tap_a_gold
                elif test_mode == 3:
                    # tap mode
                    if reduce_mode == 0:
                        tap_a_gold = (3 * pixel + 0)
                        tap_b_gold = (3 * pixel + 1)
                        tap_c_gold = (3 * pixel + 2)
                    elif mipi_raw_mode == 0:
                        tap_a_gold = quantize(9 * pixel + 3)
                        tap_b_gold = tap_a_gold
                        tap_c_gold = tap_a_gold
                    else:
                        tap_a_gold = (9 * pixel + 3)
                        tap_b_gold = tap_a_gold
                        tap_c_gold = tap_a_gold
                else:
                    raise ValueError(f'Invalid param test_mode = {test_mode}.')

                sh = 2 if (mipi_raw_mode != 0) and bool(reduce_mode) else 4
                tap_a_gold <<= sh
                tap_b_gold <<= sh
                tap_c_gold <<= sh
                assert tap_a_gold == ftap_a
                assert tap_b_gold == ftap_b
                assert tap_c_gold == ftap_c
