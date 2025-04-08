from contextlib import nullcontext as does_not_raise
import pytest as pt
import random

import numpy as np

import cobra_system_control.exceptions as cobex
from cobra_system_control.laser import LaserPowerPercentMappedOvFactory
from cobra_system_control.metadata import PerVirtualSensorMetadata
from cobra_system_control.scan_control import SnrThresholdBv, BinningOv
from cobra_system_control.sensor_head import (
    DEFAULT_RTD_ALGORITHM_COMMON,
    DEFAULT_RTD_ALGORITHM_GRID_MODE,
    DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
    DEFAULT_DSP_MODE,
)
import cobra_system_control.random_access_scanning as ras_
from cobra_system_control.random_access_scanning import (
    INTE_TIME_US_OPTIONS, MaxRangeIdxMappedOv, InteTimeIdxMappedOv,
    FrameRateOv, NnLevelOv, FpsMultipleOv,
    STRIPE_MODE_FLAGS, DspMode,
)


@pt.mark.parametrize('mrl, error', [
    *[pt.param(x, None) for x in MaxRangeIdxMappedOv.OPTIONS],
    pt.param('1', pt.raises(ValueError)),
    pt.param(900, pt.raises(ValueError)),
])
def test_max_range_ov_make(mrl, error):
    with error or does_not_raise():
        MaxRangeIdxMappedOv(mrl)


def test_max_range_idx_ov():
    assert MaxRangeIdxMappedOv.OPTIONS == [25.2, 32.4]
    mmap = [(8, 7),
            (9, 8),
            ]
    for idx, opt in enumerate(MaxRangeIdxMappedOv.OPTIONS):
        assert MaxRangeIdxMappedOv.MAP[opt] == mmap[idx]
        assert MaxRangeIdxMappedOv(opt).mapped == mmap[idx]


@pt.mark.parametrize('itl, error', [
    *[pt.param(x, None) for x in InteTimeIdxMappedOv.OPTIONS],
    pt.param('1', pt.raises(ValueError)),
    pt.param(900, pt.raises(ValueError)),
])
def test_inte_time_ov_make(itl, error):
    with error or does_not_raise():
        InteTimeIdxMappedOv(itl)


def test_inte_time_idx_mapped_ov():
    assert InteTimeIdxMappedOv.OPTIONS == list(np.asarray(INTE_TIME_US_OPTIONS).astype(int))
    mmap = np.asarray(INTE_TIME_US_OPTIONS).astype(int) * 1e-6

    for i in InteTimeIdxMappedOv.OPTIONS:
        j = InteTimeIdxMappedOv.OPTIONS.index(i)
        assert InteTimeIdxMappedOv.MAP[j] == pt.approx(mmap[j], rel=1e-12)
        assert InteTimeIdxMappedOv(i).mapped == pt.approx(mmap[j], rel=1e-12)


@pt.mark.parametrize('frl, error', [
    *[pt.param(x, None) for x in FrameRateOv.OPTIONS],
    pt.param('1', pt.raises(ValueError)),
    pt.param(1900, pt.raises(ValueError)),
])
def test_frame_rate_ov_make(frl, error):
    with error or does_not_raise():
        FrameRateOv(frl)


def test_frame_rate_ov():
    assert FrameRateOv.OPTIONS == list(range(300, 960+10, 10))


@pt.mark.parametrize('nnl, error', [
    *[pt.param(x, None) for x in NnLevelOv.OPTIONS],
    pt.param('1', pt.raises(ValueError)),
    pt.param(6, pt.raises(ValueError)),
])
def test_nn_level_ov_make(nnl, error):
    with error or does_not_raise():
        NnLevelOv(nnl)


def test_nn_level_ov_options():
    assert NnLevelOv.OPTIONS == [0, 1, 2, 3, 4, 5]


@pt.mark.parametrize('fpl, error', [
    *[pt.param(x, None) for x in FpsMultipleOv.OPTIONS],
    pt.param('1', pt.raises(ValueError)),
    pt.param(3000, pt.raises(ValueError)),
])
def test_fps_multiple_ov_make(fpl, error):
    with error or does_not_raise():
        FpsMultipleOv(fpl)


def test_fps_multiple_ov():
    assert FpsMultipleOv.OPTIONS == list(range(1, 2**5))


ras_inputs = ("virtual_sensor_trip, fps, power, inte, "
              "mrange, binning, tag, "
              "nn, snr, algo_comm, algo_grid, algo_stripe,"
              "frate, rfrate, "
              "rangles, rpower, rinte, "
              "rfreq_ints, "
              "rbin, "
              "rtag, rtotal_roi, "
              "rflags, rvirtual_sensor_bitmask, error")
ras_errors = ([
    pt.param([(-45, 45, 1.0)], [31], [100], [15],
             [25.2], [2], [0xb],
             NnLevelOv(0), 0, 0, 0, 0,
             [500], [500]*100,
             list(range(-45, 46))*100, [0]*100,
             [15]*100,
             [MaxRangeIdxMappedOv.MAP[25.2]]*100,
             [2]*100,
             [0x4]*100, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='Too many entries for table'),
    pt.param([(-50, 50, 1)], [1], [100], [5],
             [25.2], [2], [0xb],
             NnLevelOv(0), 0, 0, 0, 0,
             500, 500,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternValueError),
             id='VIRTUAL_SENSOR angles out of bounds'),
    pt.param([-1, 1, 1], 0, 100, 5,
             25.2, 0, 0,
             NnLevelOv(0), 0, 0, 0, 0,
             500, 500,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternValueError),
             id='VIRTUAL_SENSOR not a tuple'),
    pt.param([(-45, 45, 1, 1)], [1], [100], [5],
             [25.2], [2], [0xb],
             NnLevelOv(0), 0, 0, 0, 0,
             500, 500,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='VIRTUAL_SENSOR tuple not the right size'),
    pt.param([], [1], [100], [5],
             [25.2], [2], [0xb],
             NnLevelOv(0), 0, 0, 0, 0,
             500, 500,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='Must define one VIRTUAL_SENSOR'),
    pt.param([(-1, 1, 1), (-1, 1, 1), (-1, 1, 1), (-1, 1, 1), (-1, 1, 1),
              (-1, 1, 1), (-1, 1, 1), (-1, 1, 1), (-1, 1, 1)],
             [0], [100], [5],
             [25.2], [2], [0xb],
             NnLevelOv(0), 0, 0, 0, 0,
             500, 500,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='Too many VIRTUAL_SENSOR angles'),
    pt.param([(-45, 45, 1)]*3, [1]*2, [100]*2, [5]*2,
             [25.2]*2, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*2, [0]*2,  [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='angles Common length different sizes'),
    pt.param([(-45, 45, 1)]*2, [1]*3, [100]*2, [5]*2,
             [25.2]*2, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*2, [0]*2, [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='fps Common length different sizes'),
    pt.param([(-45, 45, 1)]*2, [1]*2, [0]*3, [5]*2,
             [25.2]*2, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*2, [0]*2, [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='power Common length different sizes'),
    pt.param([(-45, 45, 1)]*2, [1]*2, [100]*2, [5]*3,
             [25.2]*2, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*2, [0]*2,  [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='inte Common length different sizes'),
    pt.param([(-45, 45, 1)]*3, [1]*2, [100]*2, [5]*2,
             [25.2]*3, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*2, [0]*2,  [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='freqs Common length different sizes'),
    pt.param([(-45, 45)]*2, [1]*2, [100]*2, [5]*2,
             [25.2]*2, [2]*3, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*2, [0]*2, [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='binning Common length different sizes'),
    pt.param([(-45, 45, 1)]*2, [1]*2, [100]*2, [5]*2,
             [25.2]*2, [2]*2, [0xb]*3,
             [NnLevelOv(0)]*2, [0]*2, [0]*2, [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='user tag Common length different sizes'),
    pt.param([(-45, 45, 1)]*2, [1]*2, [100]*2, [5]*2,
             [25.2]*2, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*3, [0]*2, [0]*2, [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='nn level Common length different sizes'),
    pt.param([(-45, 45, 1)]*2, [1]*2, [100]*2, [5]*2,
             [25.2]*2, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*2, [0]*3, [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='rtd algo Common length different sizes'),
    pt.param([(-45, 45, 1)]*2, [1]*2, [100]*2, [5]*2,
             [25.2]*2, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*3, [0]*2, [0]*2, [0]*2,
             [500]*2, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='snr threshold Common length different sizes'),
    pt.param([(-45, 45, 1)]*2, [1]*2, [100]*2, [5]*2,
             [25.2]*2, [2]*2, [0xb]*2,
             [NnLevelOv(0)]*2, [0]*2, [0]*2, [0]*2, [0]*2,
             [500]*3, None,
             None, None, None,
             None,
             None,
             None, None,
             None, None, pt.raises(cobex.ScanPatternSizeError),
             id='frame rate Common length different sizes'),
    ])

inte_us = 15
inte_s = 15e-6

ras_arg_tests = ([
    pt.param([(-1, 1, 1)], [0], [100], [inte_us],
             [25.2], [2], [0x3],
             NnLevelOv(0), 0, 0, 0, 0,
             [500], [500, 500, 500],
             [-1, 0, 1],
             [100, 100, 100],
             [inte_s, inte_s, inte_s],
             [MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2]],
             [2, 2, 2],
             [0x3, 0x3, 0x3], [3],
             [0b1, 0, 0b10], [0b1, 0b1, 0b1], None, id='fps==0 -> 1, simple single doublet'),
    pt.param([(-1, 1, 1)], [1], [100], [inte_us],
             [25.2], [2], [0x3],
             NnLevelOv(0), 0, 0, 0, 0,
             [500], [500, 500, 500],
             [-1, 0, 1],
             [100, 100, 100],
             [inte_s, inte_s, inte_s],
             [MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2]],
             [2, 2, 2],
             [0x3, 0x3, 0x3], [3],
             [0b1, 0, 0b10], [0b1, 0b1, 0b1], None, id='simple single triplet'),
    pt.param([(1, -1, 1)], [1], [100], [inte_us],
             [25.2], [2], [0x3],
             NnLevelOv(0), 0, 0, 0, 0,
             [500], [500, 500, 500],
             [1, 0, -1],
             [100, 100, 100],
             [inte_s, inte_s, inte_s],
             [MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2]],
             [2, 2, 2],
             [0x3, 0x3, 0x3], [3],
             [0b1, 0, 0b10], [0b1, 0b1, 0b1], None, id='simple single reversed triplet'),
    pt.param([(0, 0, 1)], [1, 1], [90, 90], [inte_us, inte_us],
             [25.2, 25.2], [2, 2], [1, 1],
             NnLevelOv(0), 0, 0, 0, 0,
             [500], [500, 500],
             [0, 0],
             [90, 90],
             [inte_s, inte_s],
             [MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2]],
             [2, 2],
             [1, 1], [2, 2],
             [0b11, 0b11<<4, ], [0b01, 0b10], None, id='angle_range len 1'),
])


@pt.mark.parametrize(
    ras_inputs, [
        *ras_errors,  # Note that the errors above are tested here
        *ras_arg_tests,
    ])
def test_ras_required_args_no_dip(
        virtual_sensor_trip, fps, power, inte, mrange, binning, tag,
        nn, snr, algo_comm, algo_grid, algo_stripe,
        frate, rfrate,
        rangles, rpower, rinte, rfreq_ints,
        rbin,
        rtag, rtotal_roi, rflags, rvirtual_sensor_bitmask, roi_mapping,
        error, system_type,
):
    factory = LaserPowerPercentMappedOvFactory()
    dummy_laser_power_mapped = factory(system_type, 0)
    nr = 8
    with error or does_not_raise():
        ras = ras_.RandomAccessScanning(
            angle_range=virtual_sensor_trip,
            fps_multiple=fps,
            laser_power_percent=power,
            inte_time_us=inte,
            max_range_m=mrange,
            binning=binning,
            frame_rate_hz=frate,
            user_tag=tag,
            roi_rows=nr,
            roi_mapping=roi_mapping,
            snr_threshold=snr,
            nn_level=nn,
            rtd_algorithm_common=algo_comm,
            rtd_algorithm_grid_mode=algo_grid,
            rtd_algorithm_stripe_mode=algo_stripe,
            double_dip=False,
            interleave=True,
            hdr_threshold=4095,
            hdr_laser_power_percent=40,
            hdr_inte_time_us=5,
            laser_power_mapped_cls=dummy_laser_power_mapped,
            dsp_mode=random.choice(list(DspMode))
        )
        ords = []
        srows = []
        rvirtual_sensor_metadata = PerVirtualSensorMetadata.empty_array()
        fps = [1 if x == 0 else x for x in fps]

        if rtag is None:
            # if the tag wasn't provided, we need to know
            # the random one that was assigned
            rtag = [ras.appset_dict['virtual_sensor_metadata'][i].user_tag
                    for i in range(len(virtual_sensor_trip))]
        if rbin is None:
            rbin = binning

        # need to make sure that the angle has be sequence extended
        for i, tup in enumerate(ras.angle_range):
            virtual_sensor_angles = np.arange(*tup) * -1
            o, s = ras.roi_mapping(angles=virtual_sensor_angles)
            for _ in range(fps[i]):
                ords.extend(o)
                srows.extend(s)
            print('ras range', tup, o, s, ords, srows)
            rvirtual_sensor_metadata[i] = PerVirtualSensorMetadata.build(
                user_tag=rtag[i],
                binning=BinningOv(rbin[i]),
                s_rows=max(0, min(srows) - 2),
                n_rows=min(480, max(srows) - min(srows) + nr + 4),
                n_rois=len(o),
                rtd_algorithm_common=algo_comm,
                rtd_algorithm_grid_mode=algo_grid,
                rtd_algorithm_stripe_mode=algo_stripe,
                snr_threshold=SnrThresholdBv(snr),
                nn_level=nn,
            )
        sort_idx = np.argsort(np.asarray(ras.sorting))
        ords = [ords[x] for x in sort_idx]
        srows = [srows[x] for x in sort_idx]

        assert ras.appset_dict['orders'] == ords
        assert ras.appset_dict['s_rows'] == srows

        rpow = [ras.laser_power_mapped_cls(x).mapped for x in rpower]
        assert ras.appset_dict['ci_v'] == rpow
        assert ras.appset_dict['inte_time_s'] == rinte
        assert ras.appset_dict['mod_freq_int'] == rfreq_ints
        assert ras.appset_dict['virtual_sensor_bitmask'] == rvirtual_sensor_bitmask
        if ras.dsp_mode == DspMode.CAMERA_MODE:
            assert ras.appset_dict['start_stop_flags'] == rflags
        elif ras.dsp_mode == DspMode.LIDAR_MODE:
            assert ras.appset_dict['start_stop_flags'] == [STRIPE_MODE_FLAGS] * len(ras.appset_dict['start_stop_flags'])
        assert ras.appset_dict['frame_rate_hz'] == rfrate

        for fm, rfm in zip(ras.appset_dict['virtual_sensor_metadata'], rvirtual_sensor_metadata):
            assert fm.user_tag == rfm.user_tag
            assert fm.binning == rfm.binning
            assert fm.s_rows == rfm.s_rows
            if ras.dsp_mode == DspMode.CAMERA_MODE:
                assert fm.n_rows == rfm.n_rows
                assert fm.n_rois == rfm.n_rois
            elif ras.dsp_mode == DspMode.LIDAR_MODE:
                assert fm.n_rows == ras.roi_rows
                assert fm.n_rois == 1
            assert fm.rtd_algorithm_common == rfm.rtd_algorithm_common
            assert fm.rtd_algorithm_grid_mode == rfm.rtd_algorithm_grid_mode
            assert fm.rtd_algorithm_stripe_mode == rfm.rtd_algorithm_stripe_mode
            assert fm.nn_level == rfm.nn_level


@pt.mark.parametrize("hdri, inte, error", [
    pt.param(1, 3, None, id='smaller'),
    pt.param(5, 5, None, id='same'),
    pt.param(20, 15, pt.raises(cobex.ScanPatternValueError), id='larger'),
    ])
def test_ras_hdr_inte_limit(hdri, inte, error, system_type, roi_mapping,):
    factory = LaserPowerPercentMappedOvFactory()
    dummy_laser_power_mapped = factory(system_type, 0)
    nr = 8
    with error or does_not_raise():
        _ = ras_.RandomAccessScanning(
            angle_range=[[-20, 20, 1]],
            fps_multiple=1,
            laser_power_percent=3,
            inte_time_us=inte,
            max_range_m=25.2,
            binning=2,
            frame_rate_hz=900,
            user_tag=0x2,
            roi_rows=nr,
            roi_mapping=roi_mapping,
            snr_threshold=1,
            nn_level=0,
            rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
            rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
            rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
            double_dip=True,
            interleave=True,
            hdr_threshold=4095,
            hdr_laser_power_percent=40,
            hdr_inte_time_us=hdri,
            laser_power_mapped_cls=dummy_laser_power_mapped,
            dsp_mode=DEFAULT_DSP_MODE,
        )


@pt.mark.parametrize("triplet, rlist", [
    pt.param((-2, 2, 1), [-2, -1, 0, 1, 2], id='forward'),
    pt.param((-10, -8, 1), [-10, -9, -8], id='forward neg'),
    pt.param((-10, -8, -1), [-10, -9, -8], id='forward neg, neg step'),
    pt.param((10, 12, 1), [10, 11, 12], id='forward pos'),
    pt.param((10, 12, -1), [10, 11, 12], id='forward pos, neg step'),
    pt.param((2, -2, 1), [2, 1, 0, -1, -2], id='bward'),
    pt.param((-30, -33, 1), [-30, -31, -32, -33], id='bward negative'),
    pt.param((-30, -33, -1), [-30, -31, -32, -33], id='bward negative, neg step'),
    pt.param((23, 20, 1), [23, 22, 21, 20], id='bward pos, pos step'),
    pt.param((23, 20, -1), [23, 22, 21, 20], id='bward pos, neg step'),
])
def test_fix_angle_triplet(triplet, rlist):
    rdata = np.arange(*ras_.fix_angle_triplet(triplet))
    assert list(rdata) == list(rlist)


inte_us0 = 5
inte_s0 = 5e-6
inte_us1 = 6
inte_s1 = 6e-6


@pt.mark.parametrize(
    "virtual_sensor_trip, fps, power, inte, "
    "mrange, binning, tag, "
    "frate, rfrate, "
    "rangles, rpower, rinte, "
    "rfreq_ints, "
    "rvirtual_sensormeta, "
    "rflags, rvirtual_sensor_bitmask, error",
    [
        pt.param([(-1, 1, 1), (-2, 2, 1)], [2, 1], [90, 100], [inte_us0, inte_us1],
                 [25.2, 32.4], [1, 2], [0xab, 0x3f],
                 [400, 500],
                 None, None, None, None, None, None, None, None,
                 pt.raises(cobex.ScanPatternValueError), id='two virtual_sensor, different mod freq'),
        pt.param([(-1, 1, 1), (-2, 2, 1)], [2, 1], [50, 100], [inte_us0, inte_us1],
                 [25.2, 25.2], [1, 1], [0xab, 0x3f],
                 [400, 500], [500] * 8,
                 [-1, -2, 0, 1, -1, 0, 2, 1],  # orders
                 [100] * 8,  # ci
                 [inte_s1] * 8,   # inte
                 [MaxRangeIdxMappedOv.MAP[25.2]] * 8,
                 [PerVirtualSensorMetadata.build(
                     user_tag=0xab, binning=BinningOv(1),
                     s_rows=0, n_rows=0, n_rois=3,
                     rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
                     rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
                     rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
                     snr_threshold=SnrThresholdBv(2),
                     nn_level=NnLevelOv(0)),
                  PerVirtualSensorMetadata.build(
                      user_tag=0x3f, binning=BinningOv(1),
                      s_rows=0, n_rows=0, n_rois=8,
                      rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
                      rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
                      rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
                      snr_threshold=SnrThresholdBv(2),
                      nn_level=NnLevelOv(0)),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty()],
                 [0b01 | (0b01 << 4), 0, 0, 0b10, 0b01, 0, 0, (0b10 << 4) | 0b10],  # flags
                 [0b11, 0b10, 0b11, 0b11, 0b11, 0b11, 0b10, 0b11],  # bitmask
                 None, id='two virtual_sensor, same mod freq'),
        pt.param([(0, 1, 1), (0, 1, 1)], [1, 1], [100, 100], [inte_us1, inte_us1],
                 [25.2, 25.2], [1, 1], [0x12, 0xab],
                 [500, 500], [500, 500],
                 [0, 1],
                 [100, 100],
                 [inte_s1, inte_s1],
                 [MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2]],
                 [PerVirtualSensorMetadata.build(
                     user_tag=0x12, binning=BinningOv(1),
                     s_rows=0, n_rows=0, n_rois=2,
                     rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
                     rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
                     rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
                     snr_threshold=SnrThresholdBv(2),
                     nn_level=NnLevelOv(0)),
                  PerVirtualSensorMetadata.build(
                      user_tag=0xab, binning=BinningOv(1),
                      s_rows=0, n_rows=0, n_rois=2,
                      rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
                      rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
                      rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
                      snr_threshold=SnrThresholdBv(2),
                      nn_level=NnLevelOv(0)),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty()],
                 [0b01 | (0b01 << 4), 0b10 | (0b10 << 4)],
                 [0b11, 0b11],
                 None,
                 id='overlapping rois, same parameters'),
        pt.param([(0, 1, 1), (0, 0, 1)], [1, 1], [50, 100], [inte_us1, inte_us1],
                 [25.2, 32.4], [1, 1], [0x12, 0xab],
                 [500, 600],
                 None, None, None, None, None, None, None, None,
                 pt.raises(cobex.ScanPatternValueError),
                 id='overlapping rois, different mod freq'),
        pt.param([(0, 1, 1), (0, 1, 1)], [2, 1], [100, 100], [inte_us1, inte_us1],
                 [25.2, 25.2], [1, 1], [0x12, 0xab],
                 [450, 460], [460, 460, 460, 460],
                 [0, 1, 0, 1],
                 [100, 100, 100, 100],
                 [inte_s1, inte_s1,
                  inte_s1, inte_s1],
                 [MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2],
                  MaxRangeIdxMappedOv.MAP[25.2], MaxRangeIdxMappedOv.MAP[25.2]],
                 [PerVirtualSensorMetadata.build(
                     user_tag=0x12, binning=BinningOv(1),
                     s_rows=0, n_rows=0, n_rois=2,
                     rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
                     rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
                     rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
                     snr_threshold=SnrThresholdBv(2),
                     nn_level=NnLevelOv(0)),
                  PerVirtualSensorMetadata.build(
                      user_tag=0xab, binning=BinningOv(1),
                      s_rows=0, n_rows=0, n_rois=2,
                      rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
                      rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
                      rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
                      snr_threshold=SnrThresholdBv(2),
                      nn_level=NnLevelOv(0)),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                 ],
                 [0b01 | (0b01 << 4), 0b10 | (0b10 << 4), 0b01 | (0b01 << 4), 0b10 | (0b10 << 4)],
                 [0b11, 0b11, 0b11, 0b11],
                 None,
                 id='overlapping rois, same parameters, extra fps'),
        pt.param([(0, 0, 1)], [1, 1], [100, 100], [inte_us1, inte_us1],
                 [25.2, 25.2], [2, 2], [1, 1],
                 [530, 530], [530],
                 [0], [100], [inte_s1],
                 [MaxRangeIdxMappedOv.MAP[25.2]],
                 [PerVirtualSensorMetadata.build(
                     user_tag=1, binning=BinningOv(2),
                     s_rows=0, n_rows=0, n_rois=1,
                     rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
                     rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
                     rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
                     snr_threshold=SnrThresholdBv(2),
                     nn_level=NnLevelOv(0)),
                  PerVirtualSensorMetadata.build(
                      user_tag=1, binning=BinningOv(2),
                      s_rows=0, n_rows=0, n_rois=1,
                      rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
                      rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
                      rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
                      snr_threshold=SnrThresholdBv(2),
                      nn_level=NnLevelOv(0)),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                  PerVirtualSensorMetadata.empty(),
                 ],
                 [0b00110011], [0b11], None, id='angle_range len 1'),
])
def test_ras_required_args_dip(virtual_sensor_trip, fps, power, inte, mrange, binning, tag,
                               frate, rfrate,
                               rangles, rpower, rinte, rfreq_ints,
                               rvirtual_sensormeta,
                               rflags, rvirtual_sensor_bitmask, roi_mapping,
                               error, system_type,
):
    factory = LaserPowerPercentMappedOvFactory()
    dummy_laser_power_mapped = factory(system_type, 0)
    nr = 8
    with error or does_not_raise():
        ras = ras_.RandomAccessScanning(
            angle_range=virtual_sensor_trip,
            fps_multiple=fps,
            laser_power_percent=power,
            inte_time_us=inte,
            max_range_m=mrange,
            binning=binning,
            frame_rate_hz=frate,
            user_tag=tag,
            roi_rows=nr,
            roi_mapping=roi_mapping,
            snr_threshold=1.5,
            nn_level=0,
            rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
            rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
            rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
            double_dip=True,
            interleave=True,
            hdr_threshold=4095,
            hdr_laser_power_percent=40,
            hdr_inte_time_us=5,
            laser_power_mapped_cls=dummy_laser_power_mapped,
            dsp_mode=random.choice(list(DspMode))
        )
        d = {'orders': [],
             'srows': []}
        fps = [1 if x==0 else x for x in fps]

        print('ras ang trip', ras.angle_range)
        for i, tup in enumerate(ras.angle_range):
            virtual_sensor_angles = np.arange(*tup) * -1
            o, s = ras.roi_mapping(
                angles=virtual_sensor_angles, trim_duplicates=True)

            for _ in range(fps[i]):
                d['orders'].extend(o)
                d['srows'].extend(s)
            rvirtual_sensormeta[i].s_rows = max(0, min(s) - 2)
            rvirtual_sensormeta[i].n_rows = min(480, max(s) - min(s) + nr + 4)

        d['orders'] = [x for i, x in enumerate(d['orders'])
                       if ras.full_sorting[i] in ras.trimmed_sorting]
        d['srows'] = [x for i, x in enumerate(d['srows'])
                      if ras.full_sorting[i] in ras.trimmed_sorting]
        d = ras_.schedule_scan(d, ras.sorting)

        assert ras.appset_dict['orders'] == d['orders']
        assert ras.appset_dict['s_rows'] == d['srows']

        rpow = [ras.laser_power_mapped_cls(x).mapped for x in rpower]
        assert ras.appset_dict['ci_v'] == rpow
        assert ras.appset_dict['inte_time_s'] == rinte
        assert ras.appset_dict['mod_freq_int'] == rfreq_ints
        assert ras.appset_dict['virtual_sensor_bitmask'] == rvirtual_sensor_bitmask
        if ras.dsp_mode == DspMode.CAMERA_MODE:
            assert ras.appset_dict['start_stop_flags'] == rflags
        elif ras.dsp_mode == DspMode.LIDAR_MODE:
            assert ras.appset_dict['start_stop_flags'] == [STRIPE_MODE_FLAGS] * len(ras.appset_dict['start_stop_flags'])
        assert ras.appset_dict['frame_rate_hz'] == rfrate

        for fm, rfm in zip(ras.appset_dict['virtual_sensor_metadata'], rvirtual_sensormeta):
            assert fm.user_tag == rfm.user_tag
            assert fm.binning == rfm.binning
            assert fm.s_rows == rfm.s_rows
            if ras.dsp_mode == DspMode.CAMERA_MODE:
                assert fm.n_rows == rfm.n_rows
                assert fm.n_rois == rfm.n_rois
            elif ras.dsp_mode == DspMode.LIDAR_MODE:
                assert fm.n_rows == ras.roi_rows
                assert fm.n_rois == 1
            assert fm.rtd_algorithm_common == rfm.rtd_algorithm_common
            assert fm.rtd_algorithm_grid_mode == rfm.rtd_algorithm_grid_mode
            assert fm.rtd_algorithm_stripe_mode == rfm.rtd_algorithm_stripe_mode
            assert fm.nn_level == rfm.nn_level


@pt.mark.parametrize("val, dsp_mode", [
    pt.param(val, dsp_mode, id=f'run{val}')
    for val in range(20)
    for dsp_mode in list(DspMode)
])
def test_flag_equalizer(val, dsp_mode, roi_mapping, system_type):
    factory = LaserPowerPercentMappedOvFactory()
    dummy_laser_power_mapped = factory(system_type, 0)
    num_virtual_sensor = random.randint(1, 8)
    start = [random.randint(-45, 45) for x in range(num_virtual_sensor)]
    stop = [random.randint(-45, 45) for x in range(num_virtual_sensor)]
    # Keep the scan from getting too big (i.e. < 512 entries)
    fps = []
    for i in range(num_virtual_sensor):
        try:
            fps.append(50 // abs(start[i]-stop[i]))
        except ZeroDivisionError:
            fps.append(2)
    fps = np.clip(fps, 1, 4)

    razz = ras_.RandomAccessScanning(
        angle_range=[(s, x, 1) for s, x in zip(start, stop)],
        fps_multiple=fps,
        laser_power_percent=[50],
        inte_time_us=[5],
        max_range_m=[25.2],
        binning=[2],
        user_tag=[0],
        frame_rate_hz=[840],
        roi_mapping=roi_mapping,
        roi_rows=8,
        snr_threshold=1,
        nn_level=0,
        rtd_algorithm_common=DEFAULT_RTD_ALGORITHM_COMMON,
        rtd_algorithm_grid_mode=DEFAULT_RTD_ALGORITHM_GRID_MODE,
        rtd_algorithm_stripe_mode=DEFAULT_RTD_ALGORITHM_STRIPE_MODE,
        double_dip=True,
        interleave=True,
        hdr_threshold=4095,
        hdr_laser_power_percent=40,
        hdr_inte_time_us=5,
        laser_power_mapped_cls=dummy_laser_power_mapped,
        dsp_mode=dsp_mode,
    )

    assert razz.fps_multiple == list(fps)
    new_dict = ras_.schedule_scan(razz.trimmed_appset_dict, razz.trimmed_sorting)
    if dsp_mode == DspMode.CAMERA_MODE:
        flag_list = ras_.flag_equalizer(
            new_dict['start_stop_flags'],
            new_dict['virtual_sensor_bitmask'],
            razz.fps_multiple)
    elif dsp_mode == DspMode.LIDAR_MODE:
        flag_list = (
            [0b00110011001100110011001100110011]
            * len(new_dict['start_stop_flags'])
        )
    assert flag_list == razz.appset_dict['start_stop_flags']

    for virtual_sensor, fr in enumerate(fps):
        loc_idx = np.flatnonzero(np.asarray(new_dict['virtual_sensor_bitmask'])
                                 >> virtual_sensor & 0b1 == 1)
        flags = np.asarray(flag_list)[loc_idx]
        virtual_sensor_flags = (flags >> (4*virtual_sensor)) & 0xf

        if not all(virtual_sensor_flags == 3):
            assert not any(virtual_sensor_flags == 3)
            msg = f'virtual_sensor{virtual_sensor}, {virtual_sensor_flags}'
            assert virtual_sensor_flags[0] == 1, msg
            assert virtual_sensor_flags[-1] == 2, msg

        num_threes = np.count_nonzero(virtual_sensor_flags == 3)
        assert (np.flatnonzero(flags).size + num_threes) >= fr * 2


@pt.mark.parametrize("val", [
    pt.param(val, id=f'run{val}') for val in range(15)
])
def test_replace_flag(val):
    bits = random.getrandbits(4*8)
    new_bits = random.randrange(0, 0xf)
    virtual_sensor = random.randint(0, 7)
    rdata = ras_.replace_flag(bits, new_bits, virtual_sensor)

    bitmask = 0xf << (4*virtual_sensor)
    assert rdata & (~bitmask) == bits & (~bitmask)
    assert (rdata >> (4*virtual_sensor) & 0xf) == new_bits


@pt.mark.parametrize("val", [
        pt.param(val, id=f'run{val}') for val in range(15)
])
def test_replace_flag_array(val):
    bits = [random.getrandbits(4*8) for x in range(3)]
    new_bits = [random.randrange(0, 0xf) for x in range(3)]
    virtual_sensor = random.randint(0, 7)
    rdata = ras_.replace_flag(bits, new_bits, virtual_sensor)

    for x in range(3):
        bitmask = 0xf << (4*virtual_sensor)
        assert rdata[x] & (~bitmask) == bits[x] & (~bitmask)
        assert (rdata[x] >> (4*virtual_sensor) & 0xf) == new_bits[x]


@pt.mark.parametrize("wlist, ret", [
    pt.param([1, 0, 2], True, id='102'),
    pt.param([1, 2], True, id='12'),
    pt.param([1, 2, 1, 2], True, id='1212'),
    pt.param([3, 3, 3], False, id='all three'),
    pt.param([2, 0, 1], False, id='reversed w zero'),
    pt.param([1, 1, 1], False, id='No end 2'),
    pt.param([2, 2, 2], False, id='No start 1'),
    pt.param([1, 2, 0, 1, 2], False, id='Zero after 2'),
    pt.param([1, 1, 1, 2], False, id='1 in sequence'),
    pt.param([1, 2, 2, 2], False, id='2 in sequence'),
    pt.param([2, 1, 2, 1], False, id='reversed 2'),
    pt.param([1], False, id='Single entry 1'),
    pt.param([3], False, id='Single entry 2'),
])
def test_is_valid_normal(wlist, ret):
    rdat = ras_.is_valid_normal(np.asarray(wlist))
    assert rdat == ret


@pt.mark.parametrize("wlist, ret", [
    pt.param([1, 0, 2], False, id='102'),
    pt.param([1, 2], False, id='12'),
    pt.param([1, 2, 1, 2], False, id='1212'),
    pt.param([3, 3, 3], False, id='all three'),
    pt.param([2, 0, 1], True, id='reversed w zero'),
    pt.param([1, 1, 1], False, id='No end 2'),
    pt.param([2, 2, 2], False, id='No start 1'),
    pt.param([1, 2, 0, 1, 2], False, id='Zero after 2'),
    pt.param([1, 1, 1, 2], False, id='1 in sequence'),
    pt.param([1, 2, 2, 2], False, id='2 in sequence'),
    pt.param([2, 1, 2, 1], True, id='reversed 2'),
    pt.param([1], False, id='Single entry 1'),
    pt.param([3], False, id='Single entry 2'),
])
def test_is_almost_valid_reversed(wlist, ret):
    rdat = ras_.is_almost_valid_reversed(np.asarray(wlist))
    assert rdat == ret


@pt.mark.parametrize("wlist, ret", [
    pt.param([1, 0, 2], False, id='102'),
    pt.param([1, 2], False, id='12'),
    pt.param([1], True, id='Single entry 1'),
    pt.param([3], True, id='Single entry 2'),
])
def test_is_almost_valid_threes(wlist, ret):
    rdat = ras_.is_almost_valid_threes(np.asarray(wlist))
    assert rdat == ret
