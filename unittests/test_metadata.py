from contextlib import nullcontext as does_not_raise
from unittest.mock import MagicMock

import pytest as pt

from unittests.conftest import (make_virtual_sensormeta_init_kwargs,
                                make_staticmeta_init_kwargs,
                                make_valid_scan_entry)

from cobra_system_control.metadata import PerVirtualSensorMetadata, StaticMetadata
from cobra_system_control.random_access_scanning import NnLevelOv
from cobra_system_control.scan_control import SnrThresholdBv, BinningOv, ScanTable
from cobra_system_control.numerical_utilities import btop_raw12


def test_virtual_sensor_metadata_instantiation():
    init_kwargs, regs = make_virtual_sensormeta_init_kwargs()
    with does_not_raise():
        fm = PerVirtualSensorMetadata(**init_kwargs)

    for name, _ in PerVirtualSensorMetadata.memmap():
        assert getattr(fm, name) == regs[name]


def test_virtual_sensor_metadata_build():
    init_kwargs, regs = make_virtual_sensormeta_init_kwargs()
    init_kwargs['snr_threshold'] = SnrThresholdBv.from_field(regs['snr_threshold'])
    init_kwargs['nn_level'] = NnLevelOv(regs['nn_level'])
    init_kwargs['binning'] = BinningOv(regs['binning'])

    with does_not_raise():
        fm = PerVirtualSensorMetadata.build(**init_kwargs)

    for name, _ in PerVirtualSensorMetadata.memmap():
        if name == 'snr_threshold':
            assert pt.approx(getattr(fm, name), 1) == regs[name]
        elif name == 'random_virtual_sensor_tag':
            # It's random so we expect this to fail
            continue
        else:
            assert getattr(fm, name) == regs[name]


def test_static_metadata_instantiation():
    init_kwargs, regs = make_staticmeta_init_kwargs()
    with does_not_raise():
        fm = StaticMetadata(**init_kwargs)

    for name, _ in StaticMetadata.memmap():
        assert getattr(fm, name) == regs[name]


def test_virtual_sensor_metadata_buffer_mem(mock, sensor_head):
    fms = []
    lkwargs = []
    lregs = []
    for _ in range(8):
        init_kwargs, regs = make_virtual_sensormeta_init_kwargs()
        fms.append(PerVirtualSensorMetadata(**init_kwargs))
        lkwargs.append(init_kwargs)
        lregs.append(regs)

    mb = sensor_head.metabuff
    mb.write_metadata_buffer_virtual_sensor_data(fms)

    if mock:
        mb.get_virtual_sensor_buffer_data = MagicMock(side_effect=(lambda x: fms[x].data_words))

    for i in range(8):
        rraw = mb.get_virtual_sensor_buffer_data(i)
        assert list(rraw) == list(fms[i].data_words)


def test_static_metadata_buffer_mem(mock, sensor_head):
    init_kwargs, _ = make_staticmeta_init_kwargs()
    sm = StaticMetadata(**init_kwargs)

    mb = sensor_head.metabuff
    mb.write_metadata_buffer_static(sm)
    if mock:
        mb.get_static_buffer_data = MagicMock(side_effect=(lambda: sm.data_words))
    rraw = mb.get_static_buffer_data()
    assert list(rraw) == list(sm.data_words)


@pt.mark.usefixtures('integration_only')
def test_metadata_roi_mem(ffuncs, sensor_head, scan, metabuff,
                          scan_params, lcm_ctrl):
    fms = []
    for _ in range(8):
        finit_kwargs, fregs = make_virtual_sensormeta_init_kwargs()
        finit_kwargs['snr_threshold'] = SnrThresholdBv.from_field(fregs['snr_threshold'])
        finit_kwargs['nn_level'] = NnLevelOv(fregs['nn_level'])
        finit_kwargs['binning'] = BinningOv(fregs['binning'])
        fms.append(PerVirtualSensorMetadata.build(**finit_kwargs))

    s_init_kwargs, _ = make_staticmeta_init_kwargs()
    sm = StaticMetadata(**s_init_kwargs)

    entry = make_valid_scan_entry(sensor_head.laser_ci_dac,
                                  sensor_head.fpga_field_funcs)
    table = ScanTable((entry,))
    scan_params.write_scan_table(table)
    metabuff.write_metadata_buffer_static(sm)
    metabuff.write_metadata_buffer_virtual_sensor_data(fms)
    # LCM needs to be enabled because mocked by default
    lcm_ctrl.enable()

    # reset scan controller
    scan.write_fields(reset='all_reset')
    scan.write_fields(scan_loopback=False)
    assert scan.read_fields('fifo_count') == 0
    roi_cnt_begin = scan.read_fields('scan_roi_cnt')

    npops = 3
    for _ in range(npops):
        sensor_head.write_scan_fifo(*sensor_head.valid_scan_table_pointer_range)

    assert entry.roi_sel == scan.read_fields('scan_params_roi_sel') % 512
    assert roi_cnt_begin + npops == scan.read_fields('scan_roi_cnt')
    assert entry.roi_id == scan.read_fields('param_roi_id')
    assert entry.laser_ci == scan.read_fields('param_dac_ci')

    # Ensure the dynamic metadata matches
    rraw = list(metabuff.get_dynamic_buffer_data())
    # Convert to pixels
    rpix = btop_raw12(rraw)
    # These are defined here: /resources/metadata_map.yml
    assert rpix[0] == entry.sensor_mode
    assert rpix[1] == entry.rwin0_s - 4
    assert rpix[2] == entry.rwin0_l - 4
    assert rpix[3] == entry.mod_freq0_opt + 3
    assert rpix[4] == entry.mod_freq1_opt + 3
    assert rpix[5] == entry.npulse_group_f0
    assert rpix[6] == entry.npulse_group_f1
    assert rpix[7] == entry.inte_burst_length_f0
    assert rpix[8] == entry.inte_burst_length_f1
    assert rpix[9] == entry.roi_id
    # assert rpix[10] == entry.blob1
    # assert rpix[11] == entry.blob2
    # assert rpix[12] == entry.blob3
    # assert rpix[13] == entry.blob4
    assert rpix[14] == entry.virtual_sensor_bitmask
    assert rpix[15] == entry.start_stop_flags & 0xf
    assert rpix[16] == (entry.start_stop_flags >> 4) & 0xf
    assert rpix[17] == (entry.start_stop_flags >> 8) & 0xf
    assert rpix[18] == (entry.start_stop_flags >> 12) & 0xf
    assert rpix[19] == (entry.start_stop_flags >> 16) & 0xf
    assert rpix[20] == (entry.start_stop_flags >> 20) & 0xf
    assert rpix[21] == (entry.start_stop_flags >> 24) & 0xf
    assert rpix[22] == (entry.start_stop_flags >> 28) & 0xf
    assert rpix[23] == roi_cnt_begin + npops

    # The rest of the pixels are filled in by the FPGA and I
    # don't know what they will be.

    # Ensure the static metadata matches
    rraw = metabuff.get_static_buffer_data()
    assert list(rraw) == list(sm.data_words)

    # Ensure the virtual_sensor metadata matches
    for i in range(8):
        rraw = metabuff.get_virtual_sensor_buffer_data(i)
        assert list(rraw) == list(fms[i].data_words)

    lcm_ctrl.disable()
    scan.write_fields(reset='all_reset')
