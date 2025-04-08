# pylint: disable=redefined-outer-name, protected-access
"""Fixture file for configuring pytest options in raw Python.
"""
import os
import random
import sys
import time
import uuid
from pathlib import Path
from unittest.mock import MagicMock, PropertyMock

import numpy as np
import pytest

from cobra_system_control.calibration_data import (
    CalData, CalGroup)
from cobra_system_control.cobra import Cobra
from cobra_system_control.cobra_log import log
from cobra_system_control.compute import CpuGpio
from cobra_system_control.device import DummyDevice
from cobra_system_control.dacs import (
    SensorVDac, TiDac6578,
    ItoDac, MchpDacMCP48FVB12,
    LcmVDac
)
from cobra_system_control import fe_ctl
from cobra_system_control.fpga_field_funcs import FpgaFieldFuncs
from cobra_system_control.fpga_adc import FpgaAdc
from cobra_system_control.fpga_misc import FpgaDbg, ISP
from cobra_system_control.itof import Itof, InteTimeSBv, FrameSettings
from cobra_system_control.laser import LaserCiDac, LaserVldaDac
from cobra_system_control.memory_map import (
    M30_FPGA_MEMORY_MAP, M30_SPI_FLASH_MEMORY_MAP,
    )
import cobra_system_control.m30_fpga_collateral_md5 as md5m30
from cobra_system_control.metadata import MetadataBuffer, PerVirtualSensorMetadata, StaticMetadata
from cobra_system_control.metasurface import LcmController, LcmBuff, LM10OrderOv, LcmAssembly
from cobra_system_control.scan_control import Scan, ScanParams
from cobra_system_control.sensor_head import SensorHead
from cobra_system_control.random_access_scanning import NnLevelOv, BinningOv, FrameRateOv, MaxRangeIdxMappedOv
from cobra_system_control.roi_mapping import RoiMapping
from cobra_system_control.pixel_mapping import DEFAULT_PIXEL_MAPPING
from cobra_system_control.scan_control import ScanEntry
from cobra_system_control.spi_flash import QspiController, SpiFlash
from cobra_system_control.device import I2CBus
from cobra_system_control.functional_utilities import free_i2c_bus

import cobra_system_control.w25q32jw_const as wb


SAFE_LASER_CI = 0.01


def pytest_addoption(parser):
    """Add your command line options here.

    Default calls can be specified in ``pytest.ini``.
    """
    parser.addoption(
        "--random_seed",
        action="store",
        type=int,
    )
    parser.addoption(
        "--integration",
        action="store_true",
        help="Perform integration testing (no mocking).",
    )


def pytest_configure(config):
    if config.getoption("--integration"):
        free_i2c_bus()


def pytest_unconfigure(config):
    if config.getoption("--integration"):
        free_i2c_bus()
        os.system('gpioset 4 17=0')
        os.system('gpioset 4 18=0')
        os.system('gpioset 4 19=0')


@pytest.fixture(scope="session")
def board_type(request):
    return 'nxp'


@pytest.fixture(scope="session")
def system_type(request):
    return 'm30'


@pytest.fixture(scope="session")
def fpga_addr():
    return 0x10


@pytest.fixture(scope="session")
def fifo_addr():
    return 0x11


@pytest.fixture(scope="session")
def mmap(system_type):
    return M30_FPGA_MEMORY_MAP


@pytest.fixture(scope="session")
def sfmap(system_type):
    return M30_SPI_FLASH_MEMORY_MAP


@pytest.fixture(scope="session")
def mock(request):
    """Returns if the current testing session is an integration test.
    """
    return not request.config.getoption("--integration")


@pytest.fixture(scope="session")
def full_unit(request):
    """Returns true if the current testing session
    is an integration test on a m20 unit.
    """
    return (request.config.getoption("--integration") and
            request.config.getoption("--full_unit"))


@pytest.fixture(scope="session", autouse=True)
def cobra(mock, board_type, system_type):
    if mock:
        c = MagicMock(spec=Cobra, whoami=system_type,  board_type=board_type)
    else:
        c = Cobra(whoami=system_type, board_type=board_type)
    c.connect()
    c.setup()
    c.enable()
    if not mock:
        assert system_type == c.sen.debug.read_fields('project', use_mnemonic=True)
        previous_cal_data = c.sen.get_cal_data()
    else:
        version_message = "Not available"
        c.compute.os_build_number = version_message
        c.compute.os_build_sha = version_message
        c.compute.os_build_version = version_message
        c.compute.manifest = version_message
        c.compute.manifest_sha = version_message

        c.fpga_bin_path = Path(Path(__file__).parent, 'resources',
                               'm30_fpga_dual_boot.bin').absolute()
        c.golden_sha = 0xd13247dd
        c.released_sha = md5m30.FPGA_RELEASED_SHA
        c.released_mcs_md5 = md5m30.BIN_MD5
    yield c
    if not mock:
        after_cal_data = c.sen.get_cal_data()
        # Make sure that the spi flash wasn't disturbed
        msg = 'contents of spi flash changed after unit tests'
        assert previous_cal_data.ba == after_cal_data.ba, msg
        log.debug(f'git sha is {c.sen.debug.read_fields("git_sha"):#010x}')

    c.disconnect()


@pytest.fixture(scope="session")
def golden_shas(cobra) -> dict:
    return {
        'golden': cobra.golden_sha,
        'release': cobra.released_sha,
    }


@pytest.fixture(autouse=True, scope="session")
def random_seed(request):
    seed = request.config.getoption("--random_seed")
    if seed is None:
        seed = random.randrange(sys.maxsize)
    random.seed(seed)
    np.random.seed(seed % (2 ** 32))
    seed_file = Path(Path(__file__).parent / "random_seeds.txt").resolve()
    with open(seed_file, "a", encoding='utf8') as f:
        f.write(f"{uuid.uuid4()}: {seed}\n")
    return seed


@pytest.fixture(scope="function")
def cal_data(system_type) -> 'CalData':
    return CalData


@pytest.fixture(scope="function")
def cam_i2c_bus(mock) -> I2CBus:
    i2 = I2CBus(2)
    if mock:
        yield MagicMock(spec=I2CBus)
    else:
        i2.connect()
        yield i2


@pytest.fixture(scope="function")
def cmb_i2c_bus(mock) -> I2CBus:
    i2 = I2CBus(4)
    if mock:
        yield MagicMock(spec=I2CBus)
    else:
        i2.connect()
        yield i2


@pytest.fixture(scope="function")
def ti_6578_dac(mock, cmb_i2c_bus, board_type) -> TiDac6578:
    ref = 3.3
    _ti_dac = TiDac6578(cmb_i2c_bus, 0x48, None, ref)

    if mock:
        _ti_dac.i2c.write = MagicMock()
        _ti_dac.i2c.read = MagicMock()
        _ti_dac.dac_send_cmd_and_block = MagicMock()
    _ti_dac.connect()
    _ti_dac.setup()
    yield _ti_dac


@pytest.fixture(scope="function")
def mchp_48fvb12_dac(mock, cam_i2c_bus, mmap, fpga_addr) -> MchpDacMCP48FVB12:
    _mchp = MchpDacMCP48FVB12(cam_i2c_bus, fpga_addr, mmap.dac_spi, 2.44, 1, 0)
    if mock:
        _mchp.dac_read = MagicMock()
        _mchp.dac_write = MagicMock()
        _mchp.dac_read.side_effect = {'vref': 0x5,
                                      'gain': 0}.get
        _mchp.dac_send_cmd_and_block = MagicMock()
        _mchp.dac_full_scale = _mchp.dac_full_scale_pre_gain * _mchp.gain

    _mchp.connect()
    _mchp.setup()
    yield _mchp


@pytest.fixture(scope="function")
def laser_ci_dac(mock, cam_i2c_bus, mmap, fpga_addr, system_type) -> LaserCiDac:
    """PCB REV is hardcoded to zero to set lowest power
    """
    _laser = LaserCiDac(
        MchpDacMCP48FVB12(
            cam_i2c_bus, fpga_addr, mmap.dac_spi, 2.44, 2, 0),
        1, None)

    if mock:
        _laser.dac.dac_read = MagicMock()
        _laser.dac.dac_write = MagicMock()
        _laser.dac.dac_read.side_effect = [
            0x5, 0, 512, 0x5, 0, 512, 0x5, 0, 512]
        _laser.dac.dac_full_scale = _laser.dac.dac_full_scale_pre_gain * _laser.dac.gain
        _laser.connect = MagicMock()
    _laser.connect()
    _laser.setup()
    _laser.set_ci_limit(system_type, 0)
    yield _laser
    _laser.disable()


@pytest.fixture(scope="function")
def cmb_laser_vlda_dac(mock, cmb_i2c_bus, board_type) -> LaserVldaDac:
    _laser = LaserVldaDac(TiDac6578(cmb_i2c_bus, 0x48, None, 3.3, 1),
                          0, board_type, CpuGpio(4, 17))
    if mock:
        _laser.enable_ctrl = MagicMock(spec=CpuGpio)
        _laser.connect = MagicMock()

    _laser.connect()
    _laser.setup()
    yield _laser
    _laser.disable()
    _laser.disconnect()


@pytest.fixture(scope="function")
def cmb_lcm_v_dac(mock, ti_6578_dac, board_type, lcm_ctrl) -> LaserVldaDac:
    _lcmdac = LcmVDac(ti_6578_dac, 2, board_type, CpuGpio(4, 18), lcm_ctrl)

    if mock:
        _lcmdac.cmb_enable_ctrl = MagicMock(spec=CpuGpio)
        _lcmdac.fpga_enable_ctrl = MagicMock()

    _lcmdac.full_scale_pre_gain = 3.3
    _lcmdac.dac.gain = 1
    _lcmdac.connect()
    _lcmdac.setup()
    yield _lcmdac
    _lcmdac.disable()
    _lcmdac.disconnect()


@pytest.fixture(scope="function")
def sh_laser_vlda_dac(mock, system_type,
                      cam_i2c_bus, mmap, fpga_addr, fpga_dbg):
    _laser = LaserVldaDac(
        MchpDacMCP48FVB12(
            cam_i2c_bus, fpga_addr, mmap.dac_spi, 2.44, 2, 0),
        0, system_type, fpga_dbg)

    if mock:
        #_laser.enable_ctrl = MagicMock(spec=CpuGpio)
        _laser.dac.dac_read = MagicMock()
        _laser.dac.dac_write = MagicMock()
        _laser.dac.dac_read.side_effect = [
            0x5, 0, 256, 0x5, 0, 256,  0x5, 0, 256]

        _laser.dac.dac_send_cmd_and_block = MagicMock()

        _laser.dac.dac_full_scale = _laser.dac.dac_full_scale_pre_gain * _laser.dac.gain
    _laser.connect()
    _laser.setup()
    yield _laser


@pytest.fixture(scope="function")
def sensor_v_dac(mock, ti_6578_dac, board_type) -> SensorVDac:
    _svd = SensorVDac(ti_6578_dac, 4, board_type, CpuGpio(4, 19))
    if mock:
        _svd.enable_ctrl = MagicMock(spec=CpuGpio)
    _svd.dac.full_scale_pre_gain = 3.3
    _svd.dac.gain = 1

    _svd.connect()
    _svd.setup()
    yield _svd
    _svd.disconnect()


@pytest.fixture(scope="function")
def ito_dac(mock, system_type, cam_i2c_bus, mmap, fpga_addr):
    _ito = ItoDac(MchpDacMCP48FVB12(cam_i2c_bus, fpga_addr,
                                    mmap.dac_spi, 2.44, 1, 1),
                  0)

    if mock:
        _ito.dac.dac_read = MagicMock()
        _ito.dac.dac_write = MagicMock()
        _ito.dac.dac_read.side_effect = {'vref': 0x5,
                                         'gain': 0}.get
        _ito.dac.dac_send_cmd_and_block = MagicMock()

    _ito.connect()
    _ito.setup()
    yield _ito


@pytest.fixture(scope="function")
def itof(mock, cam_i2c_bus, fpga_dbg, system_type, mmap, fpga_addr) -> Itof:
    gtof = Itof(cam_i2c_bus, fpga_addr, fpga_dbg, mmap.itof_spi, system_type)
    gtof.connect()

    if mock:
        gtof.write_fields = MagicMock()
        gtof.read_fields = MagicMock()
        gtof.reset = MagicMock()
    yield gtof
    gtof.reset()


@pytest.fixture(scope="function")
def fpga_adc(mock, cam_i2c_bus, system_type, mmap, fpga_addr) -> FpgaAdc:
    adc = FpgaAdc(cam_i2c_bus, fpga_addr, mmap.adc, system_type)
    adc.connect()

    if mock:
        adc.read_adc_and_adjust = MagicMock()
        adc.read_fields = MagicMock()
        adc.write_fields = MagicMock()
        adc.calibrate = MagicMock()
        adc._cal_gain = 0.0003
        adc._cal_offset = 0
    adc.setup()

    return adc


@pytest.fixture(scope="function")
def fpga_dbg(mock, cam_i2c_bus, mmap, fpga_addr) -> FpgaDbg:
    dbg = FpgaDbg(cam_i2c_bus, fpga_addr, mmap.dbg)
    dbg.connect()
    if mock:
        dbg.read_fields = MagicMock()
        dbg.write_fields = MagicMock()
    return dbg


@pytest.fixture(scope="function")
def fpga_isp(mock, cam_i2c_bus, mmap, fpga_addr) -> ISP:
    isp = ISP(cam_i2c_bus, fpga_addr, mmap.isp)
    isp.connect()
    if mock:
        isp.read_fields = MagicMock()
        isp.write_fields = MagicMock()
    return isp


@pytest.fixture(scope="function")
def ffuncs(mmap) -> FpgaFieldFuncs:
    _ff = FpgaFieldFuncs(
        memmap_fpga=mmap,
    )
    return _ff


@pytest.fixture(scope="function")
def lcm_ctrl(mock, cam_i2c_bus, ffuncs, mmap, scan, fpga_addr) -> LcmController:
    buff = LcmBuff(cam_i2c_bus, fpga_addr, mmap.lcm_buff)
    meta = LcmController(cam_i2c_bus, fpga_addr, mmap.lcm,
                         buff, ffuncs, scan)

    meta.connect()
    if mock:
        meta.read_fields = MagicMock()
        meta.write_fields = MagicMock()
    return meta


@pytest.fixture(scope="function")
def metabuff(mock, cam_i2c_bus, scan, mmap, fpga_addr) -> MetadataBuffer:
    _metabuff = MetadataBuffer(cam_i2c_bus, fpga_addr, mmap.meta_buff, scan)
    if mock:
        _metabuff.read_fields = MagicMock()
        _metabuff.write_fields = MagicMock()
    _metabuff.connect()
    yield _metabuff


@pytest.fixture(scope="function")
def scan(mock, cam_i2c_bus, mmap, fpga_addr) -> Scan:
    _scan = Scan(cam_i2c_bus, fpga_addr, mmap.scan)
    _scan.connect()
    if mock:
        _scan.read_fields = MagicMock()
        _scan.write_fields = MagicMock()
        # Necessary after moving wait_for_idle to the scan periph
        _scan.wait_for_scan_idle = MagicMock()
    yield _scan
    _scan.stop()


@pytest.fixture(scope="function")
def scan_params(mock, cam_i2c_bus, mmap, fpga_addr) -> ScanParams:
    _scan_params = ScanParams(cam_i2c_bus, fpga_addr, mmap.scan_params)
    _scan_params.connect()
    if mock:
        _scan_params.read_fields = MagicMock()
        _scan_params.write_fields = MagicMock()
    return _scan_params


@pytest.fixture(scope="function")
def spi_flash(mock, cam_i2c_bus, cobra, mmap, sfmap, fifo_addr) -> SpiFlash:
    qspi = QspiController(cam_i2c_bus, fifo_addr, mmap.qspi)
    _spi_flash = SpiFlash(qspi, sfmap, cobra.fpga_bin_path)
    qspi.connect()
    if mock:
        qspi._qspi_receive = MagicMock()
        # qspi.read_addressed = MagicMock()

        qspi.write_fields = MagicMock()
        qspi.read_fields = MagicMock()

        # Used to avoid infinite loops
        read_res = {"spi_busy": 0}
        qspi.read_fields.side_effect = lambda f: read_res.get(f)

    ## reset the spi flash
    qspi._qspi_send([wb.QSPI_CMD_ENABLE_RESET])
    qspi._qspi_send([wb.QSPI_CMD_RESET_DEVICE])
    # 30us reset time listed in the Windbond spec
    if not mock:
        time.sleep(0.1)  # reset time per Winbond spec
    return _spi_flash


@pytest.fixture(scope='function')
def pixelmap(mock):
    pm = DEFAULT_PIXEL_MAPPING
    if mock:
        pm.write_mapping_table_file = MagicMock()

    return pm


@pytest.fixture(scope="function")
def lcmsa():
    return LcmAssembly()


@pytest.fixture(scope="function")
def roi_mapping(lcmsa):
    return RoiMapping(
        a2a_coefficients=(45.44, -0.24, 0, 0),
        pixel_mapping=DEFAULT_PIXEL_MAPPING,
        lcm_assembly=lcmsa
        )


@pytest.fixture(scope="function")
def sensor_head(mock, itof, lcm_ctrl, cobra,
                fpga_adc, fpga_dbg, fpga_isp, cal_data,
                scan, scan_params, metabuff,
                spi_flash, ffuncs, ito_dac, cmb_lcm_v_dac,
                laser_ci_dac, cmb_laser_vlda_dac, sh_laser_vlda_dac,
                roi_mapping, lcmsa, system_type, board_type):

    # Keep the actual sensor head, but assume the peripherals are tested
    if mock:
        sf = MagicMock(spec=SpiFlash)
        sf.qspi = MagicMock(spec=QspiController)
    else:
        sf = spi_flash

    sh = SensorHead(
        whoami=system_type,
        compute_platform=cobra.compute,
        debug=MagicMock(spec=FpgaDbg) if mock else fpga_dbg,
        isp=MagicMock(spec=ISP) if mock else fpga_isp,
        ito_dac=ito_dac,
        itof=MagicMock(spec=Itof) if mock else itof,
        laser_ci_dac=laser_ci_dac,
        cmb_laser_vlda_dac=cmb_laser_vlda_dac,
        sh_laser_vlda_dac=sh_laser_vlda_dac,
        cmb_lcm_v_dac=cmb_lcm_v_dac,
        lcm_ctrl=MagicMock(spec=LcmController) if mock else lcm_ctrl,
        metabuff=metabuff,
        scan=MagicMock(spec=Scan) if mock else scan,
        scan_params=scan_params,
        spi_flash=sf,
        fpga_field_funcs=ffuncs,
        fpga_adc=fpga_adc,
    )

    if not mock:
        sh.connect()
        sh.setup()
        sh.enable()
    if mock:
        sh.aggregate = False
        sh.apply_calibration = MagicMock()
        sh.spi_flash.qspi.fast_read_data.return_value = (
            bytearray([0xaf] * 1024))
        sh.scan.wait_for_scan_idle = MagicMock()
        sh.itof.reset = MagicMock()
        fe_ctl.fe_send = MagicMock()
        sh._roi_mapping = roi_mapping
        sh.sensor_sn = random.choice(range(100))
        sh._cal_data = cal_data.empty()
        sh._cal_data.range0807 = MagicMock(spec=CalGroup)
        sh._cal_data.range0908 = MagicMock(spec=CalGroup)
        mock_valid = PropertyMock(return_value=True)
        type(sh._cal_data.range0807).is_valid = mock_valid
        type(sh._cal_data.range0908).is_valid = mock_valid
        sh._lcm_ctrl.buff = MagicMock(spec=LcmBuff)
        sh._lcm_assembly = lcmsa
        sh.fpga_adc._cal_gain = .001
        sh.fpga_adc._cal_offset = .001
        sh.ci_max = 1.5

    yield sh
    sh.disable()
    sh.itof.reset()
    sh.disconnect()


@pytest.fixture(scope="session")
def mock_only(mock):
    """Skips a test during integration testing if it's only valid with mocks
    """
    if not mock:
        pytest.skip("Test is marked for mocking only")


@pytest.fixture(scope="session")
def integration_only(mock):
    if mock:
        pytest.skip("Test is marked for integration only")


def make_scan_init_kwargs(laser_ci_dac, func=lambda r: random.randint(0, 2**r.size - 1)):
    """Used to generate kwargs for ScanEntry objects.
    """
    # generate random entries for all registers
    regs = {name: func(reg) for name, reg in ScanEntry.memmap()}

    # set CI to a very small, but non-zero value so the test is safe
    regs['laser_ci'] = laser_ci_dac.field_unshifted_from_voltage(1)
    regs['sensor_mode'] = 0
    regs['rwin0_l'] = 12
    init_kwargs = regs.copy()
    return init_kwargs, regs


def make_valid_scan_entry(laser_ci_dac, ffuncs, func=lambda r: random.randint(0, 2**r.size - 1)):
    s_row = random.choice(range(480-20))
    mod_freq_int = random.choice(list(MaxRangeIdxMappedOv.MAP.values()))
    inte = random.choice(InteTimeSBv.LIMITS)
    frame = FrameSettings(start_row=s_row,
                          mod_freq_int=mod_freq_int,
                          inte_time_s=inte)
    regs = {name: func(reg) for name, reg in ScanEntry.memmap()}
    flag = 0
    frate = random.choice(FrameRateOv.OPTIONS)

    ci_v = laser_ci_dac.field_unshifted_from_voltage(1)

    for _ in range(8):
        flag |= random.randint(0, 3)
    return ScanEntry.build(
        ffuncs,
        regs['roi_sel'],
        LM10OrderOv.from_field(regs['steering_idx']),
        ci_v,
        ci_v,
        frame,
        regs['virtual_sensor_bitmask'],
        flag,
        2, # binning
        frate,
    )


def make_virtual_sensormeta_init_kwargs(
        func=lambda r: random.randint(0, 2**r.size - 1)):
    """Used to generate kwargs for PerVirtualSensorMetadata objects.
    """
    regs = {name: func(reg) for name, reg in PerVirtualSensorMetadata.memmap()}
    regs['nn_level'] = random.choice(NnLevelOv.OPTIONS)
    regs['binning'] = random.choice(BinningOv.OPTIONS)
    init_kwargs = regs.copy()
    return init_kwargs, regs


def make_staticmeta_init_kwargs(func=lambda r: random.randint(0, 2**r.size - 1)):
    """Used to generate kwargs for StaticMetdata objects.
    """
    regs = {name: func(reg) for name, reg in StaticMetadata.memmap()}
    init_kwargs = regs.copy()
    return init_kwargs, regs
