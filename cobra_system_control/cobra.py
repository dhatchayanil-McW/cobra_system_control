"""
file: cobra.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file defines the Cobra object, the top-level class
for the M30 system. The Sensor Head class and all
peripheral classes are instantiated here. Also included
is connection and updating logic.
"""
import contextlib
import hashlib
from pathlib import Path
import queue
from typing import Optional, ContextManager, Callable
import time

import pandas as pd
import numpy as np

import Pyro5.api
import Pyro5.errors

from cobra_system_control import COBRA_DIR
from cobra_system_control import remote
from cobra_system_control.adcs import Ads7128, nxp_ads7128_channels
from cobra_system_control.cobra_log import log
from cobra_system_control.compute import ComputePlatform, CpuGpio
from cobra_system_control.dacs import (
    TiDac6578, SensorVDac, MchpDacMCP48FVB12,
    ItoDac, LcmVDac,
)
from cobra_system_control.device import I2CBus
from cobra_system_control.exceptions import FPGAFileError
from cobra_system_control.fpga_field_funcs import FpgaFieldFuncs
from cobra_system_control.fpga_adc import FpgaAdc, get_mon_all
from cobra_system_control.fpga_misc import FpgaDbg, ISP
from cobra_system_control.image_reader import TempImageReader
from cobra_system_control.itof import Itof
from cobra_system_control.laser import LaserCiDac, LaserVldaDac
import cobra_system_control.lcm_collateral_md5 as md5lcm
import cobra_system_control.m30_fpga_collateral_md5 as md5m30
from cobra_system_control.mcs_updater import (
    update_fpga, update_lcm,
    sensors_to_upgrade_fpga, sensors_to_upgrade_lcm)
from cobra_system_control.memory_map import (
    M30_FPGA_MEMORY_MAP,
    M30_SPI_FLASH_MEMORY_MAP,
)
from cobra_system_control.metadata import MetadataBuffer
from cobra_system_control.metasurface import LcmController, LcmBuff
from cobra_system_control.scan_control import Scan, ScanParams
from cobra_system_control.sensor_head import SensorHead
from cobra_system_control.spi_flash import QspiController, SpiFlash
from cobra_system_control.temp_sensor import Tmp1075
from cobra_system_control.functional_utilities import (
    get_git_sha, get_git_clean_status,
    get_compute_hostname, free_i2c_bus,
)


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class Cobra:
    """The all-powerful Cobra object
    """
    def __init__(self, whoami: str, board_type: str,
                 msg_queue: Optional[queue.Queue] = None):
        """Sets up the Cobra and Sensor Head objects according to the M30
        sensor head and NCB compute platform.

        Schematic differences to consider

        DACS
        - NCB:
          - 3.3V: 0x48 DAC6578 has 1 DAC control
          - VLDA: 0x48 DAC6578 has 1 DAC control
          - 18V: 0x48 DAC6578 has 1 DAC control
        - M30

        Enables
        - NCB:
          - 3.3V: 435 CPU GPIO EN control
          - VLDA: 433 CPU GPIO EN control
          - 18V: 434 CPU GPIO EN control
        - M30

        Indicators
        - NCB: Power supply LEDs controlled with CPU GPIOs
        - M30: N/A

       ADCS
        - NCB:
          - 3.3V: 0x17 ADS7128 voltage and currs
          - VLDA: 0x17 ADS7128 voltage and currs
          - 18V: 0x17 ADS7128 voltage and currs
          - VIN_V: 0x17 ADS7128 voltage
          - 5V: 0x17 ADS7128 currs (is this really VIN_V_currs?)
        - M30: FPGA ADC

        Constants come from M30 and NCB schematics
        """
        self._whoami = whoami
        self._board_type = board_type

        self._system_version = None

        # Create the I2C buses
        self.cam_i2c_bus_num = 2
        self.cmb_i2c_bus_num = 4
        self.cam_i2c_bus = I2CBus(self.cam_i2c_bus_num)
        self.cmb_i2c_bus = I2CBus(self.cmb_i2c_bus_num)

        self.max_upgrade_tries = 2

        self.msg_queue = msg_queue or queue.Queue()
        self.msg_queue.put(f'{whoami.upper()} and Compute Module {board_type} found')
        self.msg_queue.put('Initializing and connecting to sensor head...')

        fpga_addr = 0x10
        fifo_addr = 0x11
        temp_sensor_addr = 0x49

        cmb_adc_addr = 0x17
        cmb_adc_channels = nxp_ads7128_channels
        cmb_adc_vref = 3.3

        # The NCB DAC adjusts 3.3V (Sensor Head), 18V (LCM), 24V (VLDA)
        cmb_dac_addr = 0x48
        cmb_dac_chip = TiDac6578

        # Sensor Head
        cmb_sensor_v_dac_chip = cmb_dac_chip
        cmb_sensor_v_mmap = None
        cmb_sensor_v_full_scale_pre = 3.3
        cmb_sensor_v_chan = 4
        cmb_sensor_v_gain = 1

        # LCM DAC
        cmb_lcm_v_dac_master = LcmVDac
        cmb_lcm_v_dac_chip = cmb_dac_chip
        cmb_lcm_v_mmap = None
        cmb_lcm_v_full_scale_pre = 3.3
        cmb_lcm_v_gain = 1
        cmb_lcm_v_chan = 2

        # VLDA DAC
        cmb_vlda_dac_chip = cmb_dac_chip
        cmb_vlda_mmap = None
        cmb_vlda_full_scale_pre = 3.3
        cmb_vlda_gain = 1
        cmb_vlda_chan = 0

        self._sensor_3v3_en_gpio = CpuGpio(4, 19)
        self._sensor_21v_en_gpio = CpuGpio(4, 18)
        self._sensor_24v_en_gpio = CpuGpio(4, 17)

        temp_sensor_class = Tmp1075

        self._compute = ComputePlatform(self.board_type)
        log.debug('OS sha= %s', self.compute.os_build_sha)
        log.debug('OS version= %s', self.compute.os_build_version)
        log.debug('OS number= %s',  self.compute.os_build_number)
        log.debug('board_type= %s', self.board_type)

        self.fpga_bin_path = Path(Path(__file__).parent, 'resources',
                                  'm30_fpga_dual_boot.bin').absolute()
        self.golden_sha = md5m30.FPGA_GOLDEN_SHA
        self.released_sha = md5m30.FPGA_RELEASED_SHA
        self.released_bin_md5 = md5m30.BIN_MD5
        self.released_lcm_bin_md5 = md5lcm.BIN_MD5

        self.fpga_mmap = M30_FPGA_MEMORY_MAP
        self.sf_mmap = M30_SPI_FLASH_MEMORY_MAP

        self.fpga_field_funcs = FpgaFieldFuncs(
            memmap_fpga=self.fpga_mmap,
        )
        sh_vlda_dac_chip = MchpDacMCP48FVB12
        sh_vlda_mmap = self.fpga_mmap.dac_spi
        sh_vlda_addr = fpga_addr
        sh_vlda_chan = 0
        sh_vlda_full_scale_pre = 2.44
        sh_vlda_gain = 2

        laser_ci_dac_chip = MchpDacMCP48FVB12
        laser_ci_chan = 1
        laser_ci_full_scale_pre = 2.44  # Gain should be set to 2 for the laser ci dac
        laser_ci_gain = 2

        ito_dac_class = ItoDac
        ito_full_scale_pre = 2.44
        ito_gain = 1

        self._cmb_adc = Ads7128(self.cmb_i2c_bus, cmb_adc_addr, cmb_adc_vref,
                                cmb_adc_channels, board_type=board_type)
        self._temp_sensor = temp_sensor_class(self.cmb_i2c_bus, temp_sensor_addr)

        self._cmb_sensor_v_dac = SensorVDac(
            cmb_sensor_v_dac_chip(
                self.cmb_i2c_bus, cmb_dac_addr, cmb_sensor_v_mmap,
                cmb_sensor_v_full_scale_pre, cmb_sensor_v_gain),
            cmb_sensor_v_chan, self.board_type, self.sensor_3v3_en_gpio)

        scan = Scan(self.cam_i2c_bus, fpga_addr, self.fpga_mmap.scan)

        lcm_ctrl = LcmController(
            self.cam_i2c_bus, fpga_addr, self.fpga_mmap.lcm,
            buff=LcmBuff(self.cam_i2c_bus, fpga_addr, self.fpga_mmap.lcm_buff),
            ffun=self.fpga_field_funcs, scan_device=scan)

        self._cmb_lcm_v_dac = cmb_lcm_v_dac_master(
            cmb_lcm_v_dac_chip(
                self.cmb_i2c_bus, cmb_dac_addr, cmb_lcm_v_mmap,
                cmb_lcm_v_full_scale_pre, cmb_lcm_v_gain),
            cmb_lcm_v_chan, self.board_type, self.sensor_21v_en_gpio, lcm_ctrl)

        self._cmb_laser_vlda_dac = LaserVldaDac(
            cmb_vlda_dac_chip(
                self.cmb_i2c_bus, cmb_dac_addr, cmb_vlda_mmap,
                cmb_vlda_full_scale_pre, cmb_vlda_gain),
            cmb_vlda_chan, self.board_type, self.sensor_24v_en_gpio)

        self.debug = FpgaDbg(self.cam_i2c_bus, fpga_addr, self.fpga_mmap.dbg)

        sh_laser_vlda_dac = LaserVldaDac(
            sh_vlda_dac_chip(
                self.cam_i2c_bus, sh_vlda_addr, sh_vlda_mmap,
                sh_vlda_full_scale_pre, sh_vlda_gain, 0),
            sh_vlda_chan, self.whoami, self.debug)

        laser_ci_dac = LaserCiDac(laser_ci_dac_chip(
            self.cam_i2c_bus, fpga_addr, self.fpga_mmap.dac_spi,
            laser_ci_full_scale_pre, laser_ci_gain, 0), laser_ci_chan, None)

        isp = ISP(self.cam_i2c_bus, fpga_addr, self.fpga_mmap.isp)

        ito_dac = ito_dac_class(
            MchpDacMCP48FVB12(
                self.cam_i2c_bus, fpga_addr, self.fpga_mmap.dac_spi,
                ito_full_scale_pre, ito_gain, 1), 0)

        scan_params = ScanParams(self.cam_i2c_bus, fpga_addr,
                                 self.fpga_mmap.scan_params)

        metabuff = MetadataBuffer(self.cam_i2c_bus, fpga_addr,
                                  self.fpga_mmap.meta_buff, scan)

        qspi = QspiController(self.cam_i2c_bus, fifo_addr, self.fpga_mmap.qspi)
        spi_flash = SpiFlash(qspi, self.sf_mmap, self.fpga_bin_path)
        fpga_adc = FpgaAdc(self.cam_i2c_bus, fifo_addr,
                           self.fpga_mmap.adc, self.whoami)

        itof = Itof(self.cam_i2c_bus, fpga_addr, self.debug,
                    self.fpga_mmap.itof_spi, self.whoami)

        self._sen = SensorHead(
            whoami=self.whoami,
            compute_platform=self._compute,
            debug=self.debug,
            isp=isp,
            itof=itof,
            ito_dac=ito_dac,
            laser_ci_dac=laser_ci_dac,
            cmb_laser_vlda_dac=self.cmb_laser_vlda_dac,
            cmb_lcm_v_dac=self.cmb_lcm_v_dac,
            sh_laser_vlda_dac=sh_laser_vlda_dac,
            lcm_ctrl=lcm_ctrl,
            metabuff=metabuff,
            scan=scan,
            scan_params=scan_params,
            spi_flash=spi_flash,
            fpga_adc=fpga_adc,
            fpga_field_funcs=self.fpga_field_funcs,
        )

        # used to send images over the network with pyro
        self._img_reader = TempImageReader()
        self.clkSync = self.compute.clk_sync_gpio

    def connect(self):
        first_iteration = True
        while True:
            self.cam_i2c_bus.connect()
            self.cmb_i2c_bus.connect()

            self.compute.setup()
            self.compute.select_i2c_gpio.enable()

            self.cmb_sensor_v_dac.connect()
            self.cmb_sensor_v_dac.setup()
            self.cmb_sensor_v_dac.set_voltage(0)
            self.cmb_sensor_v_dac.enable()
            time.sleep(0.5)
            self.cmb_sensor_v_dac.set_voltage(3.4)
            self.compute.power_led.enable()
            time.sleep(0.1)
            self.sen.connect()
            self.sen.debug.setup()

            self.compute.clk_sync_gpio.sync()

            system_type = self.sen.debug.read_fields('project')
            print("System type : ", system_type)
            git_sha = self.sen.debug.read_fields('git_sha')
            print("git_sha : ", git_sha)
            git_sha_memmap = self.sen.debug.read_fields('git_sha_memmap')
            print("git_sha_memmap : ", git_sha_memmap)
            
            log.info('PROJECT=%s, FPGA_SHA=%s, and MAP_SHA=%s.',
                     system_type, git_sha, git_sha_memmap)

            upgrade_fpga, upgrade_lcm = self.check_for_updates(first_iteration)
            if upgrade_fpga or upgrade_lcm:
                if first_iteration:
                    log.debug('Upgrades required and allowed')
                    self.upgrade_sensors(upgrade_fpga, upgrade_lcm)
                    self.msg_queue.put('Updates complete, reconnecting sensor head')
                    # Cannot call self.sen.disable due to the changed SPI master registers
                    # The sequence below repeats self.disconnect() without the sen.disconnect()
                    self.shutdown_sh_power()
                    self.cmb_adc.disconnect()
                    self.cam_i2c_bus.disconnect()
                    self.cmb_i2c_bus.disconnect()
                    first_iteration = False
                    free_i2c_bus()
                    continue
            else:
                if upgrade_fpga:
                    log.warning('FPGAs not upgraded to the approved version because the '
                                'cobra/fpga_upgrade_required cookie does not exist')
                if upgrade_lcm:
                    log.warning('LCM patterns not upgraded to the approved version '
                                'because cobra/lcm_upgrade_required cookie does not exist')
            break
        self.msg_queue.put('System Bootup Complete')

    def check_for_updates(self, first_iteration: bool) -> (bool, bool):
        """Checks whether the FPGA or the LCM patterns need to be updated
        """
        # Report Git SHA and check for Git SHA of golden bitstream
        # Read Git SHA, checking for golden
        _, fpga_is_golden = self.sen.read_git_sha((self.golden_sha,))

        # Update if fpga_upgrade_required or any FPGA has booted into
        # the golden bitstream
        if (COBRA_DIR / 'fpga_upgrade_required').exists() or fpga_is_golden:
            # This assumes self.released_sha != self.golden_sha
            upgrade_fpga = sensors_to_upgrade_fpga(
                self, self.released_sha, first_iteration)
        else:
            upgrade_fpga = False

        if (COBRA_DIR / 'lcm_upgrade_required').exists():
            upgrade_lcm = sensors_to_upgrade_lcm(
                self, first_iteration)
        else:
            upgrade_lcm = False
        return upgrade_fpga, upgrade_lcm

    def upgrade_sensors(self, upgrade_fpga: bool, upgrade_lcm: bool):
        """Determines which fpgas and lcm patterns need an update
        """
        sen = self.sen
        if upgrade_fpga or upgrade_lcm:
            msg = f'Fpga Sensor to update = {upgrade_fpga}'
            log.info(msg)
            self.msg_queue.put(msg)
            msg = f'LCM pattern to update = {upgrade_lcm}'
            log.info(msg)
            self.msg_queue.put(msg)

            self.msg_queue.put(
                f'Expected update time: '
                f'{upgrade_lcm * 1 + upgrade_fpga * 4} minutes')
            self.msg_queue.put('Do not unplug compute module or '
                               'sensor head until updates complete')

            if upgrade_fpga:
                m = hashlib.md5()
                try:
                    with open(self.fpga_bin_path, 'rb') as f:
                        m.update(f.read())
                except OSError as exc:
                    raise FPGAFileError(
                        f'No BIN file found at {self.fpga_bin_path}') from exc
                md5 = m.digest().hex()
                if md5 != self.released_bin_md5:
                    raise FPGAFileError(
                        f'BIN file at {self.fpga_bin_path} has MD5 of {md5} '
                        f'but should be {self.released_bin_md5}')

                msg = 'Updating FPGA on sensor head '
                log.info(msg)
                self.msg_queue.put(msg)

                for tries in range(self.max_upgrade_tries):
                    if not update_fpga(
                            sen.spi_flash, self.fpga_bin_path, self.msg_queue):
                        if (tries < self.max_upgrade_tries - 1):
                            msg = ('Update of sensor head to '
                                   f' {self.released_sha:#010x}'
                                   'failed verification; trying again')
                            log.warning(msg)
                            self.msg_queue.put(msg)
                        else:
                            msg = ('Update of sensor head to '
                                   f' {self.released_sha:#010x}'
                                   'failed verification; '
                                   'sensor will likely be disabled')
                            log.warning(msg)
                            self.msg_queue.put(msg)
                    else:
                        break

            if upgrade_lcm:
                m = hashlib.md5()
                try:
                    with open(sen.lcm_assembly.lcm_bin_path, 'rb') as f:
                        m.update(f.read())
                except OSError as exc:
                    raise FPGAFileError(
                        f'No BIN file found at {sen.lcm_assembly.lcm_bin_path}') from exc
                md5 = m.digest().hex()
                if md5 != self.released_lcm_bin_md5:
                    raise FPGAFileError(
                        f'BIN file at {sen.lcm_assembly.lcm_bin_path} has MD5 of {md5} '
                        f'but should be {self.released_lcm_bin_md5}')

                msg = 'Updating lcm patterns on sensor head '
                log.info(msg)
                self.msg_queue.put(msg)

                for tries in range(self.max_upgrade_tries):
                    if not update_lcm(
                            sen.spi_flash, sen.lcm_assembly.lcm_bin_path, self.msg_queue):
                        if (tries < self.max_upgrade_tries - 1):
                            msg = ('Update of lcm patterns '
                                   'failed verification; trying again')
                            log.warning(msg)
                            self.msg_queue.put(msg)
                        else:
                            msg = ('Update of lcm patterns failed verification'
                                   '; sensor will likely be disabled')
                            log.warning(msg)
                            self.msg_queue.put(msg)
                    else:
                        break

    def setup(self):
        """Perform one-time setup and apply default settings to put the system
        in a known good state.
        """
        self.compute.setup()
        self.cmb_adc.setup()

        # Disable LCM rail on Sensor Head side for safety
        self.sen.lcm_ctrl.write_fields(gpio_pwr_en=0)
        # Disable VLDA on Sensor Head side for safety
        self.sen.debug.write_fields(vlda_en=0)

        # Calibrate CMB
        self.cmb_dac_adc_calibration(
            self.cmb_lcm_v_dac,
            self.cmb_adc.v21_voltage,
            self.sensor_21v_en_gpio,
        )
        self.cmb_dac_adc_calibration(
            self.cmb_laser_vlda_dac,
            self.cmb_adc.v24_voltage,
            self.sensor_24v_en_gpio,
        )

        self.sen.setup()
        self.temp_sensor.setup()

        # Fill the system version dictionary
        self._system_version = self.create_system_version_dict()

    def create_system_version_dict(self) -> dict:
        """Creates a dictionary of various HW and SW versioning
        for querying by the API.
        """
        gsha = self.sen.debug.read_fields('git_sha')
        sys_ver = {}
        sys_ver['fpga_git_sha'] = f"{gsha:08x}"
        sys_ver['manifest_sha'] = self.compute.manifest_sha
        sys_ver['os_build_sha'] = self.compute.os_build_sha
        sys_ver['os_build_version'] = self.compute.os_build_version
        sys_ver['os_build_number'] = self.compute.os_build_number
        sys_ver['cust_layer_sha'] = self.compute.cust_layer_sha
        sys_ver['firmware_sha'] = get_git_sha("cobra_system_control")
        sys_ver['compute_platform'] = self.board_type
        sys_ver['compute_hostname'] = get_compute_hostname()

        sys_ver['sensor_pcb_rev'] = self.sen.rx_pcb_rev
        sys_ver['sensor_part_number'] = '0001179'

        sys_ver['calibration_version'] = self.sen.calibration_version
        return sys_ver

    def cmb_lcm_dac_slope_offset(self) -> tuple:
        """Convenience function to get around Pyro exposure of these
        attributes.
        """
        return self.cmb_lcm_v_dac.slope, self.cmb_lcm_v_dac.offset

    def cmb_laser_vlda_dac_slope_offset(self) -> tuple:
        """Convenience function to get around Pyro exposure of these
        attributes.
        """
        return self.cmb_laser_vlda_dac.slope, self.cmb_laser_vlda_dac.offset

    def cmb_dac_adc_calibration(
            self, vdac: 'VDac', monitor_callable: Callable,
            rail_control: 'CpuGpio'
    ):
        """Calibrates a CMB DAC slope and offset values
        based on the corresponding ADC measurement.

        The Min rail output voltage is with maximum DAC output
        The Max rail output voltage is with minimum DAC output

        The Ti DAC6578 settles in about 10us but due to PCB design
        the rail settles from 24V to 11V in about 350us. Use a 1s sleep.

        The Ti ADS7128 does a new measurement when requested.
        """
        def set_and_get_dac_field_voltage(vdac: 'VDac', dac_code: int):
            """Sets a DAC code a retrieves the resulting field
            and voltage based on the gain and offset for a
            voltage DAC.
            """
            vdac.dac.dac_write(
                'write_update',
                vdac.chan_idx,
                dac_code << vdac.dac.bit_shift
            )
            time.sleep(1)  # let the rail settle
            rfield = vdac.get_field()
            rvoltage = vdac.get_voltage()
            return rfield, rvoltage

        def cmb_adc_dac_loop_and_log(
                vdac: 'VDac', monitor_callable: Callable):
            min_max_v = []
            l_rfield = []
            l_rvoltage = []
            for dac_code in [2**vdac.dac.dac_bits-1, 0]:
                rfield, rvoltage = set_and_get_dac_field_voltage(vdac, dac_code)
                l_rfield.append(rfield)
                l_rvoltage.append(rvoltage)
                l_rdata = []
                for _ in range(10):
                    l_rdata.append(monitor_callable())
                    time.sleep(0.010)  # let the ADC be ready

                rdata_v = np.mean(l_rdata)
                min_max_v.append(rdata_v)

                log.debug(
                    'Before cal for CMB ADC/DAC. %s '
                    'DAC set %0.3f V; '
                    'ADC measure %0.3f V; '
                    'Abs Err %0.4f V; '
                    'Rel Err %0.4f V. ',
                    vdac,
                    rvoltage,
                    rdata_v,
                    rvoltage - rdata_v,
                    abs(1 - rvoltage / rdata_v),
                )
            return min_max_v, l_rfield, l_rvoltage

        rail_control.enable()

        min_max_v, l_rfield, l_rvoltage = cmb_adc_dac_loop_and_log(
            vdac, monitor_callable)

        # Get slope and offset by solving system of equations
        # 3.3V = m * vmin + b
        # 0V = m * vmax + b
        # 3.3V = m * vmin - (m * vmax)
        # b = - (m * vmax)
        m = 3.3 / (min_max_v[0] - min_max_v[1])
        b = -m * min_max_v[1]
        vdac.slope = m
        vdac.offset = b

        log.debug(
            'Slope %0.4f, '
            'Offset %0.4f ',
            vdac.slope,
            vdac.offset
        )

        # Check output after calibration
        for idx in range(2):
            rvoltage_after_cal = vdac.voltage_from_field(l_rfield[idx])
            rvoltage_before_cal = l_rvoltage[idx]

            log.debug(
                'After cal for CMB ADC/DAC. %s '
                'DAC before return %0.3f V; '
                'DAC after return %0.3f V; '
                'NCB expected +24V:[24.0, 10.9], +21V:[21.0, 12.3], +3.3V:[4.0, 2.7];',
                vdac,
                rvoltage_before_cal,
                rvoltage_after_cal,
            )

        rail_control.disable()

    def enable(self):
        self.sen.enable()

    def disable(self):
        self.sen.disable()

    def disconnect(self):
        self.sen.disconnect()
        self.shutdown_sh_power()
        self.cmb_adc.disconnect()
        self.cam_i2c_bus.disconnect()
        self.cmb_i2c_bus.disconnect()

    def shutdown_sh_power(self):
        """Shuts down power to sensor head
        """
        self.sensor_21v_en_gpio.disable()
        self.sensor_24v_en_gpio.disable()
        self.sensor_3v3_en_gpio.disable()
        self.compute.power_led.disable()
        self.compute.select_i2c_gpio.disable()
        time.sleep(1)

    def stop(self):
        self.sen.stop()

    def get_db_vlda_configuration(self) -> pd.DataFrame:
        """Convenience method to return values required by
        lidar system database models.
        """
        d = {}
        d['vlda_v'] = self.cmb_laser_vlda_dac.get_voltage()
        return pd.DataFrame([d])

    def get_db_metadata_configuration(self) -> pd.DataFrame:
        """Convenience method to return values required by
        lidar system database models.
        """
        # From the sensor:
        d = {}
        d.update(self.sen.db_sensor_configuration)
        return pd.DataFrame([d])

    def get_db_software_configuration(self) -> pd.DataFrame:
        """Convenience method to return values required by
        lidar system database models.
        """
        d = {}
        d.update(self.system_version)
        d['cobra_system_control_sha'] = get_git_sha('cobra_system_control')
        d['cobra_system_control_clean'] = get_git_clean_status('cobra_system_control')
        return pd.DataFrame([d])

    def get_db_scan_table(self) -> pd.DataFrame:
        """Convenience method to return values required by
        lidar system database models.
        """
        return self.sen.scan_params.scan_table.to_dataframe()

    def get_db_mon_cmb_all(self) -> pd.DataFrame:
        """Convenience method to return values required by
        lidar system database models.
        """
        d = self.cmb_adc.get_mon_all_channels()
        return pd.DataFrame([d])

    def get_db_mon_sh_fpga_all(self) -> pd.DataFrame:
        """Convenience method to return values required by
        lidar system database models.
        """
        d = get_mon_all(self.sen.fpga_adc)
        return pd.DataFrame([d])

    @classmethod
    @contextlib.contextmanager
    def remote(
            cls, hostname: str = None,
            username: str = "root", password: str = None
    ) -> ContextManager['Cobra']:
        """Acquires remote Cobra object over network via Pyro

        This is the primary access point for remote Cobra objects. The
        Cobra object must be actively hosted on the M-series unit to
        successfully complete this action. Only one context manager
        will be able to use the Cobra object at a time.

        NOTE: be aware that unlike ``Cobra.open()``, which disables
        the system at the end of the context block, calls to
        ``Cobra.remote()`` defaults to leave the system
         in the exact state the caller leaves it in.

        Typical use:

        .. code:: python
            from cobra_system_control.cobra import Cobra

            with Cobra.remote() as c:
                c.apply_settings(cobra_settings)
                c.enable()
                c.start()

            ... do some things

                c.stop()
        """
        # shortcut control of Cobra provided through Pyro5
        with remote.remote_lookup(
                remote.COBRA_ID, hostname=hostname,
                username=username, password=password) as c:
            yield c

    @classmethod
    @contextlib.contextmanager
    def open(cls, system_type: str = 'm30', board_type: str = 'nxp',
             msg_queue: queue.Queue = None,
             ) -> ContextManager['Cobra']:
        """Instantiates resource and initializes
        resource communications within a context block.

        Appropriately handles exceptions raised while
        manipulating returned ContextManager object.

        Returns:
            (ContextManager[LidarResource]) context manager for system

        Typical use:

        .. code:: python
            from cobra_system_control.cobra import Cobra

            # calls cob.disable() at block end during unhandled exception
            with Cobra.open() as cob:

        """
        system = cls(system_type, board_type=board_type, msg_queue=msg_queue)

        system.connect()  # connects to hardware
        system.setup()
        system.enable()

        try:
            yield system
        except KeyboardInterrupt:
            log.debug('Aborting script early due to keyboard interrupt')
        # Going to catch all exceptions and report them. We shouldn't launch
        # if there are any errors
        # pylint: disable-next=broad-exception-caught
        except Exception as e:
            log.error('An unexpected exception was raised: %s', e)
            raise e
        finally:
            system.disable()
            system.disconnect()
            system.cam_i2c_bus.disconnect()  # needed to avoid segv

    @property
    def sen(self):
        return self._sen

    @property
    def sensor_3v3_en_gpio(self):
        return self._sensor_3v3_en_gpio

    @property
    def sensor_21v_en_gpio(self):
        return self._sensor_21v_en_gpio

    @property
    def sensor_24v_en_gpio(self):
        return self._sensor_24v_en_gpio

    @property
    def cmb_laser_vlda_dac(self):
        return self._cmb_laser_vlda_dac

    @property
    def cmb_sensor_v_dac(self):
        return self._cmb_sensor_v_dac

    @property
    def cmb_lcm_v_dac(self):
        return self._cmb_lcm_v_dac

    @property
    def img_reader(self):
        return self._img_reader

    @property
    def cmb_adc(self):
        return self._cmb_adc

    @property
    def temp_sensor(self):
        return self._temp_sensor

    @property
    def compute(self):
        return self._compute

    @property
    def board_type(self):
        return self._board_type

    @property
    def whoami(self):
        return self._whoami

    @property
    def system_version(self):
        return self._system_version
