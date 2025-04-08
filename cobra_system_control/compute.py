"""
file: compute.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines the ComputePlatform class that
provides a driver for the NXP GPIOs. Collects all available
GPIO into one class.
"""
import os
import subprocess

from pathlib import Path

from cobra_system_control import remote


@remote.register_for_serialization
class ComputePlatform:
    """A class to provide information about the NCB version,
    NXP temperature information, and control of the NXP GPIOs
    """
    NXP_TEMPERATURE_FIDS = {
        0: 'cpu_thermal0',
        1: 'cpu_thermal1',
        2: 'gpu_thermal0',
        3: 'gpu_thermal1',
        4: 'drc_thermal0',
        5: 'pmic_thermal0',
    }

    def __init__(self, board_type: str):
        self.os_build_sha = "Not available"
        self.os_build_version = "Not available"
        self.os_build_number = "Not available"
        self.manifest = "Not available"
        self.manifest_sha = "Not available"
        self.cust_layer_sha = "Not applicable/available"
        self.read_version_info()

        self.board_type = board_type

        self.select_i2c_gpio = CpuGpio(3, 19)
        self.clk_sync_gpio = CpuGpio(3, 18)

        self.error_gpio = CpuGpio(7, 24)
        self.interrupt_gpio = CpuGpio(7, 25)

        self.sensor_gpio2 = CpuGpio(7, 26)
        self.sensor_gpio3 = CpuGpio(7, 27)
        self.sensor_gpio4 = CpuGpio(7, 28)
        self.sensor_gpio5 = CpuGpio(7, 29)

        self.trig_in = CpuGpio(8, 10)
        self.trig_out = CpuGpio(8, 11)

        self.pps1_sel = CpuGpio(8, 13)
        #self.pps_in_buf = CpuGpio()
        self.dbg_led0 = CpuGpio(8, 21)
        self.dbg_led1 = CpuGpio(8, 20)
        self.dbg_led2 = CpuGpio(8, 19)
        self.laser_pwr_dwn_gpio = CpuGpio(4, 20)
        self.pgood_5v0_gpio = CpuGpio(4, 15)

        self.power_led = CpuGpio(8, 15)
        self.status_led = CpuGpio(8, 16)
        self.data_led = CpuGpio(8, 17)

        # # These will be controlled by the the DACs so they
        # # turn on when the power turns on
        # self.sensor_24v_en_gpio = DummyDevice()
        # self.sensor_21v_en_gpio = DummyDevice()
        # self.sensor_3v3_en_gpio = DummyDevice()

    def setup(self):
        self.select_i2c_gpio.enable()

    def read_version_info(self):
        """Reads build versioning info which shall be in format
        OS_SHA=<cobra_os SHA>
        BUILD_VERSION=<BUILD_VERSION>
        BUILD_NUMBER=<BUILD_NUMBER>
        BUILD_MANIFEST=<manifest>.xml
        MANIFEST_SHA=<cobra_manifest SHA>
        Also reads customer version info in the format
        BUILD_CUST_LAYER_SHA=<customer layer sha>
        """
        verdict = {}  # version dictionary, not "guilty"

        p = Path("/etc", "lumotive_fs_rev")
        if p.is_file():
            with p.open('rt', encoding='utf8') as f:
                lines = f.read().rstrip().split('\n')
            for line in lines:
                kva = line.split("=")
                if len(kva) == 2:
                    verdict[kva[0]] = kva[1]
            self.os_build_sha = verdict.setdefault('OS_SHA', "not found")
            self.os_build_version = verdict.setdefault('BUILD_VERSION', "not found")
            self.os_build_number = verdict.setdefault('BUILD_NUMBER', "not found")
            self.manifest = verdict.setdefault('BUILD_MANIFEST', "not found")
            self.manifest_sha = verdict.setdefault('MANIFEST_SHA', "not found")

        p = Path("/etc", "cust_fs_rev")
        if p.is_file():
            with p.open('rt', encoding='utf8') as f:
                lines = f.read().rstrip().split('\n')
            for line in lines:
                kva = line.split("=")
                if len(kva) == 2:
                    verdict[kva[0]] = kva[1]
            self.cust_layer_sha = verdict.setdefault('BUILD_CUST_LAYER_SHA', "not found")

    def read_temperatures(self) -> dict:
        rdict = {}
        for k, v in ComputePlatform.NXP_TEMPERATURE_FIDS.items():
            temp = int(subprocess.check_output(f"cat /sys/class/thermal/thermal_zone{k}/temp", shell=True))
            temp /= 1000
            rdict[v] = temp
        return rdict


class CpuGpio:
    """A class to control the NXP GPIO lines
    """
    def __init__(self, chip_num, line_num):
        self.chip_num = chip_num
        self.line_num = line_num

    def connect(self):
        pass

    def setup(self):
        pass

    def pulse(self):
        os.system(f'gpioset {self.chip_num} {self.line_num}=1')
        os.system(f'gpioset {self.chip_num} {self.line_num}=0')
        os.system(f'gpioset {self.chip_num} {self.line_num}=1')

    def sync(self):
        self.pulse()

    def enable(self):
        os.system(f'gpioset {self.chip_num} {self.line_num}=1')

    def disable(self):
        os.system(f'gpioset {self.chip_num} {self.line_num}=0')


def nxp_enable():
    # enable i2c
    si = CpuGpio(3, 19)
    si.enable()

    # enable 3.3
    s3 = CpuGpio(4, 19)
    s3.enable()
