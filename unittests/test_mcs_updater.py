import pytest as pt

from cobra_system_control.mcs_reader import BinMcsReader
from cobra_system_control.mcs_updater import (
    verify_fpga, verify_lcm_tables, sensors_to_upgrade_lcm
    )


def test_verify_fpga(mock, cobra, sensor_head: 'SensorHead'):
    if mock:
        pt.skip('only verify on system')
    mr = BinMcsReader(cobra.fpga_bin_path)
    assert verify_fpga(sensor_head.spi_flash, mr, None)


def test_verify_lcm_tables(mock, sensor_head):
    if mock:
        pt.skip('only verify on system')
    sf = sensor_head.spi_flash

    mr = BinMcsReader(sensor_head.lcm_assembly.lcm_bin_path)
    assert verify_lcm_tables(sf, mr, None, None)


def test_sensors_to_upgrade(mock, cobra):
    if mock:
        pt.skip('only verify on system')
    # Does the function error?
    _ = sensors_to_upgrade_lcm(cobra)
