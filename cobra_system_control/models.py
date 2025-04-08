"""
file: models.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file defines the models used for the Sqlite database
of the monitoring values that is saved
on the Compute Module (NCB).
"""
import datetime
import os

from cobra_system_control.adcs import nxp_ads7128_channels
from cobra_system_control.compute import ComputePlatform
from cobra_system_control.fpga_adc import MON_POSSIBLE_CHAN
from cobra_system_control import COBRA_DIR

from peewee import (
    # PostgresqlDatabase, BooleanField
    FloatField, Model, AutoField, DateTimeField,
    TextField, IntegerField, SqliteDatabase
)

DB_NAME = 'lidar_monitors.db'
db = SqliteDatabase(os.path.join(COBRA_DIR, DB_NAME))


class ComputeModuleDump(Model):
    """A class to set up the database model for the Compute
    Module measurements.
    """
    idn = AutoField(primary_key=True)
    timestamp = DateTimeField(default=datetime.datetime.now)

    class Meta:
        database = db


class SensorHeadDump(Model):
    """A class to set up the database model for the Sensor
    Head measurements.
    """
    idn = AutoField(primary_key=True)
    timestamp = DateTimeField(default=datetime.datetime.now)
    sensor_id = IntegerField(null=False)  # serial number
    state = TextField(null=False)

    class Meta:
        database = db


# Correct way to add fields is through the _meta attribute
# pylint: disable=no-member
# pylint: disable=protected-access
for k, v in ComputePlatform.NXP_TEMPERATURE_FIDS.items():
    ComputeModuleDump._meta.add_field(v, FloatField(null=False))

for c in nxp_ads7128_channels:
    ComputeModuleDump._meta.add_field(c.channel_name, FloatField(null=False))

# Add the power fields
for ch in ('v3p3_power_cb', 'v21p0_power_cb', 'v24p0_power_cb', 'vin_power_cb'):
    ComputeModuleDump._meta.add_field(ch, FloatField(null=False))

# Add the 24V rail slope and offset
ComputeModuleDump._meta.add_field('cmb_24v_dac_slope', FloatField(null=False))
ComputeModuleDump._meta.add_field('cmb_24v_dac_offset', FloatField(null=False))
ComputeModuleDump._meta.add_field('cmb_21v_dac_slope', FloatField(null=False))
ComputeModuleDump._meta.add_field('cmb_21v_dac_offset', FloatField(null=False))

for ch in MON_POSSIBLE_CHAN:
    SensorHeadDump._meta.add_field(ch, FloatField(null=False))

# Add in rx_pcb_rev since it was removed from FPGA_ADC
SensorHeadDump._meta.add_field('rx_pcb_rev', IntegerField(null=False))

# Add the cal_gain and cal_offset values
SensorHeadDump._meta.add_field('fpga_adc_cal_gain', FloatField(null=False))
SensorHeadDump._meta.add_field('fpga_adc_cal_offset', FloatField(null=False))

# pylint: enable=no-member
# pylint: enable=protected-access


db.connect()
db.create_tables([ComputeModuleDump, SensorHeadDump])
db.close()
