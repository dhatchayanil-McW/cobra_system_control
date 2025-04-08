"""
file: monitor.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file runs as a systemd service to continually poll the
Cobra object for the relevant monitoring values and save them
as an entry in the database. The values are also printed to
a file at OUTPUT_PATH. Currently, the polling occurs
every QUERY_PERIOD_S seconds.
The database models are defined in models.py
"""
import datetime
import pathlib
import signal
import time
import sys
from typing import Type

import pandas as pd
from peewee import Model
import Pyro5.errors

from cobra_system_control.cobra import Cobra
from cobra_system_control.cobra_log import log
from cobra_system_control.models import ComputeModuleDump, SensorHeadDump
from cobra_system_control.state import State
from cobra_system_control.fpga_adc import get_mon_all


OUTPUT_PATH = pathlib.Path('/run/lumotive/monitor')
QUERY_PERIOD_S = 5.0  # seconds
MAX_ENTRIES = 100000
DELETE_COUNT = 100


def rotate(dump: Type[Model]):
    if dump.select().count() > MAX_ENTRIES:
        query = (dump
                 .select()
                 .where(dump.idn > 0)
                 .limit(DELETE_COUNT)
                 .order_by(dump.idn))
        for dump in query:
            dump.delete_instance()
        log.info('Deleted %s entries (IDNs %s - %s) from %s',
                 DELETE_COUNT, query[0].idn, query[-1].idn,
                 dump.__class__.__name__)


def sigterm_handler(signo, stack_frame):
    """Exits, raising SystemExit(0)"""
    print('Raising SystemExit(0)')
    raise SystemExit(0)


def write_status_file(msg: str):
    with open(OUTPUT_PATH, 'w') as fid:
        fid.write(msg)


def dump_status(c: Cobra):
    # --- CMB Monitoring ---

    cmb_status = {}
    out_str = f'\nUpdated: {datetime.datetime.now()}\n\n'

    cmb_status.update(c.cmb_adc.get_mon_all_channels())
    cmb_status.update(c.compute.read_temperatures())
    slope, offset = c.cmb_laser_vlda_dac_slope_offset()
    cmb_status.update(cmb_24v_dac_slope=slope)
    cmb_status.update(cmb_24v_dac_offset=offset)
    slope, offset = c.cmb_lcm_dac_slope_offset()
    cmb_status.update(cmb_21v_dac_slope=slope)
    cmb_status.update(cmb_21v_dac_offset=offset)

    # save cmb status
    cmb_dump = ComputeModuleDump(**cmb_status)
    cmb_dump.save()
    rotate(ComputeModuleDump)

    out_str += '--- Compute Module Board ---\n\n'
    out_str += pd.DataFrame([cmb_status]).T.to_string(na_rep="", float_format='{:.3f}'.format)

    # --- Sensor Head Monitoring ---

    # used later by dataframe to nicely print to file
    sh_status_dicts = []

    sh = c.sen
    # record channel ID and state
    sh_status = {'rx_pcb_rev': sh.rx_pcb_rev,
                 'state': State(sh.state).name}

    # get the sensor serial number
    sh_status['sensor_id'] = int(sh.serial_number())

    # record ADC values
    sh_status.update(get_mon_all(sh.fpga_adc))
    gain, offset = sh.fpga_adc_gain_offset()
    sh_status.update(fpga_adc_cal_gain=gain)
    sh_status.update(fpga_adc_cal_offset=offset)

    # save to database
    sh_dump = SensorHeadDump(**sh_status)
    sh_dump.save()

    # append status dicts and additional info to print later
    sh_status_dicts.append(sh_status)

    rotate(SensorHeadDump)

    df = pd.DataFrame.from_records(sh_status_dicts, )
    out_str += '\n\n--- Sensor Head ---\n\n'
    out_str += df.T.to_string(
        na_rep="",
        float_format="{:.5f}".format,
    )

    # --- Write to file ---
    write_status_file(out_str)


if __name__ == '__main__':

    signal.signal(signal.SIGTERM, sigterm_handler)
    signal.signal(signal.SIGINT, sigterm_handler)

    if not OUTPUT_PATH.exists():
        write_status_file('Waiting for status...')

    try:
        with Cobra.remote() as c_:
            while True:
                dump_status(c_)
                time.sleep(QUERY_PERIOD_S)
    except Pyro5.errors.NamingError:
        log.error(
            'Pyro5.errors.NamingError: Monitor trying to access the remote '
            'object but it is not yet available. You may see this error '
            'repeated until the remote object is launched')
        sys.exit(1)
    except Exception as exc:
        raise exc
