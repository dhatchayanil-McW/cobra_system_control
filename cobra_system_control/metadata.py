"""
file: metadata.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provides classes that define the locations
and sizes of the Static and PerVirtualSensor
metadata used by RawToDepth.
"""
from collections.abc import ItemsView
import inspect
import random
import struct
from typing import Sequence, Optional, List

import pandas as pd

from cobra_system_control.device import Device
from cobra_system_control.itof import StartRow, FovRowsOv
from cobra_system_control import remote
from cobra_system_control.numerical_utilities import ptob_raw12
from cobra_system_control.validation_utilities import Register


NUM_VIRTUAL_SENSOR = 8


@remote.register_for_serialization
class PerVirtualSensorMetadata:
    """A class to aggregate metadata for a single Virtual Sensor.
    """
    RAW12_WORDS = 13

    user_tag = Register(offset=0, position=0, size=12)
    binning = Register(offset=1, position=0, size=12)
    nn_level = Register(offset=2, position=0, size=3)
    s_rows = Register(offset=3, position=0, size=12)
    n_rows = Register(offset=4, position=0, size=12)
    n_rois = Register(offset=5, position=0, size=12)
    rtd_algorithm_common = Register(offset=6, position=0, size=12)
    snr_threshold = Register(offset=7, position=0, size=12)

    random_virtual_sensor_tag = Register(offset=10, position=0, size=12)
    rtd_algorithm_grid_mode = Register(offset=11, position=0, size=12)
    rtd_algorithm_stripe_mode = Register(offset=12, position=0, size=12)

    def __init__(
            self, *,
            user_tag: int, binning: int,
            s_rows: int, n_rows: int, n_rois: int,
            rtd_algorithm_common: int,
            rtd_algorithm_grid_mode: int,
            rtd_algorithm_stripe_mode: int,
            snr_threshold: int,
            nn_level: int,
            random_virtual_sensor_tag: int,
    ):
        self.user_tag = user_tag
        self.binning = binning
        self.s_rows = s_rows
        self.n_rows = n_rows
        self.n_rois = n_rois
        self.rtd_algorithm_common = rtd_algorithm_common
        self.snr_threshold = snr_threshold
        self.nn_level = nn_level
        self.rtd_algorithm_grid_mode = rtd_algorithm_grid_mode
        self.rtd_algorithm_stripe_mode = rtd_algorithm_stripe_mode
        self.random_virtual_sensor_tag = random_virtual_sensor_tag

    @classmethod
    def memmap(cls) -> ItemsView:
        """Returns the mapping of {reg_name -> Register} for each register
        descriptor
        """
        return {n: p for n, p in cls.__dict__.items() if
                isinstance(p, Register)}.items()

    @property
    def data_words(self) -> Sequence[int]:
        """Returns (15 1-byte words) of the PerVirtualSensor metadata in
        order, packed as RAW12. There are 32, 12-bit RAW12 words.
        """
        raw_words = [0] * PerVirtualSensorMetadata.RAW12_WORDS  # 12-bits each
        for name, param_def in PerVirtualSensorMetadata.memmap():
            val = getattr(self, name)
            raw_words[param_def.offset] |= (val << param_def.position)
        bytewords = ptob_raw12(raw_words)
        return bytewords

    @classmethod
    def empty(cls) -> 'PerVirtualSensorMetadata':
        """Returns PerVirtualSensorMetadata instance with all zero values
        """
        return cls(user_tag=0, binning=0, s_rows=0, n_rows=0, n_rois=0,
                   rtd_algorithm_common=0,
                   rtd_algorithm_grid_mode=0,
                   rtd_algorithm_stripe_mode=0,
                   snr_threshold=0, nn_level=0,
                   random_virtual_sensor_tag=0)

    @classmethod
    def empty_array(cls) -> Sequence['PerVirtualSensorMetadata']:
        return [PerVirtualSensorMetadata.empty()] * 8

    @classmethod
    def build(cls,
              user_tag: int,
              binning: 'BinningOv',
              s_rows: StartRow, n_rows: FovRowsOv, n_rois: int,
              rtd_algorithm_common: int,
              rtd_algorithm_grid_mode: int,
              rtd_algorithm_stripe_mode: int,
              snr_threshold: 'SnrThresholdBv',
              nn_level: 'NnLevelOv',
              random_virtual_sensor_tag: int = None,
    ) -> 'PerVirtualSensorMetadata':
        return cls(
            user_tag=user_tag, binning=binning.field,
            s_rows=s_rows, n_rows=n_rows, n_rois=n_rois,
            rtd_algorithm_common=rtd_algorithm_common,
            rtd_algorithm_grid_mode=rtd_algorithm_grid_mode,
            rtd_algorithm_stripe_mode=rtd_algorithm_stripe_mode,
            snr_threshold=snr_threshold.field,
            nn_level=nn_level.field,
            random_virtual_sensor_tag=random_virtual_sensor_tag or random.randrange(2**12-1),
        )


def print_virtual_sensor_metadata(
        virtual_sensor_metadata: List[PerVirtualSensorMetadata]):
    cols = inspect.getfullargspec(PerVirtualSensorMetadata).kwonlyargs
    df = pd.DataFrame(columns=cols)
    for i, e in enumerate(virtual_sensor_metadata):
        data = {col: getattr(e, col) for col in cols}
        df.loc[i] = data
    print(df.to_string(index=False))


@remote.register_for_serialization
class StaticMetadata:
    """A class to aggregate metadata common across all Virtual Sensors.

    Static metadata currently can occupy the pixels
    between 48 and 100

    """
    RAW12_WORDS = 26

    # Starts at pixel offset 48
    rtd_output = Register(offset=0, position=0, size=1)
    reduce_mode = Register(offset=1, position=0, size=1)
    sensor_sn = Register(offset=2, position=0, size=12)
    test_mode = Register(offset=3, position=0, size=2)
    quant_mode = Register(offset=4, position=0, size=2)
    mipi_raw_mode = Register(offset=5, position=0, size=3)
    hdr_threshold = Register(offset=6, position=0, size=12)
    system_type = Register(offset=7, position=0, size=12)
    rx_pcb_type = Register(offset=8, position=0, size=12)
    tx_pcb_type = Register(offset=9, position=0, size=12)
    lcm_type = Register(offset=10, position=0, size=12)
    range_cal_offset_mm_lo_0807 = Register(offset=11, position=0, size=12)
    range_cal_offset_mm_hi_0807 = Register(offset=12, position=0, size=12)
    range_cal_mm_per_volt_lo_0807 = Register(offset=13, position=0, size=12)
    range_cal_mm_per_volt_hi_0807 = Register(offset=14, position=0, size=12)
    range_cal_mm_per_celsius_lo_0807 = Register(offset=15, position=0, size=12)
    range_cal_mm_per_celsius_hi_0807 = Register(offset=16, position=0, size=12)
    range_cal_offset_mm_lo_0908 = Register(offset=17, position=0, size=12)
    range_cal_offset_mm_hi_0908 = Register(offset=18, position=0, size=12)
    range_cal_mm_per_volt_lo_0908 = Register(offset=19, position=0, size=12)
    range_cal_mm_per_volt_hi_0908 = Register(offset=20, position=0, size=12)
    range_cal_mm_per_celsius_lo_0908 = Register(offset=21, position=0, size=12)
    range_cal_mm_per_celsius_hi_0908 = Register(offset=22, position=0, size=12)
    adc_cal_gain = Register(offset=23, position=0, size=12)
    adc_cal_offset = Register(offset=24, position=0, size=12)
    random_scan_table_tag = Register(offset=25, position=0, size=12)

    def __init__(self, *,
                 rtd_output: int,
                 reduce_mode: int,
                 sensor_sn: int,
                 test_mode: int,
                 quant_mode: int,
                 mipi_raw_mode: int,
                 hdr_threshold: int,
                 system_type: int,
                 rx_pcb_type: int,
                 tx_pcb_type: int,
                 lcm_type: int,
                 range_cal_offset_mm_lo_0807: int,
                 range_cal_offset_mm_hi_0807: int,
                 range_cal_mm_per_volt_lo_0807: int,
                 range_cal_mm_per_volt_hi_0807: int,
                 range_cal_mm_per_celsius_lo_0807: int,
                 range_cal_mm_per_celsius_hi_0807: int,
                 range_cal_offset_mm_lo_0908: int,
                 range_cal_offset_mm_hi_0908: int,
                 range_cal_mm_per_volt_lo_0908: int,
                 range_cal_mm_per_volt_hi_0908: int,
                 range_cal_mm_per_celsius_lo_0908: int,
                 range_cal_mm_per_celsius_hi_0908: int,
                 adc_cal_gain: int,
                 adc_cal_offset: int,
                 random_scan_table_tag: int = None,
    ):
        self.rtd_output = rtd_output
        self.reduce_mode = reduce_mode
        self.sensor_sn = sensor_sn
        self.test_mode = test_mode
        self.quant_mode = quant_mode
        self.mipi_raw_mode = mipi_raw_mode
        self.hdr_threshold = hdr_threshold
        self.system_type = system_type
        self.rx_pcb_type = rx_pcb_type
        self.tx_pcb_type = tx_pcb_type
        self.lcm_type = lcm_type

        self.range_cal_offset_mm_lo_0807 = range_cal_offset_mm_lo_0807
        self.range_cal_offset_mm_hi_0807 = range_cal_offset_mm_hi_0807
        self.range_cal_mm_per_volt_lo_0807 = range_cal_mm_per_volt_lo_0807
        self.range_cal_mm_per_volt_hi_0807 = range_cal_mm_per_volt_hi_0807
        self.range_cal_mm_per_celsius_lo_0807 = range_cal_mm_per_celsius_lo_0807
        self.range_cal_mm_per_celsius_hi_0807 = range_cal_mm_per_celsius_hi_0807
        self.range_cal_offset_mm_lo_0908 = range_cal_offset_mm_lo_0908
        self.range_cal_offset_mm_hi_0908 = range_cal_offset_mm_hi_0908
        self.range_cal_mm_per_volt_lo_0908 = range_cal_mm_per_volt_lo_0908
        self.range_cal_mm_per_volt_hi_0908 = range_cal_mm_per_volt_hi_0908
        self.range_cal_mm_per_celsius_lo_0908 = range_cal_mm_per_celsius_lo_0908
        self.range_cal_mm_per_celsius_hi_0908 = range_cal_mm_per_celsius_hi_0908

        self.adc_cal_gain = adc_cal_gain
        self.adc_cal_offset = adc_cal_offset

        self.random_scan_table_tag = random_scan_table_tag or random.randrange(2**12-1)

    def __str__(self) -> str:
        cols = inspect.getfullargspec(StaticMetadata).kwonlyargs
        df = pd.DataFrame(columns=cols)
        data = {col: getattr(self, col) for col in cols}
        df.loc[0] = data
        out = df.transpose().to_string(index=True)
        if out is not None:
            return str(out)
        else:
            return ""

    @classmethod
    def memmap(cls) -> ItemsView:
        """Returns the mapping of {reg_name -> Register} for each register
        descriptor
        """
        return {n: p for n, p in cls.__dict__.items() if
                isinstance(p, Register)}.items()

    @property
    def data_words(self) -> Sequence[int]:
        """Returns (8 1-byte words) of the static metadata in order, packed as RAW12
        There are 7, 12-bit RAW12 words.
        """
        raw_words = [0] * StaticMetadata.RAW12_WORDS  # 12-bits each
        for name, param_def in StaticMetadata.memmap():
            val = getattr(self, name)
            raw_words[param_def.offset] |= (val << param_def.position)
        bytewords = ptob_raw12(raw_words)
        return bytewords


class MetadataBuffer(Device):
    """A class to interface with the FPGA metabuff peripheral.

    Handles writing static metadata to the metadata buffer.
    """
    ALL_BYTES = 2880  # 1920 * 12bits / 8
    STATIC_BYTES = 39  # 26 * 12bits / 8
    VIRTUAL_SENSOR_BYTES = 21  # 13 * 12bits / 8
    DYNAMIC_BYTES = 60

    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph', lmmi_scan: 'Scan'):
        super().__init__(bus, device_addr, 2, 1,
                         memmap_periph,
                         addr_bigendian=True, data_bigendian=False)
        self._virtual_sensor_metadata: Optional[Sequence['PerVirtualSensorMetadata']] = None
        self._static_metadata: Optional['StaticMetadata'] = None
        self._lmmi_scan = lmmi_scan
        self.memmap_periph = memmap_periph

    def __str__(self):
        # Static Metadata
        cols = inspect.getfullargspec(StaticMetadata).kwonlyargs
        df = pd.DataFrame(columns=cols)
        data = {col: getattr(self.static_metadata, col) for col in cols}
        df.loc[0] = data
        sms = df.to_string(index=False)

        # VIRTUAL_SENSOR metadata
        cols = inspect.getfullargspec(PerVirtualSensorMetadata).kwonlyargs
        df = pd.DataFrame(columns=cols)
        for i, e in enumerate(self.virtual_sensor_metadata):
            data = {col: getattr(e, col) for col in cols}
            df.loc[i] = data
        fms = df.to_string(index=False)
        return f'{sms} \n {fms}'

    @property
    def virtual_sensor_metadata(self):
        return self._virtual_sensor_metadata

    @property
    def static_metadata(self):
        return self._static_metadata

    def get_all_metadata_buffer_data(self):
        """Reads the metadata buffer, retrieving both the
        static and virtual_sensor sections
        """
        return self.get_metadata_buffer_data(
            'dynamic', MetadataBuffer.ALL_BYTES)

    def get_static_buffer_data(self):
        """Reads the metadata buffer for the static metadata
        """
        return self.get_metadata_buffer_data(
            'static', MetadataBuffer.STATIC_BYTES)

    def get_virtual_sensor_buffer_data(self, idx: int):
        """Reads the metadata buffer for the chosen VIRTUAL_SENSOR [0,8)
        """
        if idx not in range(NUM_VIRTUAL_SENSOR):
            raise ValueError(f'PerVirtualSensor buffer index invalid {idx}')
        return self.get_metadata_buffer_data(
            f'virtual_sensor_{idx}', MetadataBuffer.VIRTUAL_SENSOR_BYTES)

    def get_dynamic_buffer_data(self):
        """Reads the metadata buffer for the virtual_sensor metadata
        """
        return self.get_metadata_buffer_data(
            'dynamic', MetadataBuffer.DYNAMIC_BYTES)

    def get_metadata_buffer_data(self, field, nbytes):
        self._lmmi_scan.write_fields(scan_lmmi_meta_en=1)
        faddr = self.periph.get_field_addr(field)
        addr = bytearray(struct.pack(self.i2c.addr_pack, faddr))
        ba = self.i2c.bus.read(self.i2c.device_addr, addr, nbytes)
        self._lmmi_scan.write_fields(scan_lmmi_meta_en=0)
        return ba

    def write_metadata_buffer_virtual_sensor_data(
            self, virtual_sensor_metadata_list: List['PerVirtualSensorMetadata']):
        """Packs PerVirtualSensorMetadata information into RAW12
        and writes the metadata buffer
        """
        self._lmmi_scan.write_fields(scan_lmmi_meta_en=1)
        for i, virtual_sensor in enumerate(virtual_sensor_metadata_list):
            addr = self.get_abs_addr(f'virtual_sensor_{i}')
            ba = bytearray([*struct.pack(self.i2c.addr_pack, addr)])
            for d in virtual_sensor.data_words:
                ba.extend(struct.pack(self.i2c.data_pack, d))
            self.i2c.bus.write(self.i2c.device_addr, ba)
        self._lmmi_scan.write_fields(scan_lmmi_meta_en=0)
        self._virtual_sensor_metadata = virtual_sensor_metadata_list

    def write_metadata_buffer_static(self, static_metadata: 'StaticMetadata'):
        """Packs the StaticMetadata into RAW12 and writes
        to the metadata buffer
        """
        self._lmmi_scan.write_fields(scan_lmmi_meta_en=1)
        addr = self.get_abs_addr('static')
        ba = bytearray([*struct.pack(self.i2c.addr_pack, addr)])
        for d in static_metadata.data_words:
            ba.extend(struct.pack(self.i2c.data_pack, d))
        self.i2c.bus.write(self.i2c.device_addr, ba)
        self._lmmi_scan.write_fields(scan_lmmi_meta_en=0)
        self._static_metadata = static_metadata
