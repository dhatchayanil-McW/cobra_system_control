"""
file: adcs.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file is a driver for the Compute Module (NCB)
TI ADS7128 ADC. The driver provides functions to
configure the ADC, read the ADC, and do value
conversions based on the PCB schematic.
"""
import dataclasses as dc
import time
from copy import deepcopy
from typing import List, Union

from cobra_system_control.device import Device


I2C_OPCODES = {
    "RD_SING_REG": 0b0001_0000,  # Read single register
    "WR_SING_REG": 0b0000_1000,  # Write single register
    "RD_CONT_REG": 0b0011_0000,  # Read continuous block of registers (unused)
    "WR_CONT_REG": 0b0010_1000,  # Write continuous block of registers (unused)
    "SET_BIT": 0b0001_1000,  # Set a single bit (unused)
    "CLR_BIT": 0b0011_0000,  # Clear a single bit (unused)
}


@dc.dataclass
class AdsChannel:
    """Class to define attributes of each channel to the
    ADS7128. All channels are configured as analog in

    Slope and offset values satisfy the equations:

    For current measurements:
    I[A] = slope * (adc_volts) + offset
    The ADC measures in volts and the relation needs
    to be converted to current in units of Amps.

    For voltage measurements:
    note that the offset is added t other adc_volts
    before being scaled by the slope
    V[V] = slope * (adc_volts + offset)
    """
    id: int
    channel_name: str
    slope: float
    offset: float


nxp_ads7128_channels = [
    # For current measurements:
    # I[A] = slope * (adc_volts) + offset
    # The ADC measures in volts and the relation needs
    # to be converted to current in units of Amps.
    #
    # For voltage measurements:
    # note that the offset is added to other adc_volts
    # before being scaled by the slope
    # V[V] = slope * (adc_volts + offset)


    # The 3.3V rail is a direct input to the ADC on Rev1.
    # We do not have detection of revision on the PCB
    # The 3.3V rail has a 0.5 voltage divider on Rev2.
    # Let's assume a Rev2 here. Later, we can tell if
    # its a Rev1 if the ADC measures ~6V.
    AdsChannel(0, "v3p3_volts_cb", 2, 0),

    # Current scales by V/A
    AdsChannel(1, "v3p3_currs_cb", 1, 0),

    # Voltage divider = 10/(10+82.5)
    AdsChannel(2, "v21p0_volts_cb", 9.25, 0),

    # V/A, ISMON: 1A = 1V+0.25V offset.
    # 0.25V needs to be subtracted from the ADC measure
    AdsChannel(3, "v21p0_currs_cb", 1, -0.25),

    # Voltage divider = 10/(10+82.5)
    AdsChannel(4, "v24p0_volts_cb", 9.25, 0),

    # V/A, ISMON: 1A = 1V+0.25V offset.
    # 0.25V needs to be subtracted from the ADC measure
    AdsChannel(5, "v24p0_currs_cb", 1, -0.25),

    # voltage divider = 10/(10+120)
    AdsChannel(6, "vin_volts_cb", 13, 0),

    # A, Iin-out[A] = 3 * Vseti / Rseti[koHm] = 3 * vsense / 1.5
    # This channel is mislabeled as "CurrS_5.0" on the NCB schematic
    AdsChannel(7, "vin_currs_cb", 2, 0),
]


class Ads7128(Device):
    """Controls an ADS7128 peripheral

    For connect, there are no write and read callbacks
    specified and read_fields/write_fields are not allowed.
    because this creates exploding peripheral callbacks on a now
    unsupported platform.
    """
    # V_ref = AVDD
    CAL_DELAY_S = 0.01  # wait 10ms for calibration to complete
    CHANNEL_DWELL_TIME = 0.001
    # We are now using bare writes because I2C read is not currently
    # working properly on Bronco B due to the I2C mux
    REG_GENERAL_CFG = 0x1
    REG_DATA_CFG = 0x2
    REG_OSR_CFG = 0x3
    REG_PIN_CFG = 0x5
    REG_GPIO_CFG = 0x7
    REG_GPO_DRIVE_CFG = 0x9
    REG_GPO_VALUE = 0xb
    REG_CHANNEL_SEL = 0x11

    def __init__(self, bus: 'I2CBus', device_addr: int, vref: float,
                 ads_channels: List[AdsChannel], board_type):
        super().__init__(
            bus=bus,
            device_addr=device_addr,
            addr_bytes=1,
            data_bytes=1,
            mmap_periph=None,
            addr_bigendian=True,
            data_bigendian=True,
        )
        self.device_addr = device_addr
        self.vref = vref
        self.nbits = 12
        self.lsb_voltage = self.vref / 2**self.nbits
        # Don't modify the original specifications - keep internal ref only
        self.ads_channels = deepcopy(ads_channels)
        self._is_enabled = False

        self._name_to_ch = {ch.channel_name: ch.id for ch in self.ads_channels}
        self.board_type = board_type

    def adc_digital_to_volts(self, adc_rval: int):
        """Scales the digital ADC value to a voltage
        based on the bit depth and Vref of the ADC.
        """
        return ((adc_rval >> 4) & 0xfff) * self.lsb_voltage

    def scale_adc_voltage(self, adc_voltage: float, channel_data: AdsChannel):
        """Scales the measured ADC voltage by a slope
        and offset to return a current or voltage
        measurement.

        The offset is added to the value first due to how the current
        monitoring on the voltage regulators work. The voltage rail
        measurements do not have an offset so this implementation
        is okay for both.
        """
        adc_voltage += channel_data.offset
        adc_voltage *= channel_data.slope
        return adc_voltage

    def write(self, addr, data):
        """Write callback for the ADS7128 single register write protocol.

        The manual (pg. 28, section 8.5.2.1) specifies the master must
        provide an I2C command with four frames - the write command,
        the write opcode, the register address, and the register data.
        """
        self.i2c.bus.write(
            self.i2c.device_addr,
            bytearray([I2C_OPCODES["WR_SING_REG"], addr, data])
        )

    def read(self, addr) -> int:
        """Read register for the ADS7128 single register read protocol.

        The manual (pg. 27, section 8.5.1.1) specifies the master must
        provide an I2C command with three frames - the write command,
        the read opcode, and the register address. After this, then another
        frame can be sent to get the data.
        """
        ret = self.i2c.bus.read(
            self.i2c.device_addr,
            bytearray([I2C_OPCODES["RD_SING_REG"], addr]),
            self.i2c.data_nbytes
        )
        return int.from_bytes(ret, byteorder="big", signed=False)

    def bare_read(self) -> int:
        ret = self.i2c.bus.read(
            self.device_addr,
            bytearray(2), 2,
        )
        return int.from_bytes(ret, byteorder="big", signed=False)

    @property
    def is_enabled(self):
        return self._is_enabled

    def setup(self):
        # We are doing bare writes because the I2C read doesn't work
        # properly on deprecated HW

        # Reset the device and wait 10 ms (reset takes 5ms)
        self.write(Ads7128.REG_GENERAL_CFG, 0x1)
        time.sleep(0.01)
        # Calibrate the ADC offset
        self.calibrate()

        # (opt) Append channel id to output data
        self.write(Ads7128.REG_DATA_CFG, 0x10)

    def enable(self):
        pass

    def disable(self):
        pass

    def calibrate(self):
        """Calibrates the variation in the ADC offset error resulting from
        changes in temperature or AVDD.

        Doing bare writes so need to make sure we
        1. Leave the stats_en on
        2. Leave all channels configured as analog inputs
        """
        self.write(Ads7128.REG_GENERAL_CFG, 0b100110)
        time.sleep(Ads7128.CAL_DELAY_S)

    def get_ch_id_from_ch_name(self, channel_name: str) -> int:
        return self._name_to_ch[channel_name]

    def get_ch_name_from_ch_id(self, channel_id: int) -> str:
        return self.ads_channels[channel_id].channel_name

    def get_channel(self, channel: Union[str, int]) -> AdsChannel:
        if isinstance(channel, str):
            ret = self.ads_channels[self.get_ch_id_from_ch_name(channel)]
        elif isinstance(channel, int):
            ret = self.ads_channels[channel]
        else:
            raise ValueError("Channel identifier must be str or int")
        return ret

    def get_channel_level(self, ch_id: int, scaled=False):
        """Helper method to get the level on the specified channel.
        This simply calls ``read_all_channels`` and parses the return.
        By default the raw measured voltages will be returned,
        if scaled == True the physical values inferred from
        the voltage measurement will be returned.
        """
        if ch_id not in self._name_to_ch.values():
            raise ValueError(f"Invalid channel ID {ch_id}.")

        ret = self.read_channel(ch_id, scaled)
        return ret[ch_id]["value"]

    def read_channel(
            self, channel: int, scaled=False, dwell=CHANNEL_DWELL_TIME,
            return_dict: bool = True,
    ) -> Union[dict, float]:
        self.write(Ads7128.REG_CHANNEL_SEL, channel)
        time.sleep(dwell)  # default 1 ms
        while True:
            rd_val = self.bare_read()
            ch_id = rd_val & 0xf
            if ch_id == channel:
                break

        channel_data = self.ads_channels[ch_id]
        measured_val = self.adc_digital_to_volts(rd_val)

        if scaled:
            measured_val = self.scale_adc_voltage(measured_val, channel_data)

        if return_dict:
            return dict(value=measured_val, name=channel_data.channel_name)
        else:
            return measured_val

    def read_all_channels(
            self, scaled=False, dwell=CHANNEL_DWELL_TIME,
    ) -> dict:
        """Performs a read operation N times, where N is the number of
        channels configured for readout.

        This assumes auto-sequence mode and that the APPEND_STATUS register
        has been set to 0b01 (append channel ID to last 4 bits)

        If scaled == True, scaling factors will be applied .
        If scale factors are applied all current
        channels will return the measured current in milliamps.

        Returns:
            (dict): Keys are the channel ID, values are the readouts
        """
        results = {}
        for ch in self.ads_channels:
            results[ch.id] = self.read_channel(
                ch.id, scaled=scaled, dwell=dwell)

        return results

    def get_mon_all_channels(self) -> dict:
        """Loops through all the channels and creates a dictionary
        of channel names and values.
        """
        res = {result['name']: result['value']
               for result in self.read_all_channels(scaled=True).values()}

        res['v3p3_power_cb'] = res['v3p3_volts_cb'] * res['v3p3_currs_cb']
        res['v21p0_power_cb'] = res['v21p0_volts_cb'] * res['v21p0_currs_cb']
        res['v24p0_power_cb'] = res['v24p0_volts_cb'] * res['v24p0_currs_cb']
        res['vin_power_cb'] = res['vin_volts_cb'] * res['vin_currs_cb']
        return res

    def v24_voltage(self) -> float:
        """Reads/returns the 24V voltage using the ADS7128 ADC on the CMB
        """
        return self.read_channel(4, scaled=True, return_dict=False)

    def v21_voltage(self) -> float:
        """Reads/returns the 21v0 rail voltage using the ADS7128 ADC on the CMB
        """
        return self.read_channel(2, scaled=True, return_dict=False)

    def v3p3_voltage(self) -> float:
        """Reads/returns the 3v3 rail voltage using the ADS7128 ADC on the CMB
        """
        return self.read_channel(0, scaled=True, return_dict=False)
