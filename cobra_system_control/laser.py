"""
file: laser.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file provide drivers for the DACs that control laser
CI voltage or laser VLDA voltage present on the
Compute Platform (NCB) and the M30 Sensor Head.

Helper functions are provided to help set the maximum
safe operating CI based on the system type and the
PCB revision.
"""
from typing import Union

import numpy as np
import Pyro5.api

from cobra_system_control.dacs import DacDevice, VDac
from cobra_system_control.values_utilities import OptionValue


LASER_PERCENT_OPTIONS = list(range(1, 100+1))


def ci_max_by_system(whoami: str, pcb_rev: int):
    """Returns the maximum safe operating CI voltage
    level for the CI rail by system type and PCB revision.
    These values were determined experimentally.
    """
    if whoami == "m30":
        if pcb_rev == 1:
            ci_max = 1.8
        elif pcb_rev == 2:
            ci_max = 2.075
        else:
            ci_max = 1.8
    else:
        ci_max = 1.8
    return ci_max


def ci_percentage_array(
        ci_max: float,
        slope: float = 850, offset: float = -1013):
    """Converts the CI max voltage and fit from experimental
    data to array for the LaserPowerPercentMappedOv

    Relationship for M30 is approximately: power [mW] = CI * 850 - 1013
    which was determined experimentally.
    """
    max_power = ci_max * slope + offset
    relative_power_array = np.asarray(LASER_PERCENT_OPTIONS) / 100
    ci_array = (max_power * relative_power_array - offset) / slope
    return ci_array


@Pyro5.api.expose
class LaserPowerPercentMappedOv(OptionValue):
    """Sets the valid index options for CI
    to provide a relative laser power percent

    Relationship is approximately: power [mW] = CI * 850 - 1013
    which was determined experimentally.
    """
    MAP = None
    OPTIONS = None

    def __reduce__(self):
        state = self.__dict__.copy()
        return (self.__class__, (), state)

    @property
    def mapped(self):
        """Returns the mapped CI value given a power percent
        input value.
        """
        # pylint: disable-next=unsubscriptable-object
        return LaserPowerPercentMappedOv.MAP[
            LaserPowerPercentMappedOv.OPTIONS.index(self.value)]


@Pyro5.api.behavior(instance_mode="single")
class LaserPowerPercentMappedOvFactory:
    def __call__(self, whoami: str, pcb_rev: int):
        """Converts the system type and pcb revision to a
        class that controls the CI voltage level and maps
        them to a percentage usable by the API.
        """
        ci_max = ci_max_by_system(whoami, pcb_rev)
        ci_array = ci_percentage_array(ci_max)

        LaserPowerPercentMappedOv.MAP = list(ci_array)
        LaserPowerPercentMappedOv.OPTIONS = LASER_PERCENT_OPTIONS

        return LaserPowerPercentMappedOv


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class LaserCiDac(VDac):
    """A class to control the DAC to set the Laser CI.

    - The DAC runs off the 3.3 rail, which limits CI to 3.3V.
      this corresponds to a digital value of about 700
    """
    def __init__(self, dac_device: DacDevice, chan_idx: int,
                 enable_ctrl):
        super().__init__(dac_device, chan_idx, enable_ctrl)
        self.ci_limit = 1.5   # set low until system configured
        self.slope = 1
        self.offset = 0

    def setup(self):
        super().setup()
        self.set_voltage(0)

    def set_ci_limit(self, whoami: str, pcb_rev: int):
        """Sets the ci_limit for the CI DAC. This needs to
        be called after the PCB Revision number is known.
        """
        self.ci_limit = ci_max_by_system(whoami, pcb_rev)

    def set_voltage(self, vset: float):
        """Calculates the payload for a CI-V setting
        and writes it to the DAC60508"""
        if vset > self.ci_limit:
            raise ValueError(
                f'Desired CI voltage {vset} larger than limit of {self.ci_limit}')
        super().set_voltage(vset)

    def disable(self):
        """This is a direct write in case the slope and offset
        of the dac have not been initialized
        """
        self.raw_set_zero_voltage()

    def disconnect(self):
        self.disable()


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class LaserVldaDac(VDac):
    """Class to control the Laser VLDA DAC on
    either the Sensor Head or the NCB.
    On the NCB, this controls the output of the 24p0 regulator.
    On the Sensor Head, this sets the output of the VLDA regulator.

    Based on the chosen resistors, the calculated range is

    For the NCB/"nxp":
    3.3V DAC ~= 10.88V Regulator output
    0V DAC ~= 24.14V Regulator output

    For M30 and M25, the expected output is dependent
    on the input from the NCB, due to this, we set the NCB
    voltage to max and adjust with the Sensor Head DAC down to
    the desired value.

    The slope and offset values are initialized with approximate
    values and then updated during a calibration
    based on feedback from the ADC.
    """
    def __init__(self, dac_device: DacDevice, channel_idx: int, dac_loc: str,
                 enable_ctrl: Union['FpgaDbg', 'CpuGpio']):
        super().__init__(dac_device, channel_idx, enable_ctrl)
        self.dac_loc = dac_loc

        if self.dac_loc == "nxp":
            # 3.3 = m (10.88) + b
            # 0 = m (24.14) + b
            self.slope = -0.2489
            self.offset = 6.008
        elif self.dac_loc == "m30":
            # 3.3 = m (12) + b
            # 0 = m (23) + b
            self.slope = -0.26277
            self.offset = 6.26509

        else:
            raise ValueError(f'slope and offset not defined for {dac_loc}')

    def connect(self):
        self.dac.connect()

    def setup(self):
        super().setup()
        # Set min VLDA voltage
        self.raw_set_max_dac_voltage()

    def enable(self):
        if self.dac_loc == "m30":
            self.enable_ctrl.write_fields(vlda_en=1)
        elif self.dac_loc == "nxp":
            self.enable_ctrl.enable()

    def disable(self):
        if self.dac_loc == "m30":
            self.enable_ctrl.write_fields(vlda_en=0)
        elif self.dac_loc == "nxp":
            self.enable_ctrl.disable()
