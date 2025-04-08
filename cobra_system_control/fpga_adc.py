"""
file: fpga_adc.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file provides a driver for interfacing with the ADC
internal to the FPGA. A method is provided to retrieve the
value for each available monitor, with value conversions
based on the PCB schematic.
"""
import time
from typing import Union, Sequence

import numpy as np

from cobra_system_control.device import Device
from cobra_system_control.cobra_log import log
from cobra_system_control import remote


MON_VOLTAGE_CHAN = (
    'v18p0', 'vlda', 'v1p2',
    'vcc', 'vccaux', 'vccio0',
    'vref_hi', 'vref_lo',
    'v24p0', 'v9p0', 'v2p8', 'vmgh',
    'v21p0', 'vito',
)

MON_CURR_CHAN = (
    'vlda_current',
    'lcm_current_coarse', 'lcm_current_fine',
    'ito_current',
    'fpga_current',
)

MON_TEMP_CHAN = (
    'laser_temp', 'lcm_temp',
    'pcb_temp', 'die_temp',
)

MON_MISC_CHAN = ('amb_det_1', 'amb_det_2')

MON_CALC_POWER_CHAN = (
    'lcm_power', 'vlda_power', 'ito_power', 'fpga_power'
)

MON_POSSIBLE_CHAN = (MON_VOLTAGE_CHAN + MON_CURR_CHAN
                     + MON_TEMP_CHAN + MON_MISC_CHAN
                     + MON_CALC_POWER_CHAN)


NTCG063JF103FTB_STEINHART_COEFFS = [
    7.74757206e-04, 2.88511686e-04, -4.01680505e-06, 3.36325480e-07]


def enum_external_monitors_m30():
    """Enumerates all the signals that enter the FPGA ADC
    from the analog muxes on the MON_V pin in M30.
    The list is of monitor register names in order of
    measurement sequence by the FPGA.

    +=====================+=============+
    | SIGNAL (M30)        |  AMUX SEL   |
    +=====================+=============+
    | csense_ito          |  0 = 00_0xx |
    | lcm_temp5           |  4 = 00_1xx |
    | ito                 |  8 = 01_0xx |
    | laser_therm         | 12 = 01_1xx |
    | amb_det_1           | 16 = 10_000 |
    | lcm_currs_coarse    | 17 = 10_001 |
    | 21v_raw             | 18 = 10_010 |
    | vref_hi             | 19 = 10_011 |
    | vref_lo             | 20 = 10_100 |
    | amb_det_2           | 21 = 10_101 |
    | vlda_currs          | 22 = 10_110 |
    | pcb_temp            | 23 = 10_111 |
    | fpga_currs          | 24 = 11_000 |
    | vlda                | 25 = 11_001 |
    | 24v                 | 26 = 11_010 |
    | 18v_lcm             | 27 = 11_011 |
    | 9v_lcm              | 28 = 11_100 |
    | 2p8v                | 29 = 11_101 |
    | vmgh                | 30 = 11_110 |
    | 1p2v                | 31 = 11_111 |
    +---------------------+-------------+
    """
    for name in (
            'mon_csense_ito',
            'mon_lcm_temp5',
            'mon_ito',
            'mon_laser_therm',
            'mon_amb_det_1',
            'mon_lcm_currs_coarse',
            'mon_21v_raw',
            'mon_vref_hi',
            'mon_vref_lo',
            'mon_amb_det_2',
            'mon_vlda_currs',
            'mon_pcb_temp',
            'mon_fpga_currs',
            'mon_vlda',
            'mon_24v',
            'mon_18v_lcm',
            'mon_9v_lcm',
            'mon_2p8v',
            'mon_vmgh',
            'mon_1p2v',
    ):
        yield name


def enum_internal_monitors_m30():
    """Enumerates all the internally measured FPGA values in M30.
    The list is of monitor register names.
    """
    for name in (
            'mon_lcm_currs_fine',
            'mon_vcc',
            'mon_vccaux',
            'mon_vccio0',
            'mon_dtr',
    ):
        yield name


def get_mon_all_voltages(cls: Union['FpgaAdc']) -> dict:
    d = {}
    d['v18p0'] = cls.get_mon_18v_lcm()
    d['vlda'] = cls.get_mon_vlda()
    d['v1p2'] = cls.get_mon_1p2v()
    d['vcc'] = cls.get_mon_vcc()
    d['vccaux'] = cls.get_mon_vccaux()
    d['vccio0'] = cls.get_mon_vccio0()
    d['vref_hi'] = cls.get_mon_vref_hi()
    d['vref_lo'] = cls.get_mon_vref_lo()
    d['v24p0'] = cls.get_mon_24v()
    d['v9p0'] = cls.get_mon_9v_lcm()
    d['v2p8'] = cls.get_mon_2p8v()
    d['vmgh'] = cls.get_mon_vmgh()
    d['vito'] = cls.get_mon_vsense_ito()
    d['v21p0'] = cls.get_mon_21v_raw()
    for k in MON_VOLTAGE_CHAN:
        if k not in d.keys():
            raise KeyError(f'ADC voltage dictionary missing key {k}')
    return d


def get_mon_all_currents(cls) -> dict:
    d = {}
    d['vlda_current'] = cls.get_mon_vlda_currs()
    d['lcm_current_fine'] = cls.get_mon_lcm_currs_fine()
    d['lcm_current_coarse'] = cls.get_mon_lcm_currs_coarse()
    d['ito_current'] = cls.get_mon_csense_ito()
    d['fpga_current'] = cls.get_mon_fpga_currs()
    for k in MON_CURR_CHAN:
        if k not in d.keys():
            raise KeyError(f'ADC current dictionary missing key {k}')
    return d


def get_mon_all_temps(cls) -> dict:
    d = {}
    d['laser_temp'] = cls.get_mon_laser_temp()
    d['lcm_temp'] = cls.get_mon_lcm_temp()
    d['pcb_temp'] = cls.get_mon_pcb_temp()
    d['die_temp'] = cls.get_mon_fpga_temp()
    for k in MON_TEMP_CHAN:
        if k not in d.keys():
            raise KeyError(f'ADC temperature dictionary missing key {k}')
    return d


def get_mon_all_misc(cls) -> dict:
    d = {}
    d['amb_det_1'] = cls.get_mon_amb_det_1()
    d['amb_det_2'] = cls.get_mon_amb_det_2()
    for k in MON_MISC_CHAN:
        if k not in d.keys():
            raise KeyError(f'ADC misc dictionary missing key {k}')
    return d


def get_mon_all(cls) -> dict:
    d = {}
    d.update(get_mon_all_voltages(cls))
    d.update(get_mon_all_currents(cls))
    d.update(get_mon_all_temps(cls))
    d.update(get_mon_all_misc(cls))

    # Add calculated fields
    d['lcm_power'] = d['v18p0'] * d['lcm_current_coarse']
    d['vlda_power'] = d['vlda'] * d['vlda_current']
    d['ito_power'] = d['vito'] * d['ito_current']
    d['fpga_power'] = 1.8 * d['fpga_current']
    for k in MON_POSSIBLE_CHAN:
        if k not in d.keys():
            raise KeyError(f'Monitor dictionary missing key {k}')
    return d


def steinhart_eq(res: float, coeffs: Sequence[float]):
    """ Calculate the laser thermistor temperature from resistance

    Coefficients can be calculated from best fit of manufacturer data.
    using cobra_system_control/resources/fit_laser_thermistor.py

    https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
    """
    temp_k = 1. / (coeffs[0] +
                   coeffs[1] * np.log(res) +
                   coeffs[2] * np.power(np.log(res), 2) +
                   coeffs[3] * np.power(np.log(res), 3))
    temp_c = temp_k - 273.15
    return temp_c


def get_laser_temp_from_adc_v(
        adc_v: float, external_vref: float) -> float:
    """Returns the laser temperature in Celsius.
    This portion of the calculation separated for use
    in row start calibration
    """
    # Reference resistance is 7.15k
    res = (7150 * adc_v) / (external_vref - adc_v)
    return steinhart_eq(res, NTCG063JF103FTB_STEINHART_COEFFS)


@remote.register_for_serialization
class FpgaAdc(Device):
    """A class to calibrate and retrieve values from the
    FPGA internal ADC.
    """
    ADC_BITS = 12
    ADC_SCALE = 2**ADC_BITS

    def __init__(self, bus: 'I2CBus', device_addr: int,
                 memmap_periph: 'MemoryMapPeriph', whoami: str):
        super().__init__(bus, device_addr, 2, 1, memmap_periph,
                         addr_bigendian=True, data_bigendian=True)
        self.memmap_periph = memmap_periph
        self.whoami = whoami

        self.internal_vref = 1.2
        self.external_vref = 1.22
        self.lcm_therm_ref_res = None
        self.lcm_therm_ref_temp = None

        self._cal_gain = None
        self._cal_offset = None

    def setup(self):
        self.write_fields(adc_sample_window=255)
        self.reset()
        self.calibrate()

    def reset(self):
        self.write_fields(enable=0)
        self.write_fields(enable=1)

    def enable(self):
        pass

    def disable(self):
        pass

    def calibrate(self):
        """Calibrates the FPGA ADC measurements
        """
        self.calibrate_gain_offset()
        self.calibrate_lcm_thermistor()

    def calibrate_gain_offset(self):
        """Calibrates the gain and offset scaling for the FPGA ADC measurements
        using a two point measurement of the VREF voltage.

        Get correction terms such that:
        v_calibrated = count * self.cal_gain + self.cal_offset

        ADC measures get refreshed every 256*0.64us*34=5.6ms
        """
        v_hi = self.external_vref * 41.2 / (41.2 + 4.99)
        v_lo = self.external_vref * 4.99 / (41.2 + 4.99)
        for i in range(5):
            time.sleep(0.1)
            count_hi_list = []
            count_lo_list = []
            for _ in range(5):
                # Take 5 measurements to average them
                count_hi_list.append(self.read_fields('mon_vref_hi'))
                count_lo_list.append(self.read_fields('mon_vref_lo'))
                # ADC measures get refreshed every 256*0.64us*34=5.6ms
                time.sleep(0.01)

            count_hi = np.mean(count_hi_list)
            count_lo = np.mean(count_lo_list)

            cal_gain = (v_hi - v_lo) / (count_hi - count_lo)
            # If count_lo is too low, then offset for correction is positive
            cal_offset = v_lo - count_lo * cal_gain

            # Check that the resulting vref_hi and vref_lo provide accurate results
            self._cal_gain = cal_gain
            self._cal_offset = cal_offset
            vref_hi = self.get_mon_vref_hi()
            vref_lo = self.get_mon_vref_lo()
            msg = ('vref_hi %.4f, vref_hi_diff %.4f, vref_lo %.4f, vref_lo_diff %.4f ',
                   vref_hi, vref_hi-self.external_vref, vref_lo, vref_lo-self.external_vref)
            log.debug(*msg)
            try:
                assert vref_hi <= (self.external_vref + 0.02)
                assert vref_hi >= (self.external_vref - 0.02)
                assert vref_lo <= (self.external_vref + 0.02)
                assert vref_lo >= (self.external_vref - 0.02)
                return
            except AssertionError:
                log.info('FPGA ADC Calibration Error')
                log.info(*msg)
                continue
        # If the assertions failed n times in row and the return statement was not called
        msg = ("FPGA ADC calibration gain and/or offset are out of range:"
               "fpga adc cal iter=%s, count_hi=%s, count_lo=%s, "
               " cal_gain=%.3f, cal_offset=%.3f",
               i, count_hi, count_lo, cal_gain, cal_offset)
        log.debug(*msg)
        log.info('Non-passing calibration vref: vref_hi=%s, vref_lo=%s',
                 self.get_mon_vref_hi(), self.get_mon_vref_lo())

    def calibrate_lcm_thermistor(self):
        """Read the lcm thermistor on system boot and compare to
        PCB temperature reading. This assumes that the LCM temperature
        is close to the PCB temperature at boot, which is a reasonable
        assumption. This will give much better LCM temperature
        data than assuming the same thermistor resistance for every
        LCM.
        """
        pcb_temp = self.get_mon_pcb_temp()
        lcm_resistance = self.voltage_to_lcm_therm_res(
            self.read_adc_and_adjust('mon_lcm_temp5'))
        # Save these values as the calibrated resistance and temperature
        self.lcm_therm_ref_res = lcm_resistance
        self.lcm_therm_ref_temp = pcb_temp
        lcm_temp = self.get_mon_lcm_temp()
        log.info('After LCM temp calibration, PCB temp = %s and LCM temp = %s',
                 pcb_temp, lcm_temp)

    def voltage_to_lcm_therm_res(self, v_lcm_therm):
        """ Calculate the LCM thermistor resistance from the measured voltage

        These equations are printed on the M30 RX schematic,
        sheet 3, D2-D3
        """
        temp_v = (v_lcm_therm + self.external_vref) / 3
        res = (4990 * temp_v) / (self.external_vref - temp_v)
        return res

    def lcm_therm_res_to_temp(self, res):
        """Calculates the LCM temperature from the measured resistance
        """
        c = 0.393  # %/C
        return ((res - self.lcm_therm_ref_res)
                / (c / 100 * self.lcm_therm_ref_res)
                + self.lcm_therm_ref_temp)

    def read_adc_and_adjust(self, field_name: str) -> float:
        count = self.read_fields(field_name)
        if (count >= 2**12) or (count < 0):
            raise ValueError(f'ADC code {count} is >= {2**12}')

        return count * self.cal_gain + self.cal_offset

    ### VOLTAGES ###
    def get_mon_18v_lcm(self) -> float:
        """Returns about 18.0 V"""
        v = self.read_adc_and_adjust('mon_18v_lcm')
        return v * 17.533

    def get_mon_vlda(self) -> float:
        """Returns VLDA voltage
        """
        v = self.read_adc_and_adjust('mon_vlda')
        return v * 25.85

    def get_mon_1p2v(self) -> float:
        """Returns 1.2V voltage
        """
        v = self.read_adc_and_adjust('mon_1p2v')
        return v * 1.205

    def get_mon_vcc(self) -> float:
        """Returns about 1.0 V

        Lattice update says that VCC is actually VCCM
        because hooked up wrong in silicon.
        So reads 1.2V
        """
        v = self.read_adc_and_adjust('mon_vcc')
        return v * 2.5

    def get_mon_vccaux(self) -> float:
        """Returns about 1.8 V"""
        v = self.read_adc_and_adjust('mon_vccaux')
        return v * 2.5

    def get_mon_vccio0(self) -> float:
        """Returns about 1.8 V

        This may be wrong in Lattice's IP and is
        really reading vccio1 and returning 3.3V
        """
        v = self.read_adc_and_adjust('mon_vccio0')
        return v * 2.5

    def get_mon_vref_hi(self) -> float:
        """Returns about 1.22 V"""
        v = self.read_adc_and_adjust('mon_vref_hi')
        return v * 1.121

    def get_mon_vref_lo(self) -> float:
        """Returns about 1.22 V"""
        v = self.read_adc_and_adjust('mon_vref_lo')
        return v * 9.2565

    def get_mon_24v(self) -> float:
        """Returns 24V voltage
        """
        v = self.read_adc_and_adjust('mon_24v')
        return v * 25.85

    def get_mon_9v_lcm(self) -> float:
        """Returns about 9.0 V"""
        v = self.read_adc_and_adjust('mon_9v_lcm')
        return v * 17.533

    def get_mon_2p8v(self) -> float:
        """Returns 2.8V voltage
        """
        v = self.read_adc_and_adjust('mon_2p8v')
        return v * 2.695

    def get_mon_vmgh(self) -> float:
        """Returns vmgh voltage
        """
        v = self.read_adc_and_adjust('mon_vmgh')
        return v * 2.0

    def get_mon_vsense_ito(self) -> float:
        v = self.read_adc_and_adjust('mon_ito')
        return v * 25.85

    def get_mon_21v_raw(self) -> float:
        v = self.read_adc_and_adjust('mon_21v_raw')
        return v * 17.533

    ### CURRENTS ###
    def get_mon_vlda_currs(self) -> float:
        """Returns VLDA current in Amps

        (With 27 mOhm resistor, 1V = 741mA)
        """
        v = self.read_adc_and_adjust('mon_vlda_currs')
        return v * 0.741

    def get_mon_lcm_currs_fine(self) -> float:
        """Returns Amps

        (With 1 ohm resistor, 20mA/V scaling)
        """
        v = self.read_adc_and_adjust('mon_lcm_currs_fine')
        return v * 20e-3

    def get_mon_lcm_currs_coarse(self) -> float:
        """Returns Amps
        # U35B op-amp scaling = Vadc = (1 + (Rf/R2)) * Vreg
        # Regulator scaling = Vreg/0.688 = 1A
        # Amps = Vadc / (1 + (Rf/R2)) / 0.688
        """
        v = self.read_adc_and_adjust('mon_lcm_currs_coarse')
        return v / (1 + (30.1 / 10)) / 0.688

    def get_mon_csense_ito(self) -> float:
        """INA190A2 (U33)
        Vout = (I_load * Rsense * GAIN) + Vref
        Vout = (I_load * 1.6 * 50) + 0
        Vout = I_load * 80
        I_load = Vout / 80
        """
        v = self.read_adc_and_adjust('mon_csense_ito')
        return v / 80

    def get_mon_fpga_currs(self) -> float:
        """Current on the 1.8V rail going to FPGA

        INA190A2 (U24)
        Vout = (I_load * Rsense * GAIN) + Vref
        Vout = (I_load * 100e-3 * 50) + 0
        Vout = I_load * 5
        I_load = Vout / 5
        """
        v = self.read_adc_and_adjust('mon_fpga_currs')
        return v / 5

    ### TEMPERATURES ###
    def get_mon_laser_temp(self) -> float:
        """Returns laser temp in Celsius.
        """
        v = self.read_adc_and_adjust('mon_laser_therm')
        return get_laser_temp_from_adc_v(
            v, self.external_vref)

    def get_mon_lcm_temp(self) -> float:
        v = self.read_adc_and_adjust('mon_lcm_temp5')
        res = self.voltage_to_lcm_therm_res(v)
        return self.lcm_therm_res_to_temp(res)

    def get_mon_pcb_temp(self) -> float:
        """Uses NTCG063JF103FTB
        Reference resistance is 7.15k
        """
        v = self.read_adc_and_adjust('mon_pcb_temp')
        res = (7150 * v) / (self.external_vref - v)
        return steinhart_eq(res, NTCG063JF103FTB_STEINHART_COEFFS)

    def get_mon_fpga_temp(self) -> float:
        """Returns FPGA die temp in Celsius.

        temp_c = 440.6 - code * vref / 7.105
               = 440.6 - code * (vref / 4096) * 576.5
               = 440.6 - v_adj * 576.5
        """
        v = self.read_adc_and_adjust('mon_dtr')
        return 440.6 - v * 576.5

    ###   MISC   ###
    def get_mon_amb_det_1(self) -> float:
        return self.read_adc_and_adjust('mon_amb_det_1')

    def get_mon_amb_det_2(self) -> float:
        return self.read_adc_and_adjust('mon_amb_det_2')

    @property
    def cal_gain(self):
        return self._cal_gain

    @property
    def cal_offset(self):
        return self._cal_offset
