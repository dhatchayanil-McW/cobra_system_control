"""
file: fpga_field_funcs.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

Provides utilities to convert values to fields that can be
written to the FPGA (and the reverse).

The getf_* functions are for getting field values for direct programming
into the FPGA memory.

The getv_* functions are the inverse of the getf_* functions. They are
intended for debug (to decipher the FPGA's state) and error checking (to
qualify the FPGA's state).

The comp_* functions are simply helpers for the other functions.

getf_* functions implement limit checking and, optionally, identity checking
prior to return. For identity checking, typically, `getf(getv(field))`
exactly equals `field` whereas `getv(getf(value))` equals `value` but with
some error due to quantization in `getf()`.

getv_* functions implement limit checking on inputs.
"""
import dataclasses as dc
import math

import cobra_system_control.exceptions as cobex


ENABLE_IDENTITY_CHECKS = True


def limit_check(val, low, high, name, *args):
    if not low <= val < high:
        msg = (f'Invalid value for field {name}: {val} ({val:#_x}). The '
               f'limits are [{low}, {high}). Extra args: {args}.')
        raise cobex.MemoryMapFieldValueError(msg)


def identity_check(data, check, epsilon, name, *args):
    if abs(check - data) > epsilon:
        msg = (f'Identity check failed for field {name}: original = {data} '
               f'and check = {check} with epsilon = {epsilon}. Extra '
               f'args: {args}.')
        raise cobex.MemoryMapFieldValueError(msg)


@dc.dataclass
class FpgaFieldFuncs:
    """A class to provide conversion functions between real-world
    values and FPGA register words.
    """
    memmap_fpga: 'MemoryMapPeriph'
    tp1_period_n_bits: int = dc.field(init=False)
    dac_ci_settle_tc_n_bits: int = dc.field(init=False)
    ito_cnt_n_bits: int = dc.field(init=False)
    clk_freq_mhz: int = 100
    epsilon: float = 2**-20

    def __post_init__(self):
        f_obj = self.memmap_fpga.lcm.fields['tp1_period']
        self.tp1_period_n_bits = f_obj.size

        f_obj = self.memmap_fpga.scan.fields['dac_ci_settle_tc']
        self.dac_ci_settle_tc_n_bits = f_obj.size

        f_obj = self.memmap_fpga.lcm.fields['ito_toggle_tc']
        self.ito_cnt_n_bits = f_obj.size

    def getf_tp1_period(self, tp1_period_us, ito_fmult):
        """Get the tp1_period field value from the given TP1 period,
        in us, up to
        655.36 us in 0.01 us steps. The returned value has
        the following meaning:
        0: 0.01 us, ..., 65535: 655.36 us. The LCM period is one HIMAX POL
        period or two TP1 periods.
        """
        tp1_pw = 49
        ito_toggle_tc = self.getf_ito_toggle_tc(tp1_period_us, ito_fmult)
        # reduce requested time by the time added by hardware
        tp1_period = (ito_toggle_tc + 1) * (ito_fmult + 1) - (tp1_pw + 4) - 1

        limit_check(tp1_period, 0, 2**self.tp1_period_n_bits,
                    'tp1_period', tp1_period_us, ito_fmult,
                    tp1_pw, ito_toggle_tc)
        if ENABLE_IDENTITY_CHECKS:
            check = self.getv_tp1_period(tp1_period, ito_fmult)
            identity_check(tp1_period_us, check, (ito_fmult + 1)
                           / self.clk_freq_mhz,
                           'tp1_period', tp1_period, ito_fmult,
                           tp1_pw, ito_toggle_tc)
        return tp1_period

    def getv_tp1_period(self, tp1_period, ito_fmult):
        """Return the TP1 period, in us, given the tp1_period field value.
        """
        tp1_pw = 49
        limit_check(tp1_period, 0, 2**self.tp1_period_n_bits,
                    'tp1_period', ito_fmult, tp1_pw)
        time_us = ((tp1_period + 1) + (tp1_pw + 4)) / self.clk_freq_mhz
        return time_us

    def getf_ito_toggle_tc(self, tp1_period_us, ito_fmult):
        """Get the ito_toggle_tc field value from the
        TP1 period, in us, and the ito_fmult field value.
        """
        ito_step_us = 0.01
        ito_toggle_tc = round(
            tp1_period_us / (ito_fmult + 1) / ito_step_us) - 1

        limit_check(ito_toggle_tc, 0, 2**self.ito_cnt_n_bits,
                    'ito_toggle_tc', tp1_period_us, ito_fmult, ito_step_us)
        return ito_toggle_tc

    def getv_ito_toggle_tc(self, ito_toggle_tc, ito_fmult):
        ito_step_us = 0.01
        limit_check(ito_toggle_tc, 0, 2**self.ito_cnt_n_bits,
                    'ito_toggle_tc', ito_fmult, ito_step_us)
        time_us = (ito_toggle_tc + 1) * ito_step_us
        return time_us

    def getf_dac_settle_tc(self, time_us):
        """Get the dac_ci_settle_tc field value for the given settling time. The
        DAC's analog output updates on the rising edge of SS_B. The settling
        time timer starts counting upon assertion of the SPI controller's done
        signal, which is at the same time as the deassertion of SS_B.

        WARNING: this calculation assumes no extra delay in starting the SPI
        frame, which can happen if the CPU is using the SPI bus and the Scan
        controller's request is queued.
        """
        # step is 0 -> 00_1111 -> 0.16us, step is 1 -> 01_1111 -> 0.32 us
        settling_time_steps = time_us / 0.16
        dac_ci_settle_tc = math.ceil(settling_time_steps - self.epsilon) - 1

        limit_check(dac_ci_settle_tc, 0, 2**self.dac_ci_settle_tc_n_bits,
                    'dac_ci_settle_tc', time_us)
        if ENABLE_IDENTITY_CHECKS:
            check = self.getv_dac_settle_tc(dac_ci_settle_tc)
            identity_check(time_us, check, 16/self.clk_freq_mhz,
                           'dac_ci_settle_tc', dac_ci_settle_tc)
        return dac_ci_settle_tc

    def getv_dac_settle_tc(self, dac_ci_settle_tc):
        """Return the settling time, in us, given
        the DAC's dac_ci_settle_tc field
        value.
        """
        limit_check(dac_ci_settle_tc, 0, 2**self.dac_ci_settle_tc_n_bits,
                    'dac_ci_settle_tc')
        return (dac_ci_settle_tc + 1) * 0.16

    def getf_pol_cnt_tc(self, pol_cnt):
        """FPGA requires pol_cnt-1 to be written
        to the register
        """
        return int(pol_cnt - 1)

    def getv_pol_cnt_tc(self, pol_cnt_tc):
        """FPGA register is one less than
        the real-world value
        """
        return int(pol_cnt_tc + 1)
