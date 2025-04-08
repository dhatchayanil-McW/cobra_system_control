import pytest as pt
import random
from unittest.mock import MagicMock

from cobra_system_control.metasurface import (
    LcmController, LM10_LCM_ORDERS, LM10OrderOv)


def test_order_ov():
    assert LM10OrderOv.OPTIONS == LM10_LCM_ORDERS
    assert LM10OrderOv(LM10OrderOv.OPTIONS[0]).field == 0
    assert LM10OrderOv(LM10OrderOv.OPTIONS[0]).offset == 0
    assert LM10OrderOv(LM10OrderOv.OPTIONS[2]).field == 2
    assert LM10OrderOv(LM10OrderOv.OPTIONS[2]).offset == 0x800

    # only requirement on actual orders in the table is that the last order is 0
    assert LM10OrderOv(LM10OrderOv.OPTIONS[-1]).value == 0
    assert LM10OrderOv(LM10OrderOv.OPTIONS[-1]).field == 463 - 1
    assert LM10OrderOv(LM10OrderOv.OPTIONS[-1]).offset == (463 - 1) * 0x400

    # Test nonzero_sorted_orders
    sorted_orders = LM10OrderOv.nonzero_sorted_orders()
    assert len(sorted_orders) == 463 - 1
    assert 0 not in sorted_orders


@pt.mark.parametrize("field", [
    pt.param(random.choice((range(len(LM10_LCM_ORDERS))))) for x in range(5)
])
def test_order_ov_from_field(field):
    assert LM10OrderOv.from_field(field).value == LM10_LCM_ORDERS[field]


def test_disable(mock:  bool, lcm_ctrl: LcmController):
    if mock:
        pt.skip('integration only')

    lcm_ctrl.disable()
    assert lcm_ctrl.read_fields('tcon_enable') == 0


def test_setup(mock: bool, lcm_ctrl: LcmController):
    golden_writes = {'reset_code': 0x00,
                     'pol_finish_ovr': 0,
                     'tp1_done_high': 0,
                     'n_steps': 170,
                     'rst_pw': 4,
                     'tx_wait': 7,
                     'pol_toggle': 0,
                     'settle_tc': 9,
                     'pattern_sync_mode': 'pulse',
                     'tcon_enable': 0,
                     'pol_invert': 1,
                     'gpio_ito_select': 0,
                     }

    if mock:
        actual_writes = {}
        lcm_ctrl.write_fields = MagicMock(
            side_effect=lambda **f: actual_writes.update(**f))
        lcm_ctrl.setup()
    else:
        lcm_ctrl.setup()
        actual_writes = {
            k: lcm_ctrl.read_fields(k, use_mnemonic=isinstance(golden_writes[k], str))
            for k in golden_writes.keys()
        }
    assert actual_writes == golden_writes
