from contextlib import nullcontext as does_not_raise

import pytest as pt

import cobra_system_control.exceptions as cobex
from cobra_system_control.numerical_utilities import ptob_raw12, btop_raw12
from cobra_system_control.functional_utilities import get_common_length
from cobra_system_control.validation_utilities import Register


PIXELS = [
    0xcba, 0xfed, 0x741,
    0x321, 0x654, 0x987,
    0xcba, 0xfed, 0x741,
    0x321, 0x654, 0x987,
]

BYTES = [
    0xcb, 0xfe, 0xda,
    0x74, 0x32, 0x11,
    0x65, 0x98, 0x74,
    0xcb, 0xfe, 0xda,
    0x74, 0x32, 0x11,
    0x65, 0x98, 0x74,
]


def test_ptob_raw12():
    ret = ptob_raw12(PIXELS)
    assert ret == BYTES
    ret = btop_raw12(ret)
    assert ret == PIXELS


def test_btop_raw12():
    ret = btop_raw12(BYTES)
    assert ret == PIXELS
    ret = ptob_raw12(ret)
    assert ret == BYTES


@pt.mark.parametrize('kwargs, expected, error', [
    pt.param(dict(a=[1, 2, 3], b=[4, 5, 6]), 3, None,
             id='2 equal length sequences'),
    pt.param(dict(a=1, b=[4, 5, 6]), 3, None,
             id='1 sequence, one integer'),
    pt.param(dict(a=[1], b=[4, 5, 6]), 3, None,
             id='1 length-3 sequence, 1 length-1 sequence'),
    pt.param(dict(a=[1, 2], b=[4, 5, 6]), 3, pt.raises(
        cobex.ScanPatternSizeError),
             id='2 non-unity, unequal lengths'),
    pt.param(dict(a=[1], b=[4, 5, 6], c=3), 3, None,
             id='3 inputs, each w/ different length / type'),
    pt.param(dict(a=1), 1, None, id='1 integer'),
    pt.param(dict(a=1, b=2), 1, None, id='2 integers'),
])
def test_get_common_length(kwargs: dict, expected, error):
    with error or does_not_raise():
        assert get_common_length(**kwargs) == expected


def test_register():
    class A:
        x = Register(offset=0, position=1, size=12)

    a = A()
    assert A.x.offset == 0
    assert A.x.position == 1
    assert A.x.size == 12

    with pt.raises(ValueError):
        a.x = -1
    with pt.raises(ValueError):
        a.x = 2 ** 12
    a.x = 1
    assert a.x == 1
