from contextlib import nullcontext as does_not_raise
from typing import Callable

import unittest

from parameterized import parameterized
import pytest as pt

from cobra_system_control.validation_utilities import (
    is_in_bounds, BoundedNumber, Options, cast_to_sequence, cast,
)


class TestIsInBounds(unittest.TestCase):

    @parameterized.expand([
        (1, 0, 2, True, True),
        (0, 0, 2, True, True),
        (0, 0, 2, False, False),
        (2, 0, 2, False, False),
        (2, 0, 2, True, True),
        (3, 0, 2, True, False),
        (-1, 0, 2, True, False),
    ])
    def test_is_in_bounds(self, value, lower, upper, inclusive, expected):
        assert is_in_bounds(value, lower, upper, inclusive) == expected


def return_descriptor_class(limits):
    class A:
        bounded = BoundedNumber(*limits)

        def __init__(self, v):
            self.bounded = v

    return A


class TestBoundedNumber(unittest.TestCase):

    @parameterized.expand([
        (1, ('no', 'good'), TypeError),  #short cicuits check on upper bound!!
        (1, (0, 'nope'), TypeError),  # checks if upper bound is not valid
        ('hi', (0, 1), TypeError),
        (.5, (0, 1), None),
    ])
    def test_type(self, value, bounds, expected):
        if expected == TypeError:
            self.assertRaises(TypeError, lambda: return_descriptor_class(bounds)(value))
        else:
            return_descriptor_class(bounds)(value)

    @parameterized.expand([
        (1, (0, 2), None),
        (0, (0, 2), None),
        (2, (0, 2), None),
        (2.5, (0, 2), ValueError),
        (-10, (0, 2), ValueError)
    ])
    def test_bounds(self, value, bounds, expected):
        if expected == ValueError:
            self.assertRaises(ValueError, lambda: return_descriptor_class(bounds)(value))
        else:
            a = return_descriptor_class(bounds)(value)
            self.assertEqual(value, a.bounded)


def return_options_class(opts):
    class A:

        value = Options(*opts)

        def __init__(self, v):
            self.value = v

    return A


class TestOptions(unittest.TestCase):

    @parameterized.expand([
        (1,(),ValueError),
        (1, (1, 2, 3), None),
        (.5, 'None', ValueError),
        (1, (None, 1), None),
    ])
    def test_options_iterable(self, value, options, expected):
        if ValueError == expected:
            self.assertRaises(expected, lambda: return_options_class(options)(value))
        else:
            return_options_class(options)(value)

    @parameterized.expand([
        (1, (1, 2, 3), None),
        (.5, (1, 2, 3), ValueError),
    ])
    def test_in_options(self, value, options, expected):
        if ValueError == expected:
            self.assertRaises(expected, lambda: return_options_class(options)(value))
        else:
            a = return_options_class(options)(value)
            self.assertEqual(a.value, value)


class A:

    def __init__(self, a: float):
        self.a = a

    def __eq__(self, other):
        return self.a == other.a


class TestCast(unittest.TestCase):

    @parameterized.expand([
        (0.5, float, 0.5),
        (2, float, 2.0),
        (0.5, A, A(.5))
    ])
    def test_cast(self, inst, cls, expected):
        self.assertEqual(cast(inst, cls), expected)


@pt.mark.parametrize('val, length, func, expected, error', [
    pt.param(1, 5, lambda x: x, [1] * 5, None, id='int to sequence'),
    pt.param(1, 5, lambda x: 2 * x, [2] * 5, None, id='use func'),
    pt.param([1], 5, lambda x: x, [1] * 5, None, id='1-length seq ok'),
    pt.param([1, 2], 5, lambda x: x, [2] * 5, pt.raises(ValueError),
             id='incompatible lengths'),
    pt.param([1, 2, 3, 4, 5], 5, lambda x: 2 * x, [2, 4, 6, 8, 10], None,
             id='compatible lengths'),
])
def test_cast_to_sequence(val, length, func: Callable, expected, error):
    with error or does_not_raise():
        assert cast_to_sequence(val, length, func) == expected
