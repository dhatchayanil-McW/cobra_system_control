import math
from contextlib import nullcontext as does_not_raise
import unittest

import pytest as pt
from parameterized import parameterized

from cobra_system_control.values_utilities import BoundedTuple, BoundedValue, OptionValue, Value, clamp


def make_bounded_value_class(limits, tolerance):

    class A(BoundedValue):
        LIMITS = limits
        TOLERANCE = tolerance

    return A


def make_option_value_class(options):

    class A(OptionValue):
        OPTIONS = options

    return A


def make_bounded_tuple_class(limits, tolerance):

    class A(BoundedTuple):
        LIMITS = limits
        TOLERANCE = tolerance
        LENGTH = 2

    return A


@pt.fixture(scope="module")
def v_cls():

    class V(Value):

        def __eq__(self, other):
            if issubclass(other.__class__, Value):
                other = other.value
            return self.value == other

    return V


@pt.mark.parametrize("value, other, raises", [
    pt.param(2, 4, does_not_raise(), id="nominal"),
    pt.param(5, "s", pt.raises(TypeError), id="other_str"),
    pt.param("s", 5, pt.raises(TypeError), id="value_str"),
    pt.param(None, 6, pt.raises(TypeError), id="value_None"),
    pt.param("s", "a", pt.raises(TypeError), id="both_str"),
])
class TestValueArithmetic:
    """Tests the basic arithmetic dunder methods implemented in classes
    derived from ``Value``.
    """

    def test_add(self, v_cls, value, other, raises):
        if isinstance(value, str) and isinstance(other, str):
            # Strings support __add__ for concatenation
            raises = does_not_raise()

        with raises:
            assert v_cls(value) + other == value + other
        with raises:
            assert v_cls(value) + v_cls(other) == value + other
        with raises:
            assert value + v_cls(other) == value + other

    def test_sub(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) - other == value - other
        with raises:
            assert v_cls(value) - v_cls(other) == value - other
        with raises:
            assert value - v_cls(other) == value - other

    def test_mul(self, v_cls, value, other, raises):
        if ((isinstance(value, str) and isinstance(other, int)) or
            (isinstance(value, int) and isinstance(other, str))):
            # Strings support __mul__ with integers for duplication
            raises = does_not_raise()

        with raises:
            assert v_cls(value) * other == value * other
        with raises:
            assert v_cls(value) * v_cls(other) == value * other
        with raises:
            assert value * v_cls(other) == value * other

    def test_pow(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value)**other == value**other
        with raises:
            assert v_cls(value)**v_cls(other) == value**other
        with raises:
            assert value**v_cls(other) == value**other

    def test_truediv(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) / other == value / other
        with raises:
            assert v_cls(value) / v_cls(other) == value / other
        with raises:
            assert value / v_cls(other) == value / other

    def test_floordiv(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) // other == value // other
        with raises:
            assert v_cls(value) // v_cls(other) == value // other
        with raises:
            assert value // v_cls(other) == value // other

    def test_mod(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) % other == value % other
        with raises:
            assert v_cls(value) % v_cls(other) == value % other
        with raises:
            assert value % v_cls(other) == value % other

    def test_divmod(self, v_cls, value, other, raises):
        with raises:
            assert divmod(v_cls(value), other) == divmod(value, other)
        with raises:
            assert divmod(v_cls(value), v_cls(other)) == divmod(value, other)
        with raises:
            assert divmod(value, v_cls(other)) == divmod(value, other)

    def test_iadd(self, v_cls, value, other, raises):
        if isinstance(value, str) and isinstance(other, str):
            # Strings support __add__ for concatenation
            raises = does_not_raise()

        with raises:
            val = v_cls(value)
            val += other
            assert val == value + other

    def test_isub(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val -= other
            assert val == value - other

    def test_imul(self, v_cls, value, other, raises):
        if ((isinstance(value, str) and isinstance(other, int)) or
            (isinstance(value, int) and isinstance(other, str))):
            # Strings support __mul__ with integers for duplication
            raises = does_not_raise()

        with raises:
            val = v_cls(value)
            val *= other
            assert val == value * other

    def test_itruediv(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val /= other
            assert val == value / other

    def test_ifloordiv(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val //= other
            assert val == value // other

    def test_imod(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val %= other
            assert val == value % other

    def test_ipow(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val **= other
            assert val == value**other


@pt.mark.parametrize("value, other, raises", [
    pt.param(2, 4, does_not_raise(), id="nominal"),
    pt.param(5, "s", pt.raises(TypeError), id="other_str"),
    pt.param("s", 5, pt.raises(TypeError), id="value_str"),
    pt.param(None, 6, pt.raises(TypeError), id="value_None"),
    pt.param(5, 5.5, pt.raises(TypeError), id="other_float"),
    pt.param(5.5, 5, pt.raises(TypeError), id="value_float"),
    pt.param("s", "a", pt.raises(TypeError), id="both_str"),
])
class TestValueBitwise:
    """Tests the bitwise dunder methods implemented in classes
    derived from ``Value``.
    """

    def test_lshift(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) << other == value << other
        with raises:
            assert v_cls(value) << v_cls(other) == value << other
        with raises:
            assert value << v_cls(other) == value << other

    def test_rshift(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) >> other == value >> other
        with raises:
            assert v_cls(value) >> v_cls(other) == value >> other
        with raises:
            assert value >> v_cls(other) == value >> other

    def test_and(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) & other == value & other
        with raises:
            assert v_cls(value) & v_cls(other) == value & other
        with raises:
            assert value & v_cls(other) == value & other

    def test_xor(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) ^ other == value ^ other
        with raises:
            assert v_cls(value) ^ v_cls(other) == value ^ other
        with raises:
            assert value ^ v_cls(other) == value ^ other

    def test_or(self, v_cls, value, other, raises):
        with raises:
            assert v_cls(value) | other == value | other
        with raises:
            assert v_cls(value) | v_cls(other) == value | other
        with raises:
            assert value | v_cls(other) == value | other

    def test_ilshift(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val <<= other
            assert val == value << other

    def test_irshift(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val >>= other
            assert val == value >> other

    def test_iand(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val &= other
            assert val == value & other

    def test_ixor(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val ^= other
            assert val == value ^ other

    def test_ior(self, v_cls, value, other, raises):
        with raises:
            val = v_cls(value)
            val |= other
            assert val == value | other


@pt.mark.parametrize("value, raises", [
    pt.param(5, does_not_raise(), id="value_int"),
    pt.param(5.68769576895768, does_not_raise(), id="value_float"),
    pt.param("s", pt.raises(TypeError), id="value_str"),
    pt.param(None, pt.raises(TypeError), id="value_None"),
])
class TestValueMath:
    """Tests the unary and ``math`` module dunder methods in classes
    derived from ``Value``.
    """

    def test_neg(self, v_cls, value, raises):
        with raises:
            assert -v_cls(value) == -value

    def test_pos(self, v_cls, value, raises):
        with raises:
            assert +v_cls(value) == +value

    def test_abs(self, v_cls, value, raises):
        with raises:
            assert abs(v_cls(value)) == abs(value)

    def test_invert(self, v_cls, value, raises):
        if isinstance(value, float):
            # ~ is the only unary operand that doesn't allow float
            raises = pt.raises(TypeError)

        with raises:
            assert ~(v_cls(value)) == ~value

    def test_round(self, v_cls, value, raises):
        for n_dig in range(9):
            with raises:
                assert round(v_cls(value), n_dig) == round(value, n_dig)

    def test_trunc(self, v_cls, value, raises):
        with raises:
            assert math.trunc(v_cls(value)) == math.trunc(value)

    def test_floor(self, v_cls, value, raises):
        with raises:
            assert math.floor(v_cls(value)) == math.floor(value)

    def test_ceil(self, v_cls, value, raises):
        with raises:
            assert math.ceil(v_cls(value)) == math.ceil(value)


class TestBoundedValue:

    @pt.mark.parametrize("limits, tolerance, expected, raises", [
        pt.param((0, 1), 0.05, 2, does_not_raise(), id="valid"),
        pt.param((0, 1, 2), 0.05, None, pt.raises(ValueError),
                     id="bad_limits_len"),
        pt.param((0, "s"), 0.05, None, pt.raises(TypeError),
                     id="bad_limits_type"),
        pt.param((1, 0), 0.05, None, pt.raises(ValueError),
                     id="bad_limits_order"),
        pt.param((0, 1), None, None, pt.raises(AttributeError),
                     id="bad_tolerance"),
    ])
    def test_inputs(self, limits, tolerance, expected, raises):
        with raises:
            cls = make_bounded_value_class(limits, tolerance)
            assert len(cls.LIMITS) == expected
            assert all(isinstance(lim, (int, float)) for lim in cls.LIMITS)
            assert isinstance(cls.TOLERANCE, (int, float))
            assert limits[1] > limits[0]

    @pt.mark.parametrize("value, raises", [
        pt.param(0, does_not_raise(), id="lower_lim"),
        pt.param(10, does_not_raise(), id="middle"),
        pt.param(5, does_not_raise(), id="upper_lim"),
        pt.param(-1, pt.raises(ValueError), id="outside_range"),
    ])
    def test_instance_values(self, value, raises):
        with raises:
            cls = make_bounded_value_class(limits=(0, 10), tolerance=0)
            assert cls(value).value == value

    @pt.mark.parametrize("val_1, val_2, expected", [
        pt.param(0, 1, False, id="not_equal"),
        pt.param(1, 1, True, id="equal"),
        pt.param(0.5, 0.5, True, id="float_equal"),
        pt.param(0, 0.5, False, id="float_unequal"),
        pt.param(0.5, 0, False, id="float_unequal"),
    ])
    def test_equality(self, val_1, val_2, expected):
        cls_1 = make_bounded_value_class(limits=(0, 10), tolerance=0.5)
        cls_2 = make_bounded_value_class(limits=(0, 10), tolerance=0.5)

        # Different ways of testing floating-point representations
        assert (cls_1(val_1) == cls_2(val_2)) == expected
        assert (cls_1(val_1) != cls_2(val_2)) != expected
        assert (cls_1(val_1) == val_2) == expected
        assert (cls_1(val_1) != val_2) != expected

    @pt.mark.parametrize("value1, value2, expected", [
        (0, 1, True),
        (1, 0, True),
        (0, 1 + 1e-9, False),
        (1 - 1e-9, 2, False),
        (0, 2, False),
    ])
    def test_equality_tolerance(self, value1, value2, expected):
        cls = make_bounded_value_class(limits=(0, 10), tolerance=1)
        # Only way to invoke the "tolerance" parameter - same class
        assert (cls(value1) == cls(value2)) == expected
        assert (cls(value1) != cls(value2)) != expected


class TestOptionValue:

    @pt.mark.parametrize("options, value, raises", [
        pt.param([1, 2], 1, does_not_raise(), id="list_opt_int_val"),
        pt.param({'s', 'q'}, 'q', does_not_raise(), id="set_opt_str_val"),
        pt.param(1, None, pt.raises(TypeError), id="bad_opts_non_iter"),
        pt.param({1, 2}, 0, pt.raises(ValueError),
                     id="invalid_opt_int"),
        pt.param(
            ('s', 'q'), 'a', pt.raises(ValueError), id="invalid_opt_str"),
    ])
    def test_option_value(self, options, value, raises):
        with raises:
            cls = make_option_value_class(options)
            assert cls.OPTIONS == options
            assert cls(value).value == value
            assert cls(value) == value


class TestBoundedTuple:

    @pt.mark.parametrize("limits, tolerance, expected, raises", [
        pt.param((0, 1), 0.05, 2, does_not_raise(), id="good_values"),
        pt.param((0, 1, 2), 0.05, None, pt.raises(ValueError),
                     id="limits_bad_len"),
        pt.param((0, 's'), 0.05, None, pt.raises(TypeError),
                     id="limits_bad_type"),
        pt.param(None, 0.05, None, pt.raises(AttributeError),
                     id="limits_missing"),
        pt.param((1, 0), 0.05, None, pt.raises(ValueError),
                     id="limits_bad_order"),
    ])
    def test_inputs(self, limits, tolerance, expected, raises):
        with raises:
            cls = make_bounded_tuple_class(limits, tolerance)
            assert len(cls.LIMITS) == expected
            assert limits[1] > limits[0]

    @pt.mark.parametrize("value, raises", [
        pt.param((0, 1), does_not_raise(), id="valid_low_values"),
        pt.param((4, 10), does_not_raise(), id="valid_high_values"),
        pt.param((0, 11), pt.raises(ValueError), id="value_2_invalid"),
        pt.param((-1, 9), pt.raises(ValueError), id="value_1_invalid"),
        pt.param(
            (-1, -2), pt.raises(ValueError), id="both_values_invalid"),
    ])
    def test_instance_values(self, value: tuple, raises):
        with raises:
            cls = make_bounded_tuple_class(limits=(0, 10), tolerance=0)
            assert cls(*value) == value

    @pt.mark.parametrize("val_1, val_2, expected", [
        pt.param((0, 1), (2, 3), False, id="out_tolerance"),
        pt.param((1, 2), (1, 2), True, id="same_values"),
        pt.param((0.2, 0.3), (0.4, 0.5), False, id="in_tolerance"),
    ])
    def test_eq_diff_subclass(self, val_1: tuple, val_2: tuple, expected):
        cls_1 = make_bounded_tuple_class(limits=(0, 10), tolerance=0.5)
        cls_2 = make_bounded_tuple_class(limits=(0, 10), tolerance=0.5)
        assert (cls_1(*val_1) == cls_2(*val_2)) == expected
        assert (cls_1(*val_1) != cls_2(*val_2)) != expected

    @pt.mark.parametrize("val_1, val_2, expected", [
        pt.param((1, 2), (1, 3), False, id="mismatch_value"),
        pt.param((1, 2), (1, 2), True, id="same_values"),
        pt.param((0.5, 8.8), (0.3, 7.7), False, id="float_out_tolerance"),
    ])
    def test_eq_same_subclass(self, val_1: tuple, val_2: tuple, expected):
        cls = make_bounded_tuple_class(limits=(0, 10), tolerance=0.5)
        assert (cls(*val_1) == cls(*val_2)) == expected
        assert (cls(*val_1) != cls(*val_2)) != expected

    @pt.mark.skip("We don't support BoundedTuple tolerance (yet)")
    @pt.mark.parametrize("val_1, val_2, expected", [
        pt.param((0, 0.1), (1, 1.1), True),
    ])
    def test_equality_tolerance(self, val_1: tuple, val_2: tuple, expected):
        cls = make_bounded_value_class(limits=(0, 10), tolerance=1)
        assert (cls(*val_1) == cls(*val_2)) == expected
        assert (cls(*val_1) != cls(*val_2)) != expected

    def test_iteration(self):
        cls = make_bounded_tuple_class(limits=(0, 10), tolerance=0.5)
        a = cls(1, 2)
        for i, value in enumerate(a):
            assert a[i] == value


class TestClamp(unittest.TestCase):
    """Tests the clamp function
    """
    @parameterized.expand([
        (5, 0, 10, 5),  # No clamping
        (-20, 0, 10, 0),  # Lower clamp
        (20, 0, 10, 10),  # Upper clamp
        (20, 0, None, 20),  # No upper clamp
        (-20, None, 10, -20),  # No lower clamp
        (0, None, None, 0),  # No clamp
        (None, None, None, None, ValueError),  # val = None
        (0, 2, 1, None, ValueError),  # low > high
        (0, 2, 2, None, ValueError),  # low == high
        (5.5, 1, 2, None, TypeError),  # val is float
        (5, 1.2, 2, None, TypeError),  # low is float
        (5, 1, 2.2, None, TypeError),  # high is float
    ])
    def test_clamp(self, val, low, high, expected, error=None):
        if error:
            with self.assertRaises(error):
                clamp(val, low, high)
        else:
            c_val = clamp(val, low, high)
            self.assertIsInstance(c_val, type(val))
            self.assertEqual(c_val, expected)
