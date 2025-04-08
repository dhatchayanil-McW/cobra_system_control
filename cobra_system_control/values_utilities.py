"""
file: value_utilities.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines a Value class and related subclasses
to validate input values as bounded or options.
"""
from abc import ABCMeta, abstractmethod
from typing import Iterable, Tuple, Union
from typing import TypeVar
import math

from cobra_system_control.validation_utilities import BoundedNumber, Options

Number = Union[float, int]


class Value(metaclass=ABCMeta):
    """Abstract class for input validation, value <-> field mapping, and comparison."""

    def __init__(self, value):
        self.value = value

    @abstractmethod
    def __eq__(self, other):
        """Tests for equality with other Inputs."""
        raise NotImplementedError()

    def __repr__(self):
        """How to represent the underlying data."""
        return f'{self.__class__.__name__}(value={self.value})'

    def __format__(self, format_spec):
        return format(self.value, format_spec)

    def __add__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value + other

    def __sub__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value - other

    def __mul__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value * other

    def __pow__(self, power, modulo=None):
        return pow(self.value, power, modulo)

    def __truediv__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value / other

    def __floordiv__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value // other

    def __mod__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value % other

    def __divmod__(self, other):
        return self.__floordiv__(other), self.__mod__(other)

    def __lshift__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value << other

    def __rshift__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value >> other

    def __and__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value & other

    def __xor__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value ^ other

    def __or__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return self.value | other

    def __radd__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other + self.value

    def __rsub__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other - self.value

    def __rmul__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other * self.value

    def __rtruediv__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other / self.value

    def __rfloordiv__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other // self.value

    def __rmod__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other % self.value

    def __rdivmod__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return divmod(other, self.value)

    def __rpow__(self, other, modulo=None):
        if issubclass(other.__class__, Value):
            other = other.value
        return pow(other, self.value, modulo)

    def __rlshift__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other << self.value

    def __rrshift__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other >> self.value

    def __rand__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other & self.value

    def __rxor__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other ^ self.value

    def __ror__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        return other | self.value

    def __iadd__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value += other
        return self

    def __isub__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value -= other
        return self

    def __imul__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value *= other
        return self

    def __itruediv__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value /= other
        return self

    def __ifloordiv__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value //= other
        return self

    def __imod__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value %= other
        return self

    def __ipow__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value **= other
        return self

    def __ilshift__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value <<= other
        return self

    def __irshift__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value >>= other
        return self

    def __iand__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value &= other
        return self

    def __ixor__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value ^= other
        return self

    def __ior__(self, other):
        if issubclass(other.__class__, Value):
            other = other.value
        self.value |= other
        return self

    def __neg__(self):
        return -self.value

    def __pos__(self):
        return +self.value

    def __abs__(self):
        return abs(self.value)

    def __invert__(self):
        return ~self.value

    def __round__(self, n=None):
        return round(self.value, n)

    def __trunc__(self):
        return math.trunc(self.value)

    def __floor__(self):
        return math.floor(self.value)

    def __ceil__(self):
        return math.ceil(self.value)


BvSubclass = TypeVar('BvSubclass', bound='BoundedValue')


class BoundedValue(Value):
    """Abstract base class for bounded numerical values.

    Typical use:

        .. code::python

            class CI(BoundedValue):

                LIMITS = (0, 4)
                TOLERANCE = 0.05

                @property
                def field(self) -> int:
                    return func(self.value) # convert the value to field...

                @classmethod
                def from_field(cls, field: int):
                    return cls(inverse_func(field))  # convert the field to a value

            ci = CI(4)
            print(ci == CI.from_field(ci.field))  # prints True if methods defined correctly
    """

    LIMITS: Tuple[float, float] = (None, None)
    TOLERANCE: float = None

    def __eq__(self, other):
        """Test for equality with other instances of subclass ``BoundedValue``.

        We need to have a tolerance parameter for two reasons:
        1. Numerical error from solvers, such as ``MonotonicFit``.
        2. Precision loss from conversion to and from fixed point values.

        The tolerance parameter will only be used if comparison with another instance
        of the same ``BoundedValue`` subclass is used. Otherwise, the comparison
        is done with their floating point representation.

        """
        if isinstance(other, self.__class__):
            return abs(other.value - self.value) <= self.TOLERANCE
        else:
            return float(other) == float(self)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __float__(self):
        return float(self.value)

    def __int__(self):
        return int(self.value)

    def __lt__(self, other):
        return float(self) < float(other)

    def __le__(self, other):
        return float(self) <= float(other)

    def __gt__(self, other):
        return float(self) > float(other)

    def __ge__(self, other):
        return float(self) >= float(other)

    @classmethod
    def __init_subclass__(cls, **kwargs):
        if any(attr is None for attr in (cls.LIMITS, cls.TOLERANCE)):
            raise AttributeError('subclasses of BoundedValue must define class'
                                 ' attributes "LIMITS" and "TOLERANCE"')

        try:
            if any([not isinstance(x, (int, float))
                    for x in (*cls.LIMITS, cls.TOLERANCE)]):
                raise TypeError('limits and tolerance must be integers or floats.')
        except AttributeError as exc:
            raise AttributeError('subclasses of BoundedValue must pass '
                                 '"limits" and "tolerance"') from exc

        if len(cls.LIMITS) != 2:
            raise ValueError('limits must have exactly 2 elements.')

        if cls.LIMITS[1] <= cls.LIMITS[0]:
            raise ValueError('left limit must be strictly '
                             'less than the right limit.')

        # Assign descriptor to class
        cls.value = BoundedNumber(*cls.LIMITS)
        # For some reason, this is not called automatically.
        cls.value.__set_name__(owner=cls, name='value')
        super().__init_subclass__(**kwargs)


class OptionValue(Value):
    """Abstract base class for values with options.

    Typical use:

        .. code::python

            class Level(OptionValue):

                OPTIONS = (1, 2, 8)

            level = Level(2)  # valid
            level = Level(4)  # throws ValueError; not in (1, 2, 8)
    """
    OPTIONS: Iterable

    def __eq__(self, other) -> bool:
        if isinstance(other, self.__class__):
            return other.value == self.value
        else:
            return other == self.value

    def __init_subclass__(cls, **kwargs):
        if cls.OPTIONS is not None:
            try:
                cls.value = Options(*cls.OPTIONS)
                # For some reason, this is not called automatically.
                cls.value.__set_name__(owner=cls, name='value')
            except TypeError as exc:
                raise TypeError('OPTIONS class attribute must be iterable.') from exc
        else:
            cls.value = None

    def __float__(self):
        return float(self.value)

    def __int__(self):
        return int(self.value)

    def __lt__(self, other):
        return float(self) < float(other)

    def __le__(self, other):
        return float(self) <= float(other)

    def __gt__(self, other):
        return float(self) > float(other)

    def __ge__(self, other):
        return float(self) >= float(other)


class BoundedTuple:
    """Abstract base class for a bounded Tuple.
    """

    LIMITS: Tuple[float, float] = (None, None)
    LENGTH: int = None

    def __init__(self, *values):

        if len(values) != self.LENGTH:
            raise ValueError(f'number of provided values ({len(values)}) does'
                             f' not match {self.LENGTH}')

        if any(not isinstance(x, (int, float)) for x in values):
            raise TypeError(f'value inputs must be integers or floats: {values}.')

        for i, val in enumerate(values):
            setattr(self, f'_value{i}', val)  # performs checks

        self._values = values

    @property
    def values(self):
        return self._values

    def __getitem__(self, item) -> Number:
        return self.values[item]

    def __len__(self) -> int:
        return len(self.values)

    def __repr__(self):
        return f'{self.values!r}'

    def __eq__(self, other):
        if isinstance(other, BoundedTuple):
            return self.values == other.values
        else:
            return self.values == other

    def __ne__(self, other):
        return not self.__eq__(other)

    def __float__(self):
        return (float(v) for v in self.values)

    def __int__(self):
        return (int(v) for v in self.values)

    @classmethod
    def __init_subclass__(cls, **kwargs):
        if any(attr is None for attr in (cls.LIMITS, cls.LENGTH)):
            raise AttributeError('subclasses of BoundedTuple must define class'
                                 ' attributes "LENGTH" and "LIMITS"')

        if not isinstance(cls.LENGTH, int):
            raise TypeError('"LENGTH" must be an integer')

        if any([not isinstance(x, (int, float)) for x in cls.LIMITS]):
            raise TypeError('LIMITS must be integers or floats.')

        if len(cls.LIMITS) != 2:
            raise ValueError('LIMITS must have only two elements.')

        if cls.LIMITS[1] <= cls.LIMITS[0]:
            raise ValueError('left limit must be strictly '
                             'less than the right limit.')

        # Assign descriptors to class. Descriptors cannot be stored in tuples,
        # so they get added to te class with a dynamically generated id
        for i in range(0, cls.LENGTH):
            descriptor_id = f'_value{i}'
            setattr(cls, descriptor_id, BoundedNumber(*cls.LIMITS))
            getattr(cls, descriptor_id).__set_name__(owner=cls, name=f'_value{i}')
        super().__init_subclass__(**kwargs)


def clamp(val: Number, low: Number = None, high: Number = None) -> Number:
    """Clamps the input value to the range [low, high].

    If either boundary is not specified, the function will not clamp
    at that bound.

    Raises:
        TypeError if all inputs are not of the same type
    """
    if val is None:
        raise ValueError("Input value is None, which cannot be clamped!")

    t_set = {type(i) for i in (val, low or val, high or val)}
    if len(t_set) != 1:
        raise TypeError("'val', 'low', and 'high', must all be the same type.")

    if low is not None and high is not None and low >= high:
        raise ValueError("Low and high boundaries are incorrectly set!")

    if low is not None and val < low:
        return type(val)(low)
    elif high is not None and val > high:
        return type(val)(high)
    else:
        return val
