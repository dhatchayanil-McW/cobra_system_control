"""
file: validation_utilities.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines a Descriptor class and subclasses that validate
the values of registers before being applied to hardware. This
helps ensure safety to the HW and avoid undefined or erroneous
operation.
"""
import dataclasses as dc
from typing import Callable, Union, Any, Sequence, Type, TypeVar


Number = Union[int, float]


def is_in_bounds(val: Number, lower_bound: Number,
                 upper_bound: Number, inclusive=True) -> bool:
    """Returns whether the value is between the bounds

    Args:
        val (Number): the value in question
        lower_bound (Number): the lower bound
        upper_bound (Number): the upper bound
        inclusive (bool): include the endpoints of the range (defaults true)

    Returns:
         (bool) whether ``value`` is between
         ``lower_bound`` and ``upper_bound``

    """
    if inclusive:
        return lower_bound <= val <= upper_bound
    else:
        return lower_bound < val < upper_bound


class Descriptor:
    """A textbook descriptor

    This is a base class that will be built upon to enforce
    arbitrary constraints.
    See https://docs.python.org/3/howto/descriptor.html for
    general info on descriptors.
    """

    def __set__(self, instance, value):
        instance.__dict__[self.name] = value

    def __get__(self, instance, owner):
        if not instance:
            return self
        return getattr(instance, self.name)

    def __set_name__(self, owner, name):
        # pylint: disable-next=attribute-defined-outside-init
        self.name = f'{owner.__name__}:{name}'


@dc.dataclass
class Register(Descriptor):
    """Descriptor used to validate the size of register values, and store
    information on the offset and position for ``ScanEntry``"""

    offset: int
    position: int
    size: int

    def __set__(self, instance, value):
        if not 0 <= value < 2 ** self.size:
            raise ValueError(
                f'Scan parameter {self.name} with value {value} is out '
                f'of bounds [0, {2 ** self.size - 1}]')
        super().__set__(instance, value)


class Bounded(Descriptor):
    """Enforces bounds on the provided value

    Typical use:

    .. code::python

        class A:

            x = Bounded(0, 10)

        a = A(5)
        print(a.x)  # prints "5"

        a = A(11)  # raises ValueError
    """

    def __init__(self, lower_bound, upper_bound):
        self.bounds = (lower_bound, upper_bound)

    def __set__(self, instance, value):
        if not is_in_bounds(value, *self.bounds):
            raise ValueError(
                f'Value for {self.name}: {value:.3f} is out of '
                f'the bounds {self.bounds}.')
        super().__set__(instance, value)


class Options(Descriptor):
    """Enforces the provided value is one of the given options

        Typical use:

        .. code::python

            class A:

                x = Options(2, 4, 8)

            a = A(2)
            print(a.x)  # prints "2"

            a = A(3)  # raises ValueError
        """

    def __init__(self, *opts):
        if len(opts) == 0:
            raise ValueError('No options were provided.')
        self.options = opts

    def __set__(self, instance, value):
        if value not in self.options:
            raise ValueError(f'Value for {self.name}: {value} is not'
                             f' in valid options: {self.options}')
        super().__set__(instance, value)


def typed(expected_type, cls=None):
    """Class decorator which enforces the provided expected type."""

    if cls is None:
        return lambda cls: typed(expected_type, cls)

    super_set = cls.__set__

    def __set__(self, instance, value):
        if not isinstance(value, expected_type):
            raise TypeError(f'expected {expected_type}')
        super_set(self, instance, value)
    cls.__set__ = __set__
    return cls


@typed((float, int))
class BoundedNumber(Bounded):
    """A bounded number descriptor

    Typical use:

    .. code::python

        class A:

            x = BoundedNumber(0, 10)

        a = A(5)
        print(a.x)  # prints "5"

        a = A('s')  # raises TypeError
        a = A(11)  # raises ValueError

    """
    VALID_TYPES = (float, int)

    # @typechecked
    def __init__(self, lower_bound: Number, upper_bound: Number):
        super().__init__(lower_bound, upper_bound)
        if not isinstance(lower_bound, (int, float)):
            raise TypeError('lower_bound is neither <int> or <float> type')
        if not isinstance(upper_bound, (int, float)):
            raise TypeError('upper_bound is neither <int> or <float> type')


T = TypeVar('T')


def cast(x: Any, t: Type[T]) -> T:
    """Casts the value ``x`` to type ``T``

    This function is largely a shortcut useful for class constructors to accept
    different types, but cast them to the same type.

    Typical use:

    .. code::python

        class A:
            x = BoundedNumber(0, 10)

        class B:

            def __init__(y: Union[A, int], z: Union[A, float]):
                self.y = cast(y, A)  # Performs bounds check on y if int
                self.z = cast(z, A)  # Performs bounds checks on z if float

    :param x: any value to cast to ``T``
    :param t: the type ``T`` which has a single argument constructor
    :return: instance of ``T`` which was created from x
    """
    return x if isinstance(x, t) else t(x)


def cast_to_sequence(val: Union[Sequence, int, float],
                     length, func: Callable = lambda x: x):
    try:
        if len(val) == 1:
            return [func(val[0])] * length
        elif len(val) != length:
            raise ValueError(f'cannot cast sequence of length {len(val)} '
                             f'to length {length}')
        return list(map(func, val))
    except TypeError:
        return [func(val)] * length
