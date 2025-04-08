"""
file: state.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines the allow Sensor Head states and
the function that controls transitioning between
states.
"""
from enum import Enum, auto
import functools
from typing import Mapping


class State(Enum):
    """Sensor Head State Enum

    A SenorHead has five states:
    1. INITIALIZED - state on SensorHead instantiation or after disconnect()
    2. CONNECTED - state after connect()
    3. READY - state after setup() or disable()
    4. ENERGIZED - state after enable() or stop()
    5. SCANNING - state after start()

    """
    INITIALIZED = auto()
    CONNECTED = auto()
    READY = auto()
    ENERGIZED = auto()
    SCANNING = auto()


class StateError(Exception):
    """Errors associated with a SensorHead being in the wrong state
    """


# Access to _state is fine here.
# pylint: disable=protected-access
def state_transition(transitions: Mapping[State, State]):
    """Function generator which defines the state transitions which result
    from calling the decorated function.

    Args:
        transitions: a mapping of "enter: exit" which are allowed.
    """

    def outer(func):
        @functools.wraps(func)
        def inner(self, *args, **kwargs):
            if self._state not in transitions.keys():
                names = [state.name for state in transitions.keys()]
                msg = (f'method "{func.__name__}" is only accessible from '
                       f'state(s): {",".join(names)} '
                       f'but is currently in {self._state.name}')
                raise StateError(msg)
            exit_state = transitions[self._state]
            ret = func(self, *args, **kwargs)
            self._state = exit_state
            return ret

        return inner

    return outer
# pylint: enable=protected-access
