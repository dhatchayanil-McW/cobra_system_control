"""
file: exceptions.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provides specific exceptions.
"""
from cobra_system_control import remote


@remote.register_for_serialization
class CalibrationError(Exception):
    pass


@remote.register_for_serialization
class CalibrationSizeError(CalibrationError):
    pass


@remote.register_for_serialization
class ScanPatternError(Exception):
    pass


@remote.register_for_serialization
class ScanPatternPowerError(Exception):
    pass


@remote.register_for_serialization
class ScanPatternSizeError(Exception):
    pass


@remote.register_for_serialization
class ScanPatternValueError(Exception):
    pass


@remote.register_for_serialization
class BadPayload(Exception):
    pass


@remote.register_for_serialization
class FPGAFileError(Exception):
    pass


@remote.register_for_serialization
class PerVirtualSensorMetadataError(Exception):
    pass


@remote.register_for_serialization
class MemoryMapError(Exception):
    pass


@remote.register_for_serialization
class MemoryMapPeriphAlignmentError(MemoryMapError):
    """Indicates that a MemoryMapPeriph has a base address that is not a
    multiple of the peripheral's size in bytes, rounded up to the nearest
    power of two.
    """


@remote.register_for_serialization
class MemoryMapFieldAlignmentError(MemoryMapError):
    """Indicates that a MemoryMapField is not aligned properly in the
    addressable space according to its size and position.
    """


@remote.register_for_serialization
class MemoryMapFieldOverflowError(MemoryMapError):
    """Indicates that a MemoryMapField object is outside the addressable range
    of it's MemoryMapPeriph.
    """


@remote.register_for_serialization
class MemoryMapFieldAttributeError(MemoryMapError):
    """Indicates that a MemoryMapField object was defined with an unrecognized
    attribute.
    """


@remote.register_for_serialization
class MemoryMapFieldValueError(MemoryMapError):
    """Indicates that an attribute of a MemoryMapField object had an illegal
    value.
    """
