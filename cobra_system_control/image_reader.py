"""
file: image_reader.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

The file provides a way to interface with RawToDepth to
facilitate getting raw data at various entry points
from the DSP. The functionality of this module
should not be relied upon as it is likely to be
deprecated in the future.
"""
import enum
import os
from pathlib import Path

import Pyro5.api
import numpy as np
import serpent

from cobra_system_control.cobra_log import log


# location of files saved from r2d / frontend
IMG_DATA_PATH = Path('/run')


class ImageType(enum.Enum):
    """The types of images saved to the /tmp directory when
    using accumulation mode in R2D"""

    RAW = enum.auto()
    PHASE_0 = enum.auto()
    PHASE_1 = enum.auto()


IMAGE_PROPERTIES = {
            ImageType.RAW: ('*cobra_accumulated_raw*.bin', np.uint16),
            ImageType.PHASE_0: ('*cobra_phase0*.bin', np.float32),
            ImageType.PHASE_1: ('*cobra_phase1*.bin', np.float32),
        }


def encode(data: np.ndarray) -> bytes:
    """Converts numpy array to bytes prior to shipping over network"""
    return data.tobytes()


def decode(serialized_data, img_type: ImageType) -> np.ndarray:
    """Converts serial data to bytes, then a numpy array"""
    _, dtype = IMAGE_PROPERTIES[img_type]

    # pyro's serializer will convert to dict
    if isinstance(serialized_data, dict):
        data_bytes = serpent.tobytes(serialized_data)

    # if not serializing, we can skip this step
    else:
        data_bytes = serialized_data

    data_arr = np.frombuffer(data_bytes, dtype=dtype)
    if img_type is ImageType.RAW:
        return data_arr
    else:
        return np.reshape(data_arr, newshape=(-1, 320))


def get_and_decode(ir: 'TempImageReader', img_type: ImageType) -> np.ndarray:
    """Uses the image reader to request the temporary
     image over the network and decode it back into a numpy array."""
    attempts = 100
    for i in range(attempts):
        try:
            return decode(ir.get(img_type), img_type)
        except (IndexError, ValueError):
            log.debug('Could not get image after attempt %s / %s ,'
                      'trying again', i, attempts)


@Pyro5.api.behavior(instance_mode='single')
@Pyro5.api.expose
class TempImageReader:
    """Use to read images from the filesystem
    """

    def get(self, img_type: ImageType) -> bytes:
        """Reads an image from the filesystem according to
        ``img_stage``, and returns it.
        """
        log.debug('Requested image from disk')

        # pyro does not understand enums
        img_type = ImageType(img_type)
        if img_type is ImageType.RAW:
            dtype = np.uint16
        else:
            dtype = np.float32

        glob, _ = IMAGE_PROPERTIES[img_type]

        paths = list(IMG_DATA_PATH.glob(glob))
        if len(paths) == 0:
            raise FileNotFoundError(
                f'No matching file with glob {glob} in {IMG_DATA_PATH}')
        latest_file = max(paths, key=os.path.getctime)
        data = np.fromfile(latest_file, dtype=dtype)
        log.debug('Loaded latest file: %s', latest_file)
        log.debug('Returning encoded from disk with size %s', data.size)

        return encode(data)
