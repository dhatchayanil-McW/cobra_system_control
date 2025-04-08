"""
file: remote.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file provides the utilities needs to set up
the Pyro5 remote object.
"""
import codecs
import contextlib
import ipaddress
import logging
import pathlib
import pickle
from typing import TypeVar, ContextManager

import numpy as np
import pandas as pd

import Pyro5.api
import Pyro5.errors
import Pyro5.server

log = logging.getLogger(__name__)


T = TypeVar('T')

NS_HOST = '0.0.0.0'
HOST = '127.0.0.1'
PORT = 9095
COBRA_ID = 'cobra'
COBRA_MESSENGER_ID = 'cobra_messenger'
SENSOR_IDS = ('A', )

# add non-settings objects which require network serialization here
# subclasses of Device are auto-registered
SERIALIZATION_REGISTRY = {
    np.ndarray,
    np.int64,
    np.uint16,
    np.int8,
    np.int16,
    pathlib.Path,
    pathlib.PosixPath,
    pathlib.PurePath,
    pathlib.PurePosixPath,
    pathlib.PureWindowsPath,
    pd.DataFrame,
}


def register_for_serialization(cls):
    """Class decorator which adds the class to
    ``SERIALIZATION_REGISTRY``"""
    SERIALIZATION_REGISTRY.add(cls)
    return cls


def register_serializable_classes():
    """Registers classes in ``SERIALIZATION_REGISTRY``
    for serialization with Pyro5"""
    for cls in SERIALIZATION_REGISTRY:
        Pyro5.api.register_class_to_dict(cls, to_pyro_dict)
        Pyro5.api.register_dict_to_class(cls.__name__, from_pyro_dict)


def serialize(obj) -> str:
    """Pickles an object and encodes it as a string"""
    return codecs.encode(pickle.dumps(obj), "base64").decode()


def deserialize(s: str) -> T:
    """Reverses `serialize` to unpickle an object from a string"""
    return pickle.loads(codecs.decode(s.encode(), "base64"))


def to_pyro_dict(obj) -> dict:
    """Produces a serialized, Pyro-compatible dict.

    The __class__ key is necessary so Pyro can recognize this as a dictionary
    which needs to be deserialized.
    """
    return {'data': serialize(obj),
            '__class__': obj.__class__.__name__}


def from_pyro_dict(_: str, d: dict) -> T:
    """Returns an object from a dictionary

    The Pyro5 method ``register_dict_to_class`` expects a callable with
    two arguments, but because we are using pickle we do not use the first one.
    """
    return deserialize(d['data'])


def get_safe_runner(errors, error_limit=10):
    """Returns a function which catches Pyro5 communication errors
    the specified number of times in a row before error-ing out.
    """

    sequential_errors = 0

    def run_safe(func, *args, **kwargs):
        nonlocal sequential_errors
        try:
            ret = func(*args, **kwargs)
            sequential_errors = 0
            return ret
        except errors as e:
            log.error('Error caught in "run_safe": %s', e)
            sequential_errors += 1

            if sequential_errors == error_limit:
                log.error('Errored too many times in a row.\n'
                          'Last error was %s', e)
                raise e
            else:
                return None
        except Exception as e:
            log.critical('Fatal error caught in "run_safe": %s', e)
            raise e

    return run_safe


@contextlib.contextmanager
def remote_lookup(idn: str, hostname: str = None,
                  username: str = "root", password: str = None,
                  log_warnings: bool = True,
) -> ContextManager:
    """Looks for Pyro5 nameserver and attempts to get the designated object.

    If an ssh tunnel to the device currently exists this will crash,
    and because we do not want to make this code disable
    things a user might have set up outside of python so it will be
    up to the end-user to handle checking if the tunnel
    exists or not when using this method and handle the exception.

    """

    # control of M30 provided through Pyro5
    # with tunneling handled by python we will always check @ 127.0.0.1
    # if on a remote PC you have to know the hostname of the target device
    if log_warnings:
        log.info('Connecting to %s', hostname)
    if hostname:
        # Pylint disable since sshtunnel only needs to be installed on client
        import sshtunnel  # pylint: disable=import-error

        try:
            ipaddress.IPv4Address(hostname)

            # start the tunnels
        except ipaddress.AddressValueError:
            log.debug('Hostname is not IP address; using .local for scan.')
            host_str = f'{hostname}.local'
        else:
            host_str = hostname

        tun = sshtunnel.open_tunnel(
            host_str,
            ssh_config_file='~/.ssh/config',
            ssh_username=username,
            ssh_password=password,
            local_bind_addresses=[('', 9090), ('', 9095)],
            remote_bind_addresses=[('127.0.0.1', 9090), ('127.0.0.1', 9095)]
        )

        # obviously we want to avoid hardcoded passwords
        # right now we have no choice...
        # but this can be extended to use keys fairly easily

        log.info('Starting tunnel to %s', hostname)
        tun.start()
        log.info('SSH tunnel is up')
    try:
        # if we fail to find nameserver on '127.0.0.1' ssh tunnel is not up
        # or
        # we are on the compute and the nameserver is not up
        # either way we can't move forward
        ns = Pyro5.api.locate_ns('127.0.0.1')
    except Pyro5.errors.NamingError:
        if log_warnings:
            log.critical(
                "Unable to find Nameserver, make sure the remote object"
                " is running on your target")
        raise
    uri = ns.lookup(idn)
    register_serializable_classes()
    with Pyro5.api.Proxy(uri) as c:
        try:
            yield c
        # implement finer exception handling from remote object
        # here if deemed necessary
        except Pyro5.errors.CommunicationError as e:
            msg = (f'Cannot communicate with idn: {idn}. '
                   'Please verify the remote object is available and the '
                   'ssh tunnel is active (if accessing via external PC).')
            raise Pyro5.errors.CommunicationError(f'{msg}\n{e}')
        except Exception:
            print("Pyro traceback:")
            print("".join(Pyro5.errors.get_pyro_traceback()))
            raise

    try:
        # stop the tunnel if we can
        tun.stop()
        log.info('SSH tunnel successfully closed.')
    except NameError:
        pass
