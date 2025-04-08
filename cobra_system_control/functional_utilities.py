"""
file: functional_utilities.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines common functions that are generally
useful relating to function wrapping regarding waits or
try/excepts and filesystem or git interactions.
"""
import argparse
import contextlib
import os
from pathlib import Path
import subprocess
import time
from typing import Callable, Sequence

import numpy as np

from cobra_system_control.cobra_log import log
import cobra_system_control.exceptions as cobex


def wait_for_true(predicate: Callable[[], bool], n_tries: int,
                  interval_s: float, timeout_msg: str = None):
    """Wraps the predicate, waiting it to return True within
    n_tries, with the given interval. If the predicate does not
    return True, a TimeoutError is raised with the provided message
    """
    for _ in range(n_tries):
        if predicate():
            return
        time.sleep(interval_s)
    timeout_msg = (
            timeout_msg
            or f'predicate {predicate.__name__} never evaluated true after '
               f'{n_tries} tries with {interval_s} seconds between attempts')
    raise TimeoutError(timeout_msg)


def try_me(func, default=None, expected_e=(Exception, )):
    """Wraps func in a try/except block, returning default
    if one of the expected exceptions was caught

    Example:
    Use with attribute accesses like
        rval = try_me(lambda: class.attribute)
    """
    try:
        return func()
    except expected_e as e:
        log.debug("Error %e from func %s", e, func)
        return default


def add_host_arg(parser: argparse.ArgumentParser):
    """Adds the hostname argument for ``Cobra.remote`` call inputs
    """
    parser.add_argument(
        '-z', '--host',
        default=None,
        type=str,
        help=('host name or IP address of the compute module '
              'to which you wish to connect')
    )


@contextlib.contextmanager
def pushd(new_dir):
    prev_dir = os.getcwd()
    os.chdir(new_dir)
    try:
        yield
    finally:
        os.chdir(prev_dir)


def _subprocess_lines(command):
    """Helper to split multiline output into a list of lines.
    """
    try:
        proc = subprocess.run(
            command,
            shell=True,
            capture_output=True,
            text=True,
            check=True,
        )
    except subprocess.CalledProcessError:
        log.warning('Failed to issue shell command %s', command)

        ret = [""]
    else:
        ret = proc.stdout.strip().split('\n')
    return ret


def get_git_sha(repo: str) -> str:
    """Gets the first 8 hex characters of the git sha
       Starts by attempting to get the git sha from
       the repository in /home/root/. This
       repository only exists in dev scenarios. If
       the repository doesn't exist, gets the git sha
       from /etc/m30_sha_<name>.txt, which is put into
       the filesystem by our Yocto build.
    """
    p = Path('~', repo).expanduser()
    if os.path.exists(p):
        with pushd(p):
            cmd = 'git rev-parse --short=8 HEAD'
            out = _subprocess_lines(cmd)
        if out[0] == '':
            return 'no_git_repo'
        else:
            return out[0]
    else:
        pre = f'/etc/m30_sha_{repo}.txt'
        try:
            with open(pre, "r", encoding='utf8') as f:
                return f.readline().strip()
        except OSError as e:
            log.error('failed to read file at %s to '
                      'get git sha for %s because %s',
                      pre, repo, str(e))
            return '<no_sha_info>'


def get_git_clean_status(repo: str) -> bool:
    """Gets the clean status of a git repo by checking
    the status of tracked files
    """
    p = Path('~', repo).expanduser()
    if os.path.exists(p):
        with pushd(p):
            cmd = 'git status --porcelain --untracked-files=no'
            out = _subprocess_lines(cmd)
        if out[0] == '':
            return True
        else:
            return False
    else:
        return '<no_git_status>'


def get_compute_hostname() -> str:
    try:
        return _subprocess_lines('hostname')[0]
    except OSError:
        return None


def free_i2c_bus():
    """Frees up the i2c busses if they are left in a bad state
    """
    os.system('gpioset 4 19=0')
    os.system('gpioset 3 19=0')
    os.system('gpioset 3 19=1')
    os.system('gpioset 4 19=1')
    time.sleep(1)


def get_common_length(**seq_params):
    # check that all possible sequence are the same length
    length_map = {name: len(x) for name, x in seq_params.items()
                  if isinstance(x, Sequence) and len(x) > 1}
    unique_lengths = np.unique(np.array(list(length_map.values())))
    if len(unique_lengths) > 1:
        raise cobex.ScanPatternSizeError(
            f'At least two parameter sequences have different lengths: '
            f'{length_map}; this is not allowed.')
    return unique_lengths[0] if len(unique_lengths) == 1 else 1
