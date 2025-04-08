"""
file: cobra_log.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file sets up the logging handlers to log to
/var/log/messages.
"""
import logging
import logging.handlers
from pathlib import Path
import platform

import cobra_system_control

log = logging.getLogger(__name__)

logging_setup = False


def setup_logging():
    """Attempts to set up logging handlers

    Always sets up logging to console

    Will attempt to log to syslog via logging.handlers.SysLogHandler
    If the OS is determined to be Windows or MacOS then the syslog handler is
    not used, otherwise we assume this means the code is being run on a Jetson
    """
    # Set up with global state so the logger doesn't keep
    # getting remade if it already exists.
    # pylint: disable-next=global-statement
    global logging_setup
    if logging_setup:
        return

    logging.captureWarnings(True)
    log.setLevel(logging.INFO)
    formatter = logging.Formatter(
        '{asctime} {msecs:0>3.0f} {levelname:>9} {module:>20} '
        '{funcName:>20} {lineno:>5} {message}',
        style='{',
        datefmt='%b %d %H:%M:%S')

    # MacOS will not error out on UNIX socket instantiation...
    # Check for OS and do appropriate actions

    os = platform.system()
    if os == 'Windows' or os == 'Darwin' or not Path('/dev/log').exists():
        logfile = cobra_system_control.COBRA_DIR / 'cobra_log.log'
        logfile.touch(exist_ok=True)
        formatter = logging.Formatter(
            '{asctime}.{msecs:03.0f} {levelname:>9} {module:>25} '
            '{funcName:>25} {lineno:>5} {message}',
            style='{',
            datefmt='%Y%m%d %H:%M:%S')
        hdl = logging.handlers.RotatingFileHandler(logfile.expanduser(),
                                                   mode='a',
                                                   maxBytes=5 * 1024 * 1024,
                                                   backupCount=10)
    else:
        # try to set up syslog handler to/dev/log (default syslog unix socket)
        hdl = logging.handlers.SysLogHandler('/dev/log')

    hdl.setLevel(logging.DEBUG)
    hdl.setFormatter(formatter)
    log.addHandler(hdl)

    logging_setup = True
