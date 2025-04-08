"""
file: fe_ctl.py

Copyright (C) 2023 Lumotive, Inc. All rights reserved.

This file defines the interface between the SCC and the
frontend. More information about the frontend design
can be found in cobra_raw2depth/front-end-cpp/fe_design.md
"""
import socket
from cobra_system_control.cobra_log import log


def fe_send(command, timeout):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(('localhost', 1234))
        ba = bytearray(1)
        ba[0] = command
        s.send(ba)
        rdata = s.recv(1)
        if rdata[0] != command:
            log.error("incorrect value received=%s, expected=%s", rdata[0], command)

        s.close()
    except ConnectionRefusedError:
        log.error('Connection refused from socket. Continuing')


def fe_start_streaming(mode: int, timeout: float = 4.0):
    if mode < 0 or mode > 9:
        log.error("mode must be in the range of [0, 9] but is %s", mode)
        return
    fe_send((0 & 0x3) | (mode << 2), timeout)


def fe_stop_streaming(timeout: float = 4.0):
    fe_send(0x40 | (0 & 0x3), timeout)


def fe_reload_cal_data(timeout: float = 4.0):
    fe_send(0x80 | (0 & 0x3), timeout)


def fe_get_mode(num_rows: int, reduce_mode: int,
                aggregate: bool) -> int:
    """Determines the streaming mode to put the
    front end into based on the args

    Lines are 640 * 3 = 1920 long.

    NCB modes:
    0 = 480 x 6 + 1 RGB888
    1 = 20 x 6 + 1 RGB888
    2 = 20 x 2 + 1 RGB888
    3 = (20 x 2 + 1) x 10 RGB888
    4 = 8 x 6 + 1 RGB888
    5 = 8 x 2 + 1 RGB888
    6 = (8 x 2 + 1) x 10 RGB888
    7 = 6 x 6 + 1 RGB888
    8 = 6 x 2 + 1 RGB888
    9 = (6 x 2 + 1) x 10 RGB888

    """
    if reduce_mode == 0 and aggregate:
        log.error('Must use reduce mode if aggregating')

    # [DMFD, TA, AG]
    fe_mode_map = {
        480: [0, -1, -1],
        20:  [1, 2, 3],
        8:   [4, 5, 6],
        6:   [7, 8, 9],
    }

    try:
        fe_mode = fe_mode_map[num_rows][reduce_mode + int(aggregate)]
    except KeyError:
        log.error('unsupported number of rows: %s; assuming 8 rows', num_rows)
        fe_mode = fe_mode_map[8][reduce_mode]

    if fe_mode < 0:
        log.error('tap accumulation not supported with full frames; '
                  'setting to DMFD mode')
        fe_mode = fe_mode_map[480][0]

    return fe_mode
