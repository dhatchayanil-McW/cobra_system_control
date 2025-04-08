import contextlib
import functools
import signal
from typing import Tuple, ContextManager
from queue import Queue
from threading import Thread
import time

import Pyro5.api
import Pyro5.nameserver

import cobra_system_control
from cobra_system_control.cobra import Cobra
from cobra_system_control.device import Device
from cobra_system_control import remote
from cobra_system_control.cobra_log import log


# serializes jobs through Pyro
# ensures one call at a time through the Pyro object
Pyro5.config.SERVERTYPE = "multiplex"
Pyro5.config.POLLTIMEOUT = 5


def write_system_info(series: str, platform: str):
    with open('/run/lumotive/system_props', 'w') as f:
        f.write(f'{series}\t{platform}')


def read_system_info():
    try:
        with open('/run/lumotive/system_props', 'r') as f:
            ret = f.read()
        lret = ret.split()
        return lret[0], lret[1]
    except FileNotFoundError:
        raise ('System properties not found. Were things setup correctly?')


def main():
    """Discovers the system to host the right HW.

    Now, only M30 and the NCB are supported so this sequence is simplified.
    """
    # We already know who we are!
    log.debug('Going to host M30 on NCB')
    write_system_info('m30', 'nxp')
    host_cobra('m30', 'nxp')


class RunState:
    def __init__(self):
        self.isRunning = True

    def running(self):
        return self.isRunning

    def stop(self):
        self.isRunning = False


def exit_handler(cobra_daemon, msg_daemon,
                 ns_daemon, run_state, *_):
    """Callback for SIGTERM, SIGINT, SIGABRT"""
    log.warning('shutting down cobra from user signal')
    shutdown(cobra_daemon, msg_daemon,
             ns_daemon, run_state)


def shutdown(cobra_daemon, msg_daemon,
             ns_daemon, run_state):
    """Shut down all daemons related to Pyro."""

    log.debug('Received signal to shut down remote objects')

    if cobra_daemon is not None:
        cobra_daemon.shutdown()
        log.debug('Called shutdown on cobra daemon')

    if msg_daemon is not None:
        msg_daemon.shutdown()
        log.debug('Called shutdown on msg daemon')

    if ns_daemon is not None:
        ns_daemon.shutdown()
        log.debug('Called shutdown on nameserver daemon')

    run_state.stop()


def launch_nameserver() -> Tuple[Pyro5.api.Daemon, Thread]:
    """Creates and starts event loop for nameserver

    It is the callers responsibility to shutdown the daemon and join the thread.
    """

    ns_uri, nsdaemon, bcastsrv = Pyro5.nameserver.start_ns(host="127.0.0.1")

    ns_thread = Thread(target=nsdaemon.requestLoop, daemon=True)
    log.debug('Started nameserver')
    ns_thread.start()

    return nsdaemon, ns_thread


@contextlib.contextmanager
def launch_queue(ns: Pyro5.nameserver,
                 q: Queue) -> ContextManager[Tuple[Pyro5.api.Daemon, Thread]]:
    """Creates and starts event loop for message queue

    It is the callers responsibility to shutdown the daemon and join the thread.
    """

    msg_daemon = Pyro5.api.Daemon(host='127.0.0.1', port=9100)
    msg_uri = msg_daemon.register(q, remote.COBRA_MESSENGER_ID)
    ns.register(remote.COBRA_MESSENGER_ID, msg_uri)

    try:
        msg_thread = Thread(target=msg_daemon.requestLoop, daemon=True)
        msg_thread.start()
        log.info('Launched message queue')
        yield msg_daemon, msg_thread
    finally:
        msg_daemon.unregister(q)
        ns.remove(remote.COBRA_MESSENGER_ID)
        log.info('Cleaned up message daemon and unregistered from nameserver')


@contextlib.contextmanager
def launch_cobra(ns: Pyro5.nameserver,
                 c: 'Cobra') -> ContextManager[Tuple[Pyro5.api.Daemon, Thread]]:
    """Creates and starts event loop for Cobra instance

    It is the callers responsibility to shutdown the daemon and join the thread.
    """
    remote.register_serializable_classes()
    devices_registered = set()  # keep track to avoid adding same device twice
    cobra_daemon = Pyro5.api.Daemon(host='127.0.0.1', port=remote.PORT)

    try:
        # make Cobra, its sensor heads, and devices available
        cobra_uri = cobra_daemon.register(c, remote.COBRA_ID)
        for i in [
                c.img_reader, c.cmb_adc,
                c.cmb_laser_vlda_dac, c.cmb_sensor_v_dac,
                c.cmb_lcm_v_dac,
        ]:
            try:
                cobra_daemon.register(i)
            except Pyro5.errors.DaemonError:
                devices_registered.add(i)
                continue

        sh = c.sen
        sh_uri = cobra_daemon.register(sh)
        # Register the new DACs that don't inherit Device
        for dac in [sh.laser_ci_dac, sh.sh_laser_vlda_dac, sh.ito_dac]:
            if dac not in devices_registered:
                try:
                    cobra_daemon.register(dac)
                except Pyro5.errors.DaemonError:
                    devices_registered.add(dac)
                    continue

        devices = set(getattr(sh, attr) for attr in dir(sh)
                      if isinstance(getattr(sh, attr), Device)
                      and not callable(getattr(sh, attr))
                      and not attr.startswith("__"))
        for d in devices:
            if d not in devices_registered:
                try:
                    cobra_daemon.register(d)
                except Pyro5.errors.DaemonError:
                    devices_registered.add(d)
                    continue

        ns.register(remote.SENSOR_IDS[0], sh_uri)
        ns.register(remote.COBRA_ID, cobra_uri)
        log.debug(f'Cobra URI: {cobra_uri}')

        cobra_thread = Thread(target=cobra_daemon.requestLoop, daemon=True)
        cobra_thread.start()
        log.info(f'Launched remote {c.whoami} Cobra object.')
        yield cobra_daemon, cobra_thread

    finally:
        for d in devices_registered:
            try:
                cobra_daemon.unregister(d)
            except Pyro5.errors.DaemonError:
                continue
        cobra_daemon.unregister(c.sen)
        cobra_daemon.unregister(c)

        for item in [remote.COBRA_ID, *remote.SENSOR_IDS]:
            ns.remove(item)
        log.info('Unregistered and removed Cobra object.')


def host_cobra(sensor_type: str, board_type: str):
    global Queue

    # expose all Queue methods to Pyro
    Queue = Pyro5.api.expose(Queue)
    queue = Queue()

    # kick of nameserver thread and wait until it is available
    ns_daemon, ns_thread = launch_nameserver()
    while ns_daemon.nameserver.count() == 0:
        log.debug('Waiting for nameserver')
        time.sleep(0.1)

    # kick off message queue and cobra remote objects
    with launch_queue(ns_daemon.nameserver, queue) as (msg_daemon, msg_thread):
        log.info('Opening cobra instance...')

        myRunState = RunState()

        with Cobra.open(sensor_type, board_type=board_type, msg_queue=queue) as c_:
            with launch_cobra(ns_daemon.nameserver, c_) as (c_daemon, c_thread):
                exit_cb = functools.partial(
                    exit_handler, ns_daemon, msg_daemon,
                    c_daemon, myRunState)
                signal.signal(signal.SIGTERM, exit_cb)
                signal.signal(signal.SIGINT, exit_cb)
                signal.signal(signal.SIGABRT, exit_cb)

                while myRunState.running():
                    time.sleep(1)


if __name__ == "__main__":
    main()
