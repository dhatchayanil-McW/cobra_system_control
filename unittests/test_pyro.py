import contextlib
import dataclasses as dc

import pytest

from cobra_system_control import remote
from cobra_system_control.boot_scripts.host_cobra import launch_nameserver, launch_cobra
from cobra_system_control.calibration_data import CalData, CalItem
from cobra_system_control.cobra import Cobra
from cobra_system_control.device import Device
from cobra_system_control.itof import FrameSettings
from cobra_system_control.sensor_head import SensorHead


@pytest.fixture(scope="function")
def remote_cobra(mock: bool, cobra: Cobra) -> Cobra:
    if mock:
        pytest.skip('Cannot host a mocked remote cobra object. Skipping test.')
    with launch(cobra):  # launch real cobra object via pyro
        with Cobra.remote() as c:  # acquire cobra remote object
            yield c


@dc.dataclass
class CustomClass:
    a: int
    b: int = 5


# pylint: disable=no-member
def test_pyro_exposure_and_behavior():
    for cls in [Cobra, SensorHead, Device] + Device.__subclasses__():
        assert cls._pyroExposed is True
        assert cls._pyroInstancing[0] == 'single'
# pylint: enable=no-member


def test_pyro_pickle_serialization():
    c1 = CustomClass(10)
    c2 = remote.from_pyro_dict('', remote.to_pyro_dict(c1))
    assert c1 == c2
    assert c1 is not c2


def test_register_for_serialization():
    remote.register_for_serialization(CustomClass)
    assert CustomClass in remote.SERIALIZATION_REGISTRY


def test_all_registered_classes():
    assert FrameSettings in remote.SERIALIZATION_REGISTRY
    assert CalData in remote.SERIALIZATION_REGISTRY
    assert CalItem in remote.SERIALIZATION_REGISTRY


@contextlib.contextmanager
def launch(c: Cobra):
    ns_daemon, ns_thread = launch_nameserver()
    ns = ns_daemon.nameserver

    with launch_cobra(ns, c) as (c_daemon, c_thread):
        yield
        c_daemon.shutdown()
        c_thread.join()
        ns_daemon.shutdown()
        ns_thread.join()


# pylint: disable-next=redefined-outer-name
def test_remote_cobra_read_write(golden_shas, remote_cobra: Cobra):
    assert remote_cobra.sen.debug.read_fields('git_sha') in golden_shas.values()
