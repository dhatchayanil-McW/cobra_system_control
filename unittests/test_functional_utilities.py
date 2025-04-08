import time

from unittest.mock import MagicMock, call
import pytest as pt

from cobra_system_control.functional_utilities import wait_for_true


@pt.mark.parametrize('predicate, n_tries, interval_s, error', [
    (MagicMock(return_value=True), 3, 0.5, None),
    (MagicMock(return_value=False), 5, 0.1, pt.raises(TimeoutError)),
])
def test_wait_for_true(predicate: MagicMock, n_tries,
                       interval_s, error):
    predicate.__name__ = 'function'
    start = time.time()
    if error is None:
        wait_for_true(predicate, n_tries, interval_s)
        assert time.time() - start < 0.5
        predicate.assert_called_once()
    else:
        with error:
            wait_for_true(predicate, n_tries, interval_s)
            assert time.time() - start > 0.5
            predicate.assert_called_with([call()] * n_tries)
