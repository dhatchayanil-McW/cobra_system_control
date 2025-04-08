from contextlib import nullcontext as does_not_raise

import pytest as pt


@pt.mark.skip()
@pt.mark.parametrize("tp1_in, tp1_out, error", [
    pt.param(1, 0, None),
    pt.param(55, 54, None),
    pt.param(100, 99, None),
    pt.param(1e9, 1e9, pt.raises(RuntimeError)),

])
def test_getf_tp1_period(ffuncs, tp1_in, tp1_out, error):
    with error or does_not_raise():
        tp1 = ffuncs.getf_tp1_period(tp1_in, 1)
        assert tp1 == tp1_out
