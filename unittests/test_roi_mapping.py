from contextlib import nullcontext as does_not_raise
import pytest as pt

from cobra_system_control.roi_mapping import RoiMapping, myround
from cobra_system_control.pixel_mapping import DEFAULT_PIXEL_MAPPING
import cobra_system_control.exceptions as cobex



@pt.mark.parametrize("angles, s_rows, row_roi, error", [
    pt.param(None, None, 20, pt.raises(cobex.CalibrationError)),
    pt.param([0], [0], 20, pt.raises(cobex.CalibrationError)),
    pt.param([1,2,3], None, 20, None),
    pt.param(None, [20,40,50], 20, None),
    ])
def test_call(angles, s_rows, row_roi, error, lcmsa):
    error = error or does_not_raise()
    pm = DEFAULT_PIXEL_MAPPING
    rm = RoiMapping(a2a_coefficients=(45.439334676855,
                                      -0.238209222626,
                                      0.0002051934385,
                                      -0.000000285), pixel_mapping=pm,
                    lcm_assembly=lcmsa,
                    )
    with error:
        o, r = rm(angles=angles, s_rows=s_rows, roi_rows=row_roi)
        assert len(o) == len(r)


@pt.mark.parametrize("val, base, rdata, error", [
    pt.param(4.4, 4, 4, None),
    pt.param(140, 4, 140, None),
    pt.param(7.2, 3, 6, None),
    pt.param('foo', 3, 3, pt.raises(ValueError)),
    pt.param(1, 'foo', 1, pt.raises(ValueError)),
])
def test_myround(val, base, rdata, error):
    with error or does_not_raise():
        rval = myround(val, base)
        assert rval == rdata


@pt.mark.parametrize("angles, trim", [
    pt.param([0,0,0], True),
    pt.param([0,0,0], False),
    ])
def test_trim(angles, trim, lcmsa):
    pm = DEFAULT_PIXEL_MAPPING
    rm = RoiMapping(a2a_coefficients=(45.439334676855,
                                      -0.238209222626,
                                      0, 0), pixel_mapping=pm,
                    lcm_assembly=lcmsa,
                    )
    o, r = rm(angles=angles, roi_rows=20, trim_duplicates=trim)
    assert len(o) == len(r)
    if trim:
        assert len(o) == len(set(angles))
    else:
        assert len(o) == len(angles)
