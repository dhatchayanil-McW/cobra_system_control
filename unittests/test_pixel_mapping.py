import numpy as np
import pytest as pt

import cobra_system_control.pixel_mapping as pmap


def test_vector_match_cv2_custom_undistort_points():
    try:
        import cv2
        import cv2.fisheye
    except ModuleNotFoundError:
        pt.skip('cv2 not available')

    pm = pmap.DEFAULT_PIXEL_MAPPING
    if not pm.fisheye:
        pt.skip('Default pixel map should be of fisheye type')
    custom_uv, custom_theta_phi = pm.generate_mapping_arrays()

    ### Generate old arrays using cv2.fisheye.undistortPoints()
    u = np.arange(0, pm.n_cols, dtype=np.float32)
    v = np.arange(0, pm.n_rows, dtype=np.float32)
    U, V = np.meshgrid(u, v)
    uv_array = np.stack((U, V), axis=2)
    k = np.reshape(pm.intrinsic, (3, 3)).astype(np.float32)
    uout = cv2.fisheye.undistortPoints(uv_array, k, pm.dist)  #pylint: disable=no-member
    xypp = np.reshape(uout, (-1, 2))
    theta_phi_array = pmap.xypp2theta_phi(xypp)
    uv_array = np.stack([U.ravel(), V.ravel(), np.ones_like(U).ravel()], axis=1)

    np.testing.assert_array_equal(custom_uv, uv_array)
    np.testing.assert_array_equal(custom_theta_phi, theta_phi_array)
