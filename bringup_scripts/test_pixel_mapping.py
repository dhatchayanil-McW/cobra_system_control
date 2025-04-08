import cv2
import glob
import numpy as np
import matplotlib.pyplot as plt

from cobra_system_control.pixel_mapping import DEFAULT_PIXEL_MAPPING, FISHEYE_PIXEL_MAPPING, xypp2XYZ, uv_rect2xyp, k_inverse,xypp2theta_phi, SUPERSAMPLED_PIXEL_MAPPING

from lidar_calibration_suite.camera_calibration.camera_distortion_calibrator import CameraDistortionCalibrator


def get_dat():
    fid = './20220627_164519_image_0001.npy'
    dat = np.load(fid)
    print(dat.shape)
    dat /= np.max(dat)
    dat *= 255
    fig, ax = plt.subplots()
    ax.imshow(dat)
    ax.set_xlabel('dat')
    return dat


def main(dat):

    f = FISHEYE_PIXEL_MAPPING


    k = np.reshape(f.intrinsic, (3,3)).astype(np.float32)
    d = f.dist[0:4].astype(np.float32)
    # print(k.shape, k)
    # print(d.shape, d)
    # dims = dat.shape[::-1]
    # new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    #     k, d, dims, np.eye(3), balance=0, fov_scale=3)
    # #new_k = k
    # print('newk', new_k.shape, new_k)
    # map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    #     k, d, np.eye(3), new_k, dims, cv2.CV_32FC1) # cv2.CV_16SC2)
    # print('map1', map1.shape, map1)
    # print('map2', map2.shape, map2)
    #
    # umin = 0#-320
    # umax = 640#320
    # vmin = 0 #-240
    # vmax = 480 #240
    #
    # uimg = cv2.remap(dat, map1, map2,
    #                  interpolation=cv2.INTER_NEAREST,
    #                  borderMode=cv2.BORDER_CONSTANT,
    # )


    # x = np.linspace(-3,3, 101)
    # y = np.linspace(-2,2, 101)
    # XX, YY = np.meshgrid(x,y)
    #
    # xy = np.stack((XX,YY), axis=2)
    # dout = cv2.fisheye.distortPoints(xy, k, d)
    # print('dout', dout)
    # fig, ax = plt.subplots(ncols=2)
    # ax[0].imshow(dout[...,0])
    # ax[1].imshow(dout[...,1])
    # ax[0].set_xlabel('dout')

    u = np.arange(640).astype(np.float32)
    v = np.arange(480).astype(np.float32)
    UU, VV = np.meshgrid(u,v)
    uv = np.stack((UU,VV), axis=2)
    uout = cv2.fisheye.undistortPoints(uv, k, d)
    print('uout', uout)
    fig, ax = plt.subplots(ncols=2)
    ax[0].imshow(uout[...,0])
    ax[1].imshow(uout[...,1])
    ax[0].set_xlabel('uout')

    fig, ax = plt.subplots()
    ax.pcolormesh(uout[...,0], uout[...,1], dat)


    tp = xypp2theta_phi(uout)


def ccal(dat):
    df = DEFAULT_PIXEL_MAPPING
    fids = glob.glob('/home/rossuthoff/test_cam/*.npy')
    frames = [np.load(f) for f in fids]
    cdc = CameraDistortionCalibrator(frames, fisheye=False)
    cdc.load_points()
    cdc.run_cal()
    print(cdc.k, cdc.d)
    k = np.reshape(cdc.k, (3,3)).astype(np.float32)
    d = cdc.d.ravel().astype(np.float32)
    print(d.shape)
    u = np.arange(640).astype(np.float32)
    v = np.arange(480).astype(np.float32)
    UU, VV = np.meshgrid(u,v)
    uvr = np.stack((UU.ravel(),VV.ravel()), axis=1)
    uv = np.stack((UU,VV), axis=2)
    print('uvr', uvr.shape)
    print('uv', uv.shape)
    nout = cv2.undistortPoints(uvr, k, d)
    print('nout', nout.shape)
    nout = np.reshape(nout, (480,640,2))

    eout = cv2.undistortPoints(uvr, k, df.dist)
    eout = np.reshape(eout, (480,640,2))

    cdc = CameraDistortionCalibrator(frames, fisheye=True)
    cdc.load_points()
    cdc.run_cal()
    print(cdc.k, cdc.d)

    uout = cv2.fisheye.undistortPoints(uv, cdc.k.astype(np.float32), cdc.d.ravel().astype(np.float32))
    print('uout', uout.shape)


    uv, tp = df.generate_mapping_arrays()
    xypp = np.reshape(df.xypp, (480, 640, 2))

    fig, ax = plt.subplots(nrows=4, ncols=2)
    ax[0,0].imshow(xypp[...,0], vmin=-3,vmax=3)
    ax[0,1].imshow(xypp[...,1], vmin=-3,vmax=3)
    ax[0,0].set_xlabel('existing')

    ax[1,0].imshow(eout[...,0], vmin=-3,vmax=3)
    ax[1,1].imshow(eout[...,1], vmin=-3,vmax=3)
    ax[1,0].set_xlabel('existing normal')

    ax[2,0].imshow(nout[...,0], vmin=-3,vmax=3)
    ax[2,1].imshow(nout[...,1], vmin=-3,vmax=3)
    ax[2,0].set_xlabel('normal')

    ax[3,0].imshow(uout[...,0], vmin=-3,vmax=3)
    ax[3,1].imshow(uout[...,1], vmin=-3,vmax=3)
    ax[3,0].set_xlabel('fisheye')

    fig, ax = plt.subplots(ncols=4)
    ax[0].pcolormesh(xypp[...,0], xypp[...,1], dat)
    ax[0].set_xlabel('existing')
    ax[1].pcolormesh(eout[...,0], eout[...,1], dat)
    ax[1].set_xlabel('existing normal')
    ax[2].pcolormesh(nout[...,0], nout[...,1], dat)
    ax[2].set_xlabel('normal')
    ax[3].pcolormesh(uout[...,0], uout[...,1], dat)
    ax[3].set_xlabel('fisheye')
    m = -3
    mx = 3
    ax[0].set_xlim([m,mx])
    ax[0].set_ylim([m,mx])
    ax[1].set_xlim([m,mx])
    ax[1].set_ylim([m,mx])
    ax[2].set_xlim([m,mx])
    ax[2].set_ylim([m,mx])
    ax[3].set_xlim([m,mx])
    ax[3].set_ylim([m,mx])

if __name__ == "__main__":
    #dat = get_dat()
    #main(dat)
    #ccal(dat)


    s = SUPERSAMPLED_PIXEL_MAPPING
    s.generate_mapping_arrays()

    d = DEFAULT_PIXEL_MAPPING
    d.generate_mapping_arrays()

    plt.show()
