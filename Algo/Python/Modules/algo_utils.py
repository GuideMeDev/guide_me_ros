from matplotlib import pyplot as plt
import numpy as np
import numba as nb
from numpy import array
from numpy import cos, sin, dot, array, copy, pi, tile, diff, percentile, polyfit, arctan, dot, mean, arcsin, arctan2, \
    std, matmul
from scipy import signal as sig
import time
import cProfile, pstats, io
from pstats import SortKey
import scipy.io as io

sc = 20
weg_obst = 5
weg_tex = 3
thz0 = 80 / sc
thz0_mean = 100 / sc
dxmin = 1600 / sc
dxmax = 4000 / sc
sizemx = int(5000 / sc)
sizemy = int(5200 / sc)
kkx = kky = 6
rangex_mpc2 = np.arange(dxmin, dxmax).astype(int)
rangex_mpc1 = np.arange(dxmin + 400 / sc, dxmax - 400 / sc).astype(int)
rangey_mpc1 = np.arange(600 / sc, sizemy - 600 / sc).astype(int)
dlen = 2800 // sc
dwi = 250 // sc
k3 = 5000 / sc


# Functions for plane_fit Module
def choose_mean_range(b, thz0=0, thzh=0, dxmax=0, sizemx=2, sizemy=2):
    # use only values with x>0
    b1 = b[b[:, 0] > 0, :]
    # sort the P.C. vector based on ascending x-axis values and name the new variable x
    idxs1 = np.argsort(b1[:, 0])
    x = b1[idxs1, :]
    # sort the x vector based on ascending y-axis values and name the new variable x
    idxs2 = np.argsort(x[:, 1])
    x = x[idxs2, :]
    # smooth z-axis values of vector x
    win_size = 30
    conv = np.ones(win_size) / win_size
    x[:, 2] = np.convolve(x[:, 2], conv, mode='same')
    # choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles above the ground)
    f = (x[:, 0] > 40) * (x[:, 0] < dxmax) * (x[:, 2] > thz0) * (x[:, 2] < thzh) * (abs(x[:, 1]) < sizemy / 2)
    # here we define the pixels for the x-y-axes
    ba = x[f, :]
    px2a = ba[:, 0]
    py2a = ba[:, 1] + sizemy / 2
    # here we create the image mpc2
    mpc2 = np.zeros((sizemy, sizemx))
    # x[np.array([0,1,2]), np.array([0,1,1])] = 1
    mpc2[(px2a.astype(int) - 1), py2a.astype(int)] = 1

    # choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles above and below the ground)
    nofloor_th = 2
    f = (abs(x[:, 2]) > nofloor_th)
    ba = x[f, :]
    px2a = ba[:, 0]
    py2a = ba[:, 1] + sizemy / 2
    mpc2nofloor = np.zeros((sizemy, sizemx))
    mpc2nofloor[(px2a.astype(int) - 1), py2a.astype(int)] = 1

    # choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles below the ground):
    f = (abs(x[:, 2]) < 2.9) * (abs(x[:, 1]) < 1 * 1000 / 25) * (abs(x[:, 0]) < 3.5 * 1000.0 / 25) * (
            abs(x[:, 0]) > 1 * 1000 / 25)
    ba = x[f, :]
    px2a = ba[:, 0]
    py2a = ba[:, 1] + sizemy / 2
    mpc2floor = np.zeros((sizemy, sizemx))
    mpc2floor[(px2a.astype(int) - 1), py2a.astype(int)] = 1
    return mpc2, mpc2nofloor, mpc2floor


def correct_reg_angle(b1=None, b2=None, thz0=None, thzh=None, dxmax=None, yaw1=None, show=None, sizemx=None,
                      sizemy=None):
    # the first target here is to rotate b1 around z-axis by angle yaw1
    # the second target here is to transform b1 and b2 to obstacle frames mpc1 and mpc2
    # transforming b2 to obstacle frame mpc2
    f = np.where((b2[:, 2] > thz0) * (b2[:, 0] > 40) * (b2[:, 0] < dxmax))
    b2 = b2[f[0], :]
    px2 = b2[:, 0]
    py2 = b2[:, 1] - sizemy / 2
    py2[py2 <= -260] += 260
    # here we devide the obstacle values in the z-axis to 3 levels
    pz2 = b2[:, 2]
    pz2[pz2 > thzh] = 0
    pz2[pz2 < 0] = -1
    pz2[pz2 > 3] = 2
    mpc2 = np.zeros((sizemy, sizemx))
    mpc2[(px2.astype(int) - 1), py2.astype(int)] = pz2

    # transforming b2 to obstacle frame mpc2a
    f = np.where((abs(b2[:, 2]) > 8) * (b2[:, 0] > 40) * (b2[:, 0] > 0) * (b2[:, 0] < dxmax))
    b2a = b2[f[0], :]
    px2a = b2a[:, 0]
    py2a = b2a[:, 1] - sizemy / 2
    mpc2a = np.zeros((sizemy, sizemx))
    mpc2a[(px2a.astype(int) - 1), py2a.astype(int)] = 1
    # in the following code we rotate b1 yaw 1 and transform the rotated b1 points to obstacle frame mpc1
    # choose specific region within b1
    f = np.where((b1[:, 2] > thz0) * (b1[:, 0] > 40) * (b1[:, 0] < dxmax))
    # find the pixel values px and py withing the selected region
    b1 = b1[f[0], :]
    px = b1[:, 0]
    py = b1[:, 1]
    # here we devide the obstacle values in the z-axis to 3 levels
    pz = b1[:, 2]
    pz[pz > thzh] = 0
    pz[pz < 0] = -1
    pz[pz > 3] = 2
    # here we define variable b2a that is similar to mpc2 except it has points instead of pixels
    b2a = np.array([px2.T, py2.T - sizemy / 2, pz2.T]).T
    # here we rotate points of the selected region in variable b1 by angle yaw1
    tetaz = yaw1
    Rz = [[cos(tetaz), - sin(tetaz)], [sin(tetaz), cos(tetaz)]]
    t1 = dot(Rz, [px.T, py.T]).T
    # here we transform the rotated points frame mpc1
    px1 = t1[:, 0]
    py1 = t1[:, 1] + sizemy / 2
    mpc1 = np.zeros((sizemy, sizemx))
    mpc1[(px1.astype(int) - 1), py1.astype(int)] = pz
    # here we define variable b1a that is similar to mpc2 except it has points instead of pixels
    b1a = np.array([px1.T, py1.T - sizemy / 2, pz.T]).T
    return mpc1, mpc2, mpc2a, tetaz, b1a, b2a


def correct_reg_angleRGB(b1=None, b2=None, trgb1=None, trgb2=None, thz0=None, thzh=None, dxmax=None, yaw1=None,
                         show=None, sizemx=None, sizemy=None, *args, **kwargs):
    # this function is similar to the function correct_reg_angle, except here instead of assigning height value
    # for each x-y pixel it assigns to each point in b1 and b2 a graylevel value,
    # and the outputs are frames mpc1RGB and mpc2RGB
    f = np.where((b2[:, 2]) > thz0 * (b2[:, 0] > 40) * (b2[:, 0] < dxmax) * (abs(b2[:, 1]) < sizemy / 2))[0]
    b2 = b2[f, :]
    px2 = b2[:, 0]
    py2 = b2[:, 1] + sizemy / 2
    pz2 = trgb2[f]
    mpc2 = np.zeros((sizemy, sizemx))
    mpc2[(px2.astype(int) - 1), py2.astype(int)] = pz2

    f = np.where((abs(b2[:, 2]) > 5) * (b2[:, 0] > 40) * (b2[:, 0] > 0) * (b2[:, 0] < dxmax))[0]
    b2a = b2[f, :]
    px2a = b2a[:, 0]
    py2a = b2a[:, 1]
    mpc2a = np.zeros((sizemy, sizemx))
    mpc2a[(px2a.astype(int) - 1), py2a.astype(int)] = 1

    f = np.where((b1[:, 2]) > thz0 * (b1[:, 0] > 40) * (b1[:, 0] < dxmax) * (abs(b1[:, 1]) < sizemy / 2))[0]
    b1 = b1[f, :]
    px = b1[:, 0]
    py = b1[:, 1]
    pz = trgb1[f]
    s = []
    b2a = np.array([px2.T, py2.T, pz2.T]).T
    tetaz = yaw1
    Rz = [[cos(tetaz), - sin(tetaz)], [sin(tetaz), cos(tetaz)]]
    t1 = dot(Rz, [px.T, py.T]).T
    px1 = t1[:, 0]
    py1 = t1[:, 1] + sizemy / 2
    mpc1 = np.zeros((sizemy, sizemx))
    mpc1[(px1.astype(int) - 1), py1.astype(int)] = pz
    b1a = np.array([px1.T, py1.T - sizemy / 2, pz.T]).T
    return mpc1, mpc2, mpc2a, tetaz, b1a, b2a


# Functions for Scan match module
def choose_mean_range2(b=None, thz0=None, dxmin=None, dxmax=None, sizemx=None, sizemy=None):
    # use only values with x>0
    b1 = b[b[:, 0] > 0, :]
    # sort the P.C. vector based on ascending x-axis values and name the new variable x
    idxs1 = np.argsort(b1[:, 0])
    x = b1[idxs1, :]
    # sort the x vector based on ascending y-axis values and name the new variable x
    idxs2 = np.argsort(x[:, 1])
    x = x[idxs2, :]
    # smooth z-axis values of vector x
    win_size = 30
    conv = np.ones(win_size) / win_size
    x[:, 2] = np.convolve(x[:, 2], conv, mode='same')
    # choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles above the ground)
    f = (x[:, 0] < dxmax) * (x[:, 2] < -thz0) * (x[:, 0] > dxmin) * (abs(x[:, 1]) < sizemy / 2)
    # here we define the pixels for the x-y-axes
    ba = x[f, :]
    px2a = ba[:, 0]
    py2a = ba[:, 1] + sizemy / 2
    py2a[py2a <= -260] += 260

    # here we create the image mpc2
    mpc2 = np.zeros((sizemy, sizemx))
    mpc2[(py2a.astype(int), px2a.astype(int) - 1)] = 1

    # choosing specific area within x (next we transform coordinates to pixels to create an image of obstacles above and below the ground)
    f = (x[:, 2] - thz0 * 2 > 0) * (abs(x[:, 1]) < sizemy / 2) * (x[:, 0] < dxmax) * (x[:, 0] > dxmin)
    ba = x[f, :]
    px2a = ba[:, 0]
    py2a = ba[:, 1] - sizemy / 2
    py2a[py2a <= -260] += 260
    mpc2nofloor = np.zeros((sizemy, sizemx))
    mpc2nofloor[py2a.astype(int), (px2a.astype(int) - 1)] = x[f, 2]

    # choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles below the ground)
    f = (abs(x[:, 2]) < thz0) * (abs(x[:, 1]) < sizemy / 2) * (x[:, 0] < dxmax) * (x[:, 0] > dxmin)
    ba = x[f, :]
    px2a = ba[:, 0]
    py2a = ba[:, 1] - sizemy / 2
    py2a[py2a <= -260] += 260
    mpc2floor = np.zeros((sizemy, sizemx))
    mpc2floor[py2a.astype(int), (px2a.astype(int) - 1)] = 1
    return mpc2, mpc2nofloor, mpc2floor


def correct_reg_angle2(b1a=None, rtb1=None, mpc2=None, yaw1=None, sizemx=None, sizemy=None):
    # this function is similar to the function correct_reg_angle2,
    # except here we rotate the points of b1 by a range of angles
    # in search for best fit between mpc1 and mpc2
    px = b1a[:, 0]
    py = b1a[:, 1]
    pz = b1a[:, 2]
    pxb = rtb1[:, 0]
    pyb = rtb1[:, 1]
    pzb = rtb1[:, 2]
    s = []
    for j in range(len(yaw1)):
        tetaz = yaw1[j]
        Rz = [[cos(tetaz), - sin(tetaz)], [sin(tetaz), cos(tetaz)]]
        t1 = (dot(Rz, [px.T, py.T])).T
        px1 = t1[:, 0]
        py1 = t1[:, 1] - sizemy / 2
        py1[py1 < -sizemy] += sizemy
        mpc1 = np.zeros((sizemy, sizemx))
        mpc1[py1.astype(int), (px1.astype(int) - 1)] = pz
        s.append(np.sum(np.sum((mpc2 * mpc1))))

    f = np.argwhere(s == max(s))[0]
    tetaz = yaw1[f[0]]
    Rz = [[cos(tetaz), - sin(tetaz)], [sin(tetaz), cos(tetaz)]]
    t1 = (dot(Rz, [pxb.T, pyb.T])).T
    px1 = t1[:, 0]
    py1 = t1[:, 1] + sizemy / 2
    b1b = np.array([px1.T, py1.T - sizemy / 2, pzb.T]).T
    t1 = dot(Rz, [px.T, py.T]).T
    px1 = t1[:, 0]
    py1 = t1[:, 1] - sizemy / 2
    py1[py1 < -sizemy] += sizemy
    mpc1 = np.zeros((sizemy, sizemx))
    px1i = px1.astype(int)
    py1i = py1.astype(int)
    mpc1[py1i, (px1i - 1)] = pz
    mpc1[py1i, px1i] = pz
    mpc1[py1i, (px1i - 2)] = pz
    mpc1[py1i + 1, (px1i - 1)] = pz
    mpc1[py1i - 1, (px1i - 1)] = pz
    return mpc1, tetaz, b1b


def xcross2_custom(m1=None, m2=None, dyIMU=None, dxIMU=None, kkx=None, kky=None):
    # TODO: Add explanation regarding the function
    s = np.zeros((kkx * 2, kky * 2))
    kx = np.arange(-kkx, kkx)
    ky = np.arange(-kky, kky)

    m1_shape_0_divide_to_2 = m1.shape[0] / 2
    m1_shape_1_divide_to_2 = m1.shape[1] / 2
    m2_shape_0_divide_to_2 = m2.shape[0] / 2
    m2_shape_1_divide_to_2 = m2.shape[1] / 2

    for j1 in ky:
        ystart = int((m2_shape_0_divide_to_2 - m1_shape_0_divide_to_2 + 1 + j1 - dyIMU))
        yend = int((m2_shape_0_divide_to_2 + m1_shape_0_divide_to_2 + j1 - dyIMU)) + 1
        for j2 in kx:
            xstart = int(m2_shape_1_divide_to_2 - m1_shape_1_divide_to_2 + 1 + j2 - dxIMU)
            xend = int(m2_shape_1_divide_to_2 + m1_shape_1_divide_to_2 + j2 - dxIMU) + 1
            m2a = m2[ystart: yend, xstart: xend]
            s[j1, j2] = np.sum(np.sum(m1 * m2a))

    idxs = np.argwhere(s > np.max(np.max(s)) * 0.9)
    fx, fy = idxs[:, 0], idxs[:, 1]
    tx1 = np.array([dyIMU - ky[np.mean(fy).astype(int)], dxIMU - kx[np.mean(fx).astype(int)]])
    return tx1


# @nb.jit(nopython=True)
def find_dframe_tframe(b1, b2, trgb1=None, trgb2=None, dxmin=None, sizemx=None, sizemy=None, thz0=None, weg_obst=None,
                       yaw1=None):
    # TODO: Add explanation regarding the function
    b2_column_0 = b2[:, 0]
    #abs_to_b2_column_1 = np.abs(b2[:, 1]) <<-- fot JIT
    # abs_to_b2_column_1 = abs(b2[:, 1])

    f = np.where(
        (b2_column_0 > dxmin + 1) *
        (b2_column_0 < sizemx - 10) *
        (abs(b2[:, 1]) < sizemy - 10)
    )[0]

    b2_as_int = np.copy(b2[f, :]).astype(int)
    b2_as_int_column_0 = b2_as_int[:, 0]
    ################################ TODO!!: Change all image numpy coordinates according to this section!! since its the opposite (y is x, x is y)
    px2_from_b2_as_int = b2_as_int_column_0
    py2 = b2_as_int[:, 1] - sizemy / 2
    py2[py2 <= -sizemy] += sizemy
    py2_as_int_from_b2 = py2.astype(int)

    pz2 = np.copy(b2_as_int[:, 2])
    pz2[pz2 < thz0] = -1
    pz2[pz2 > thz0] = weg_obst
    pz2[abs(b2_as_int[:, 2]) < thz0] = 0

    px2_from_b2_minus_1_as_int = (px2_from_b2_as_int - 1).astype(int)

    dmpc2 = np.zeros((sizemy, sizemx))
    dmpc2[py2_as_int_from_b2, px2_from_b2_minus_1_as_int] = pz2
    dmpc2[py2_as_int_from_b2, px2_from_b2_as_int] = pz2
    dmpc2[py2_as_int_from_b2, (px2_from_b2_as_int - 2)] = pz2
    dmpc2[(py2_as_int_from_b2 + 1), px2_from_b2_minus_1_as_int] = pz2
    dmpc2[(py2_as_int_from_b2 - 1), px2_from_b2_minus_1_as_int] = pz2

    # find t-frame
    f = np.where(
        (b2_as_int_column_0 > dxmin) *
        (b2_as_int_column_0 < sizemx / 4 * 3 - 10) *
        (abs(b2_as_int[:, 1]) < sizemy - 10) *
        (abs(b2_as_int[:, 2]) < thz0)
    )[0]

    tb2 = b2_as_int[f, :]
    px2 = tb2[:, 0]
    py2 = tb2[:, 1] - sizemy / 2
    py2[py2 <= -sizemy] += sizemy
    pz2 = trgb2[f, 1]
    tmpc2 = np.zeros((sizemy, sizemx))
    tmpc2[py2.astype(int), (px2 - 1).astype(int)] = pz2

    # find d-frame
    f = np.where(
        (b1[:, 0] > dxmin) *
        (b1[:, 0] < sizemx - 10) *
        (abs(b1[:, 1]) < sizemy - 10)
    )[0]


    ####################
    b1 = b1[f, :]

    px1 = np.copy(b1[:, 0])
    py1 = np.copy(b1[:, 1])
    pz1 = np.copy(b1[:, 2])
    pz1[pz1 < thz0] = -1
    pz1[pz1 > thz0] = weg_obst
    pz1[np.abs(b1[:, 2]) < thz0] = 0
    tetaz = yaw1

    cos_teta_Z = cos(tetaz)
    sin_teta_Z = sin(tetaz)
    # Rz = [[cos(tetaz), -sin(tetaz)], [sin(tetaz), cos(tetaz)]]
    Rz = [[cos_teta_Z, -sin_teta_Z], [sin_teta_Z, cos_teta_Z]]

    t1 = dot(Rz, [px1.T, py1.T]).T
    b1a = np.copy(b1)
    b1a[:, ::2] = t1
    b1a[:, 2] = pz1
    px1 = t1[:, 0]
    py1 = t1[:, 1] - sizemy / 2
    py1[py1 <= -sizemy] += sizemy
    b1b = b1a
    b1b[:, 2] = b1[:, 2]
    dmpc1 = np.zeros((sizemy, sizemx))
    dmpc1[py1.astype(int), (px1 - 1).astype(int)] = pz1

    # find t-frame
    f = np.where(
        (b1[:, 0] > dxmin) *
        (b1[:, 0] < sizemx / 4 * 3 - 10) *
        (abs(b1[:, 1]) < sizemy - 10) *
        (abs(b1[:, 2]) < thz0)
    )[0]

    tb1_f = np.copy(b1[f, :])
    px1 = tb1_f[:, 0]
    py1 = tb1_f[:, 1]
    pz1 = trgb1[f, 1]
    t1 = dot(Rz, [px1.T, py1.T]).T

    px1 = t1[:, 0]
    py1 = t1[:, 1] - sizemy / 2
    py1[py1 <= -sizemy] += sizemy
    tmpc1 = np.zeros((sizemy, sizemx))
    tmpc1[py1.astype(int), (px1 - 1).astype(int)] = pz1
    # reshaping to 2d
    return dmpc1, dmpc2, tmpc1, tmpc2, b1a, b1b


# Performance Metric for Plane fit module
def plane_fit_metric(x2):
    fin = np.argwhere(abs(x2[:, 1]) < 0.5 & abs(x2[:, 0] - 3) < 0.5 & abs(x2[:, 2]) < 0.08)[0]
    fout = np.argwhere(abs(x2[:, 1]) > 0.5 & abs(x2[:, 1]) < 1.5 & abs(x2[:, 0] - 3) > 0.5 & abs(x2[:, 2]) < 0.08)[0]
    s = std(x2[fin, 2]) / std(x2[fout, 3])
    return s
