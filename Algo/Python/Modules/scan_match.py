#from src.guide_me_ros.Algo.Python.Modules.algo_utils import *
import numpy as np
from scipy import signal as sig

import src.guide_me_ros.Algo.Python.Modules.algo_utils as algo

def scan_match(pcloud_curr, pcloud_next, pRGB1_curr, pRGB1_next, yaw_curr, yaw_next, dxIMU_i, dyIMU_i, tx_prev):
    b1 = pcloud_curr * 1000 / algo.sc
    b2 = pcloud_next * 1000 / algo.sc
    trgb1 = -algo.weg_tex * pRGB1_curr
    trgb2 = -algo.weg_tex * pRGB1_next
    # find d-frame and t-frame for sample (i+1) and same for frame (i) but after yaw rotation
    yaw1 = yaw_curr - yaw_next
    tetaz0 = yaw1
    mpc1, mpc2, tmpc1, tmpc2, b1a, b1b = algo.find_dframe_tframe(b1, b2, trgb1, trgb2, algo.dxmin, algo.sizemx, algo.sizemy, algo.thz0, algo.weg_obst,
                                                            yaw1)
    # find translation of frame (i) to match frame (i+1)
    m2 = mpc2[:, algo.rangex_mpc2] + tmpc2[:, algo.rangex_mpc2]
    m1 = mpc1[algo.rangey_mpc1][:, algo.rangex_mpc1] + tmpc1[algo.rangey_mpc1][:, algo.rangex_mpc1]
    tx_curr = algo.xcross2_custom(m1, m2, dyIMU_i, dxIMU_i, algo.kkx, algo.kky)
    if not len(tx_prev):
        tx_prev = tx_curr
    tx_curr = 0.8 * tx_curr + 0.2 * tx_prev
    # find no floor
    # find segments below ground level (mpc2curbe) for frame (i+1)
    mpc2curbe, mpc2nofloor, mpc2floor = algo.choose_mean_range2(b2, algo.thz0_mean, algo.dxmin * 1.5, algo.dxmax, algo.sizemx, algo.sizemy)
    # find a second rotation angle (tetaz1) to match frames (i) and (i+1)
    rtb1a = np.copy(b1a)
    rtb1a[:, 0] = rtb1a[:, 0] - tx_curr[1]
    rtb1a[:, 1] = rtb1a[:, 1] - tx_curr[0]
    b1b[:, 0:2] = rtb1a[:, 0:2]
    yaw1 = np.arange(-1.2, 1.2, 0.4) * np.pi / 180
    rtmpc1, tetaz1, rtb1b = algo.correct_reg_angle2(rtb1a, b1b, mpc2, yaw1, algo.sizemx, algo.sizemy)
    # find segments below ground level (mpc1curbe) for frame (i) after rotation (things like the road, holes, etc)
    # and translation (rt)
    mpc1curbe, mpc1nofloor, mpc1floor = algo.choose_mean_range2(rtb1b, algo.thz0_mean, algo.dxmin * 1.5, algo.dxmax, algo.sizemx, algo.sizemy)
    # update tetaz and save translation and rotation values to the matrix yawt
    tetaz = tetaz0 + tetaz1 + 0
    yaw_t = [tetaz, tx_curr[0], tx_curr[1], tetaz0]
    # find dframes (mpc2_plus and mpc1_plus) above ground level for frames (i) and (i+1) (dyanmic / static objects)
    x = b2
    f = np.where((x[:, 2] > algo.thz0 * 2) * (x[:, 0] > algo.dxmin) * (x[:, 0] < algo.dxmax) * (abs(x[:, 1]) < algo.sizemy / 2))[0]
    ba = x[f, :]
    px2a = ba[:, 0]
    py2a = ba[:, 1] - algo.sizemy / 2
    py2a[py2a <= -algo.sizemy] += algo.sizemy
    mpc_zero = np.zeros((algo.sizemy, algo.sizemx))
    mpc = np.copy(mpc_zero)
    mpc[py2a.astype(int), (px2a.astype(int))] = x[f, 2]
    c = sig.convolve2d(mpc, np.ones((3, 3)) / 9, mode='same')
    # mpc2_plus = c > 2 unused
    x = rtb1b
    f = np.where((x[:, 2] > algo.thz0 * 2) * (x[:, 0] > algo.dxmin) * (x[:, 0] < algo.dxmax) * (abs(x[:, 1]) < algo.sizemy / 2))[0]
    ba = x[f, :]
    px2a = ba[:, 0]
    py2a = ba[:, 1] - algo.sizemy / 2
    py2a[py2a <= -algo.sizemy] += algo.sizemy
    mpc1 = np.copy(mpc_zero)
    mpc1[py2a.astype(int), (px2a.astype(int))] = x[f, 2]
    c = sig.convolve2d(mpc1, np.ones((3, 3)) / 9, mode='same')
    mpc1_plus = (c > 2)
    # show dframes(pcloud) above and below ground level
    minter_plus = np.round((mpc2 * mpc1_plus) > 0)
    minter_minus = np.round(mpc1curbe * mpc2curbe)
    return yaw_t, tx_curr, minter_plus, minter_minus
