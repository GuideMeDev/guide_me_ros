import numpy

from utils import *
import time
import numpy as np
import numba as nb # jit - just in time!!
import builtins as bl
import matplotlib.pyplot as plt


# maybe some problems with PANDAS
# @nb.jit(nopython=True)
def cluster_loop(x: np.ndarray, x1: np.ndarray, x2: np.ndarray):
    # choosing p.c. points within a volume in front of the user
    f = np.argwhere((abs(x2[:, 1]) < 1.5) * (abs(x2[:, 0]) < 4) * (abs(x2[:, 2]) < 0.08) *
                    (np.append(abs(np.diff(x2[:, 2]) / np.diff(x1[:, 1])), 1) < 0.22) *
                    (np.append(abs(np.diff(x2[:, 2]) / np.diff(x2[:, 1])), 1) < 0.22)
                    )

    # applying RANSAC. choosing 50 clusters of 50 p.c. points in search for the cluster with the best planar fit
    r = np.random.randint(round(len(f) / 2), size=(10, 200))
    xr = np.zeros((r.shape[0], r.shape[1]))
    S = np.zeros((r.shape[0], 2))
    yr = np.zeros((r.shape[0], r.shape[1]))
    zr = np.zeros((r.shape[0], r.shape[1]))
    nn = np.zeros((r.shape[0], 4))
    for j in range(r.shape[0]):
        xr[j, :] = x2[f[r[j, :]], 0].T
        yr[j, :] = x2[f[r[j, :]], 1].T
        zr[j, :] = x2[f[r[j, :]], 2].T
        # linear fit to each cluster in its x-z and y-z planes.
        nx = np.polyfit(xr[j, :], zr[j, :], 1)
        xn = abs(np.dot(x[:, 0], nx[0]) + nx[1] - x[:, 2])
        ny = np.polyfit(yr[j, :], zr[j, :], 1)
        yn = abs(np.dot(x[:, 1], ny[1]) + ny[1] - x[:, 2])
        # measure the variation for between the samples and fit of each cluster
        S[j, :] = [np.std(xn), np.std(yn)]
        nn[j, :] = np.append(nx, ny)
    return f, r, S, nn

def plane_fit(I, XYZ, roll, pitch):
    xyz_length = len(XYZ)
    h1 = np.zeros(xyz_length)
    eul = np.zeros((xyz_length, 3))

    # plotting
    # ----
    # fig = plt.figure(figsize=(6, 10))
    # ax1 = fig.add_subplot(2, 1, 1)
    # ax2 = fig.add_subplot(2, 1, 2)
    # ax2.axis([-2, 2, -0.5, 1])
    # ax_img = ax1.imshow(I[0])
    # ax3 = ax2.plot([], [], '.')[0]
    # ax4 = ax2.plot([], [], '*')[0]
    # ----

    pcloud = []
    start_time = time.time()
    print("---start: %s seconds ---" % (time.time() - start_time))

    for i in range(xyz_length):
        if i % 10 == 0:
            print(f'i: {i} --- %s seconds ---{(time.time() - start_time)}')
        Xdr = XYZ[i]
        if i == 0:
            # using euler and translation from first frame
            h1[i] = 1.45
        else:
            # using euler and translation from previous frame
            h1[i] = h1[i - 1]
        print(f'i: {i}, h1[i]: { h1[i]}')
        eul[i] = np.array([roll[i] + 2 * np.pi / 180, -(pitch[i] + np.pi / 2), 0])
        tetax = eul[i, 0]
        tetay = eul[i, 1]
        tetaz = eul[i, 2]
        Rz = np.array([[np.cos(tetaz), - np.sin(tetaz), 0], [np.sin(tetaz), np.cos(tetaz), 0], [0, 0, 1]])
        Ry = np.array([[np.cos(tetay), 0, np.sin(tetay)], [0, 1, 0], [- np.sin(tetay), 0, np.cos(tetay)]])
        Rx = np.array([[1, 0, 0], [0, np.cos(tetax), - np.sin(tetax)], [0, np.sin(tetax), np.cos(tetax)]])
        high = [0, 0, h1[0]]

        R1 = np.dot(np.dot(Rz, Ry), Rx)
        height_vec = np.tile(high, (len(Xdr), 1))
        x = np.dot(R1, Xdr.T).T + height_vec
        x1 = x
        c4 = np.append(1, abs(np.diff(x1[:, 2]) / np.diff(x1[:, 1]))) < 0.22
        c3 = np.append(abs(np.diff(x1[:, 2]) / np.diff(x1[:, 1])), 1) < 0.22
        c1 = abs(x1[:, 1]) < 1.0
        c0 = abs(x1[:, 0]) < 4
        c2 = abs(x1[:, 2]) <= 0.20
        f = np.argwhere(c0 * c1 * c2 * c3 * c4)
        # choosing only points with height abs(z)<5cm
        f1 = abs(x1[f, 2] - np.percentile(x1[f, 2], 50, interpolation='midpoint')) < 0.05
        f = f[f1]
        x = x1[f]
        # linear fit to points in y-z axes
        n = np.polyfit(x[:, 1], x[:, 2], 1)
        xn = x[:, 1] * n[0] + n[1]

        f1 = abs(x[:, 2] - xn) < 0.03
        n = np.polyfit(x[f1, 1], x[f1, 2], 1)
        xn = x[:, 1] * n[0] + n[1]
        nx = n[0]
        f2 = abs(x[:, 2] - xn) < 0.02
        # linear fit to points in x-z axes
        n = np.polyfit(x[f2, 0], x[f2, 2], 1)
        xn = x[:, 0] * n[0] + n[1]
        ny = n[0]
        f3 = abs(x[:, 2] - xn) < 0.03

        # to find rotation matrix R2 around x, y and z axes derived from the above linear fit
        tetax = -np.arctan(nx)
        tetay = np.arctan(ny)
        tetaz = 0
        Rz = np.array([[np.cos(tetaz), - np.sin(tetaz), 0], [np.sin(tetaz), np.cos(tetaz), 0], [0, 0, 1]])
        Ry = np.array([[np.cos(tetay), 0, np.sin(tetay)], [0, 1, 0], [- np.sin(tetay), 0, np.cos(tetay)]])
        Rx = np.array([[1, 0, 0], [0, np.cos(tetax), - np.sin(tetax)], [0, np.sin(tetax), np.cos(tetax)]])
        R2 = np.dot(np.dot(Rz, Ry), Rx)

        # to find translation of the s.w. points in z axis
        h1new = np.mean(np.dot(np.dot(R2, R1), Xdr[f[f3], :].T).T, axis=0)
        h1new = h1new[2] + h1[i]
        h1[i] = h1[i] - h1new

        # 2nd approximation of the s.w. points of Xdr to x-y plane
        high = [0, 0, h1[i]]
        height_vec = np.tile(high, (len(Xdr), 1))
        x2 = np.dot(np.dot(R2, R1), Xdr.T).T + height_vec
        R = np.dot(R2, R1)
        eul[i, :] = np.array([np.arctan2(R[2, 1], R[2, 2]), - np.arcsin(R[2, 0]), 0])

        # # choosing p.c. points within a volume in front of the user
        # f = np.argwhere((abs(x2[:, 1]) < 1.5) * (abs(x2[:, 0]) < 4) * (abs(x2[:, 2]) < 0.08) * (
        #         np.append(abs(np.diff(x2[:, 2]) / np.diff(x1[:, 1])), 1) < 0.22) * (
        #                         np.append(abs(np.diff(x2[:, 2]) / np.diff(x2[:, 1])), 1) < 0.22))
        #
        # # applying RANSAC. choosing 50 clusters of 50 p.c. points in search for the cluster with the best planar fit
        # r = np.random.randint(round(len(f) / 2), size=(10, 200))
        # xr = np.zeros((r.shape[0], r.shape[1]))
        # S = np.zeros((r.shape[0], 2))
        # yr = np.zeros((r.shape[0], r.shape[1]))
        # zr = np.zeros((r.shape[0], r.shape[1]))
        # nn = np.zeros((r.shape[0], 4))
        # # s1 = np.zeros(len(I))
        # for j in range(r.shape[0]):
        #     xr[j, :] = x2[f[r[j, :]], 0].T
        #     yr[j, :] = x2[f[r[j, :]], 1].T
        #     zr[j, :] = x2[f[r[j, :]], 2].T
        #     # linear fit to each cluster in its x-z and y-z planes.
        #     nx = np.polyfit(xr[j, :], zr[j, :], 1)
        #     xn = abs(np.dot(x[:, 0], nx[0]) + nx[1] - x[:, 2])
        #     ny = np.polyfit(yr[j, :], zr[j, :], 1)
        #     yn = abs(np.dot(x[:, 1], ny[1]) + ny[1] - x[:, 2])
        #     # measure the variation for between the samples and fit of each cluster
        #     S[j, :] = [np.std(xn), np.std(yn)]
        #     nn[j, :] = np.append(nx, ny)

        f, r, S, nn = cluster_loop(x, x1, x2)

        s1 = np.zeros(len(I))
        # to find the angles of the cluster with the best fit to a plane
        S = sum(S.T).T
        fs = S == min(S)
        s1[i] = S[fs]
        nx = nn[fs, 0:2]
        ny = nn[fs, 2:4]
        fr = f[r[fs, :]].T
        # to find the p.c. points of the cluster with the best fit to a plane
        x = x2[fr, :]
        f = abs(x2[fr, 2] - np.mean(x2[fr, 2])) < 0.02
        fr = fr[f]
        x = x2[fr, :]
        # removing the mean of the points of the cluster and finding the plane with the best fit to cluster
        x[:, 0] = x[:, 0] - np.mean(x[:, 0])
        x[:, 1] = x[:, 1] - np.mean(x[:, 1])
        x11 = sum(x[:, 0] ** 2)
        x22 = sum(x[:, 1] ** 2)
        # x33 = sum(x[:, 2] ** 2)
        x12 = sum(x[:, 0] * x[:, 1])
        x13 = sum(x[:, 0] * x[:, 2])
        x23 = sum(x[:, 1] * x[:, 2])
        D = np.dot(x11, x22) - x12 ** 2
        a = np.dot(x23, x12) - np.dot(x13, x22)
        b = np.dot(x13, x12) - np.dot(x11, x23)
        # n1 represents the plane with the best fit to the cluster
        n1 = [a, b, D]
        n1 = n1 / np.linalg.norm(n1)
        # to find the rotation angles required to rotate the n1 plane to become parallel to x-y plane
        tetax = np.arctan2(n1[1], np.sqrt(n1[2] ** 2 + n1[0] ** 2))
        tetay = -np.arctan2(n1[0], np.sqrt(n1[2] ** 2 + n1[1] ** 2))
        tetaz = 0
        Rz = [[np.cos(tetaz), -np.sin(tetaz), 0], [np.sin(tetaz), np.cos(tetaz), 0], [0, 0, 1]]
        Ry = [[np.cos(tetay), 0, np.sin(tetay)], [0, 1, 0], [-np.sin(tetay), 0, np.cos(tetay)]]
        Rx = [[1, 0, 0], [0, np.cos(tetax), -np.sin(tetax)], [0, np.sin(tetax), np.cos(tetax)]]
        # 3rd rotation and translation of Xdr
        R3 = np.dot(np.dot(Rz, Ry), Rx)
        R = np.dot(R3, R)
        x3 = (np.dot(R, Xdr.T)).T + height_vec
        h1[i] = h1[i] - np.mean(x3[fr, 2])
        x3[:, 2] = x3[:, 2] - np.mean(x3[fr, 2])
        eul[i, :] = [np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(R[2, 0]), 0]
        # ax3.plot(x3[:,1],x3[:,2],'.')

        # ----
        # ax3.set_data(x3[:, 1], x3[:, 2])
        # ax4.set_data(x3[fr, 1], x3[fr, 2])
        # ----

        # ax5.set_data(x3[f,1],x3[f,2])
        # --------
        # ax_img.set_data(I[i])
        # fig.canvas.draw()
        # fig.canvas.flush_events()
        # fig.show()
        # ----------
        pcloud.append(x3)

    print("--- END  %s seconds ---" % (time.time() - start_time))
    return pcloud
