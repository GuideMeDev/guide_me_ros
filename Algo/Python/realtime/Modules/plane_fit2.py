from Modules.utils import *
from Modules.plane_init import plane_init
from mpl_toolkits import mplot3d
def remove_mean_of_points(x: np.ndarray) -> list:
    x[:, 0] = x[:, 0] - np.mean(x[:, 0])
    x[:, 1] = x[:, 1] - np.mean(x[:, 1])
    x11 = np.asarray([np.sum(x[:, 0] ** 2)])
    x22 = np.asarray([np.sum(x[:, 1] ** 2)])
    # x33 = sum(x[:, 2] ** 2)
    x12 = np.asarray([np.sum(x[:, 0] * x[:, 1])])
    x13 = np.asarray([np.sum(x[:, 0] * x[:, 2])])
    x23 = np.asarray([np.sum(x[:, 1] * x[:, 2])])
    D: np.float64 = np.dot(x11, x22) - np.sum(x[:, 0] * x[:, 1]) ** 2
    # D: np.float64 = np.dot(np.asarray([x11]), np.asarray([x22])) - x12 ** 2
    a: np.float64 = np.dot(x23, x12) - np.dot(x13, x22)
    b: np.float64 = np.dot(x13, x12) - np.dot(x11, x23)
    return [a, b, D]

# First method
def filter_plane_1(x1: np.ndarray, x2: np.ndarray):
    # choosing p.c. points within a volume in front of the user
    diff_array_from_X2 = np.diff(x2[:, 2])
    f = np.argwhere((abs(x2[:, 1]) < 1.5) * (abs(x2[:, 0]) < 4) * (abs(x2[:, 2]) < 0.08) *
                    (np.nan_to_num(np.append(abs(diff_array_from_X2 / np.diff(x1[:, 1])), 1) < 0.22)) *
                    (np.nan_to_num(np.append(abs(diff_array_from_X2 / np.diff(x2[:, 1])), 1) < 0.22))
                    )
    # applying RANSAC. choosing 50 clusters of 50 p.c. points in search for the cluster with the best planar fit
    array_dimensions: tuple = (400)
    r = np.random.randint(len(f), size=array_dimensions)
    fr = f[r]
    f1 = filter_plane_pc(x2[fr[:,0],:])
    fr1=fr[f1,0]
    x_plane=x2[fr1,:]
    return x_plane,fr1,r

def filter_plane_pc(x2):
    """
    In this function we first substract the center of mass from the points 
    and then rotate the points around x and y axis in order to fit the S.W. with X-Y plane
    and keep only points with distance in z-axes that is <th1
    th1 is the threshold for points distance from z_axis (ideal distance is 0)
    """
    th1=0.03
    xr=x2[:,0].T
    yr=x2[:,1].T
    zr=x2[:,2].T
    x_cm = np.zeros(3)
    zr_0=np.ones(len(zr))
    x_cm[0] = sum(xr*zr_0)/sum(zr_0)
    x_cm[1] = sum(yr*zr_0)/sum(zr_0)
    f_cm = (abs(xr-x_cm[0])<0.3) * (abs(yr-x_cm[1])<0.3)
    x_cm[2] = np.mean(zr[f_cm])
    #ax = plt.axes(projection='3d')
    # ax.plot3D(xr, yr, zr, 'ob')
    # ax.plot3D([x_cm[0]],[x_cm[1]],[x_cm[2]],'or')
    # translation
    xr_t=xr-x_cm[0];yr_t=yr-x_cm[1];zr_t=zr-x_cm[2]
    # rotation
    rot_x = np.arctan(zr_t/yr_t)*180/pi
    teta_x=np.percentile(rot_x,50)
    teta=teta_x*pi/180
    cos_teta = np.cos(teta)
    sin_teta = np.sin(teta)
    Rx = np.array([[1,0,0],[0,cos_teta,sin_teta],[0,-sin_teta,cos_teta]])
    rot_y = np.arctan(zr_t/xr_t)*180/pi
    teta_y = np.percentile(rot_y,50)
    teta = teta_y*pi/180
    cos_teta = np.cos(teta)
    sin_teta = np.sin(teta)
    Ry = np.array([[cos_teta,0,sin_teta],[0,1,0],[-sin_teta,0,cos_teta]])
    #x_cm[0] = xr_t; x_cm[1] = yr_t; x_cm[2] = zr_t
    X_t = Ry @ Rx @ [xr_t,yr_t,zr_t]
    f1=abs(X_t[2,:])<th1
    # plt.subplot(2,1,1)
    # plt.plot(yr,zr,'o')
    # plt.plot(yr[f1],zr[f1],'.r');plt.axis([-2,2,-0.2,0.2])
    # plt.subplot(2,1,2)
    # plt.plot(xr,zr,'o');plt.plot(xr[f1],zr[f1],'.r')
    # plt.axis([2,4,-0.2,0.2])
    return f1
    


def plane_fit2(I, XYZ, roll, pitch,pitch_mean,h1_prev = INIT_H1):
    # xyz_length = len(XYZ)
    # h1 = np.zeros(xyz_length)
    # eul = np.zeros((xyz_length, 3))

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
    h1 = h1_prev
    # for i in range(xyz_length):
    Xdr = XYZ
    # using euler and translation from previous frame
    # previous_frame_index -= 1
    # h1 = h1[previous_frame_index]
    # previous_frame_index = i + 1
    # print(f'i: {i}, h1[i]: { h1[i]}')
    eul0 = np.array([roll, pitch, 0])
    #eul = np.array([roll + 2 * np.pi / 180, -(pitch + np.pi / 2), 0])
    tetax = eul0[0]
    tetay = eul0[1]
    tetaz = eul0[2]

    cos_teta_Z = np.cos(tetaz)
    sin_teta_Z = np.sin(tetaz)

    cos_teta_Y = np.cos(tetay)
    sin_teta_Y = np.sin(tetay)

    cos_teta_X = np.cos(tetax)
    sin_teta_X = np.sin(tetax)

    Rz = np.array([[cos_teta_Z, - sin_teta_Z, 0], [sin_teta_Z, cos_teta_Z, 0], [0, 0, 1]])
    Ry = np.array([[cos_teta_Y, 0, sin_teta_Y], [0, 1, 0], [- sin_teta_Y, 0, cos_teta_Y]])
    Rx = np.array([[1, 0, 0], [0, cos_teta_X, - sin_teta_X], [0, sin_teta_X, cos_teta_X]])
    high = [0, 0, h1]

    R1 = np.dot(np.dot(Rz, Ry), Rx)
    height_vec = np.tile(high, (len(Xdr), 1))
    x = np.dot(R1, Xdr.T).T + height_vec
    x1 = x

    divide_array = abs(np.diff(x1[:, 2]) / np.diff(x1[:, 1]))
    c4 = np.nan_to_num(np.append(1, divide_array)) < 0.22
    c3 = np.nan_to_num(np.append(divide_array, 1)) < 0.22
    c1 = abs(x1[:, 1]) < 1.0
    c0 = abs(x1[:, 0]) < 4
    c2 = abs(x1[:, 2]) <= 0.15
    f = np.argwhere(c0 * c1 * c2 * c3 * c4)
    if len(f)/len(x1) < 0.03:
        divide_array = abs(np.diff(x1[:, 2]) / np.diff(x1[:, 1]))
        c4 = np.nan_to_num(np.append(1, divide_array)) < 0.22
        c3 = np.nan_to_num(np.append(divide_array, 1)) < 0.22
        c1 = abs(x1[:, 1]) < 1.0
        c0 = abs(x1[:, 0]) < 4
        c2 = abs(x1[:, 2]) <= 0.2
        f = np.argwhere(c0 * c1 * c2 * c3 * c4)
    # *****************************

    # choosing only points with height abs(z)<5cm
    try:
        f1 = abs(x1[f, 2] - np.percentile(x1[f, 2], 50, interpolation='midpoint')) < 0.05
        f = f[f1]
    except:
        print("no points for plane")
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
    # ******************************

    # to find rotation matrix R2 around x, y and z axes derived from the above linear fit
    tetax = -np.arctan(nx)
    tetay = np.arctan(ny)
    tetaz = 0

    cos_teta_Z = np.cos(tetaz)
    sin_teta_Z = np.sin(tetaz)

    cos_teta_Y = np.cos(tetay)
    sin_teta_Y = np.sin(tetay)

    cos_teta_X = np.cos(tetax)
    sin_teta_X = np.sin(tetax)

    # Rz = np.array([[np.cos(tetaz), - np.sin(tetaz), 0], [np.sin(tetaz), np.cos(tetaz), 0], [0, 0, 1]])
    # Ry = np.array([[np.cos(tetay), 0, np.sin(tetay)], [0, 1, 0], [- np.sin(tetay), 0, np.cos(tetay)]])
    # Rx = np.array([[1, 0, 0], [0, np.cos(tetax), - np.sin(tetax)], [0, np.sin(tetax), np.cos(tetax)]])

    Rz = np.array([[cos_teta_Z, - sin_teta_Z, 0], [sin_teta_Z, cos_teta_Z, 0], [0, 0, 1]])
    Ry = np.array([[cos_teta_Y, 0, sin_teta_Y], [0, 1, 0], [- sin_teta_Y, 0, cos_teta_Y]])
    Rx = np.array([[1, 0, 0], [0, cos_teta_X, - sin_teta_X], [0, sin_teta_X, cos_teta_X]])
    R2 = np.dot(np.dot(Rz, Ry), Rx)

    # to find translation of the s.w. points in z axis
    h1new = np.mean(np.dot(np.dot(R2, R1), Xdr[f[f3], :].T).T, axis=0)
    h1new = h1new[2] + h1
    h1 = h1 - h1new
    # 2nd approximation of the s.w. points of Xdr to x-y plane
    high = [0, 0, h1]
    height_vec = np.tile(high, (len(Xdr), 1))
    x2 = np.dot(np.dot(R2, R1), Xdr.T).T + height_vec
    R = np.dot(R2, R1)
    eul = np.array([np.arctan2(R[2, 1], R[2, 2]), - np.arcsin(R[2, 0]), 0])
    # choosing p.c. points within a volume in front of the user
    # f, r, S, nn = cluster_loop(x, x1, x2)
    # **************

    # s1 = np.zeros(len(I))
    # # to find the angles of the cluster with the best fit to a plane
    # S = sum(S.T).T
    # fs = S == min(S)
    # s1 = S[fs]
    # # nx = nn[fs, 0:2]
    # # ny = nn[fs, 2:4]
    # fr = f[r[fs, :]].T
    # # to find the p.c. points of the cluster with the best fit to a plane
    # # x = x2[fr, :]
    # f = abs(x2[fr, 2] - np.mean(x2[fr, 2])) < 0.02
    # fr = fr[f]
    # filter p.c. points, method #1
    x2,fr,r = filter_plane_1(x1,x2)
    # q_size = len(h1_frames) == total_fit_frames
    # if q_size and ((len(fr)/len(r) < 0.75) or (abs(h1-h1_mean) > 0.11)):
    #     h1 = np.mean(h1_frames)
    #     tetax = np.mean(roll_frames)
    #     tetay = np.mean(pitch_frames)
    #     Ry = [[np.cos(tetay), 0, np.sin(tetay)], [0, 1, 0], [-np.sin(tetay), 0, np.cos(tetay)]]
    #     Rx = [[1, 0, 0], [0, np.cos(tetax), -np.sin(tetax)], [0, np.sin(tetax), np.cos(tetax)]]
    #     high = [0, 0, h1]
    #     height_vec = np.tile(high, (len(Xdr), 1))
    #     Ryx = Ry @ Rx
    #     x2 = np.dot(Ryx, Xdr.T).T + height_vec
    #     f = np.argwhere( (abs(x2[:,1])<1.5) * (abs(x2[:,0]) < 4) * (abs(x2[:,2])<0.1))
    #     if len(f) > 400:
    #         f = f[np.random.randint(len(f),400)]
    #     mean_z = np.mean(x2[f,2])
    #     h1 = h1 - mean_z
    #     x2[:,2] = x2[:,2] - mean_z 
    # filter p.c. points, method #1
    #x2,fr = filter_plane_2(x1,x2)
    x = x2
    # n1 represents the plane with the best fit to the cluster
    n1 = remove_mean_of_points(x)
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
    h1 = h1 - np.mean(x3[fr, 2])
    x3[:, 2] = x3[:, 2] - np.mean(x3[fr, 2])
    eul = [np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(R[2, 0])]
    # if (pitch_mean > 0) and (pitch_mean-eul[1] > 5.5*np.pi/180) or (h1 - h1_mean > 0.1):
    #     x3 = x1
    #     h1 = h1_prev
    #     eul = eul0
    # ----
    # ax3.set_data(x3[:, 1], x3[:, 2])
    # ax4.set_data(x3[fr, 1], x3[fr, 2])
    # # --------
    # ax_img.set_data(I[i])
    # fig.canvas.draw()
    # fig.canvas.flush_events()
    # fig.show()
    # Downsampling pcloud
    #pcloud = x3[::2]
    pcloud = x3
    return pcloud,h1,eul,fr

if __name__ == '__main__':
    from scipy.io import loadmat
    os.chdir('../')
    frames = loadmat('rec2.mat')
    frames = frames["Frames"]
    h0 = 0
    for frame in frames:
        if h0 == 0:
            h0,eul0 = plane_init(frame[0],frame[1],frame[3][0][0],frame[3][0][1],1.2)
            h0 = 1.15
        else :
            plane_fit2(frame[0],frame[1],frame[3][0][0],frame[3][0][1],h0)
