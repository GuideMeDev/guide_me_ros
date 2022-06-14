import cv2
from Modules.utils import *
from Modules.plane_init import plane_init
import time
from sklearn.cluster import KMeans
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
def filter_plane_1(x2: np.ndarray):
    # choosing p.c. points within a volume in front of the user
    range_y=np.arange(-1.5,1.5,0.1)
    x2_2 = x2[:,1]
    # bin_center = x2_2[:-1] + np.diff(x2_2)/2
    # h=np.concatenate((bin_center,range_y))
    h = np.histogram(x2_2,range_y)
    h[0][0]=0;h[0][-1]=0
    h0 = h[0]
    frange_y=np.argwhere(h0>300)
    frange_y=[range_y[frange_y[0]],range_y[frange_y[-1]]]
    diff_array_from_X2 = abs(np.diff(x2[:, 2]))
    diff_x_1 = np.diff(x2[:, 1])
    f = np.argwhere((((x2[:,1])>frange_y[0]) * (x2[:,1])<frange_y[1]) * (abs(x2[:, 0]) < 3.5) * (abs(x2[:, 2]) < 0.08) *
                    (np.nan_to_num(np.append(abs(diff_array_from_X2 / diff_x_1), 1) < 0.22)) *
                    (np.nan_to_num(np.append(1,abs(diff_array_from_X2 / diff_x_1)) < 0.22))
                    )[:,0]
    # choosing random 400 p.c. points that we filter in search for the S.W. plane
    array_dimensions: tuple = (250)
    r = np.random.randint(len(f), size=array_dimensions)
    fr = f[r]
    x_plane=x2[fr,:]
    return x_plane,fr,f

def cluster_diff(x,idx):
    x_T = x.T
    x_cm1 = []
    r_12 = []
    plane_dist=[]
    for ind in range(2):
        x_t1 = x[np.where(idx == ind)[0],:]
        x_t1T = x_t1.T
        d = np.sqrt(sum((x_t1-np.tile(np.median(x_t1,axis=0),(x_t1.shape[0],1))).T **2))
        x_t1 = x_t1T[:,d<th2]
        x_cm1.append(np.median(x_t1.T,axis=0).reshape(-1,1))
        x_t1 = x_t1 - np.tile(x_cm1[ind],(1,x_t1.shape[1]))
        xt = x_t1.T
        x11=sum(xt[:,0]**2)
        x22=sum(xt[:,1]**2)
        x33=sum(xt[:,2]**2)
        x12=sum(xt[:,0]*xt[:,1])
        x13=sum(xt[:,0]*xt[:,2])
        x23=sum(xt[:,1]*xt[:,2])
        D=x11*x22 - x12**2
        a=x23*x12 - x13*x22
        b=x13*x12 - x11*x23
        n1=[a,b,D]
        n1=n1/np.linalg.norm(n1)
        tetax=np.arctan2(n1[1],np.sqrt(n1[2]**2+n1[0]**2))
        tetay=-np.arctan2(n1[0],np.sqrt(n1[2]**2+n1[1]**2))
        ang_roll=tetax;ang_pitch=tetay
        Ry=[[cos(tetay),0,sin(tetay)],[0,1,0],[-sin(tetay),0,cos(tetay)]]
        Rx=[[1,0,0],[0,cos(tetax),-sin(tetax)],[0,sin(tetax),cos(tetax)]]
        Rxy = np.dot(Rx,Ry)
        x_t1R = Rxy @ x_t1
        r_12.append(Rxy)
        plane_dist.append(std(x_t1R[2,:]))

    ind=np.argmin(plane_dist)
    x_t1R=r_12[ind]@(x_T-np.tile(x_cm1[ind],(1,x_T.shape[1])))
    x_nR=r_12[ind]@x_T
    x_t=x_t1R
    h2=0
    perc2=0
    f1=np.argwhere((abs(x_t[2,:])<th1) & (abs(x_t[0,:])<4*std(x_t1[0,:])))[:,0]
    perc1=len(f1)/len(x)
    f2=np.argwhere(abs(x_t[2,:])>1.5*th1)[:,0]
    d=x_t[2,f2]-np.median(x_t[2,f2])
    f2=f2[abs(d)<1.5*th1]
    if not len(f2):
        f2=f1[0:2]
    perc2 = len(f2) / len(x)
    z1 = np.r_[np.median(x_nR[2,f1].T) , np.median(x_t[2,f1].T)]
    z2 = np.r_[np.median(x_nR[2,f2].T) , np.median(x_t[2,f2].T)]

    return f1,f2,perc1,perc2,z1,z2

def filter_plane_pc(x):
    """
    In this function we first substract the center of mass from the points 
    and then rotate the points around x and y axis in order to fit the S.W. with X-Y plane
    and keep only points with distance in z-axes that is <th1
    th1 is the threshold for points distance from z_axis (ideal distance is 0)
    """
    f1=[1,2]
    f2=[1,2]
    perc1=0;perc2=0
    x1 = x-np.tile(np.median(x),(x.shape[0],1))
    x1 = x1/ np.tile(np.max(abs(x1)),(x1.shape[0],1))
    a = np.diff(x1,axis=0)
    a = a/np.tile(np.linalg.norm(a),(1,3))
    xa = np.c_[x1[0:-1,:],np.arccos(a[:,2]),np.arctan2(a[:,1],a[:,0])]
    kmeans = KMeans(n_clusters=2).fit(xa)
    idx = kmeans.labels_
    # x1=x1[np.where(idx==0)[0],:]
    # x1a = x1[0,:]
    # x_cm1 = np.median(x1,axis=0)
    # x_std1 = np.std(x1,axis=0)
    # d1 = np.sqrt(sum((x1-np.tile(x_cm1,(x1.shape[0],1))).T**2))
    # x1 = x1[d1<3*std(d1),:]
    # x1[0,:] = x1a
    # x2 = x[np.where(idx==1)[0],:]
    # x2a = x2[0,:]
    # x_cm2 = np.median(x2,axis=0)
    # x_std2 = np.std(x2,axis=0)
    # d2 = np.sqrt(sum((x2-np.tile(x_cm2,(x2.shape[0],1))).T**2))
    # x2 = x2[d2 < 3*std(d2),:]
    # x2[0,:] = x2a
    f1,f2,perc1,perc2,z1,z2 = cluster_diff(x,idx)
    return f1,f2,perc1,perc2,z1,z2

def plane_fit2(I, XYZ, roll, pitch,sc_acc,prev_group,h1_que,h1_prev = INIT_H1):
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
    sc2=0.5
    no_plane=0
    #h1 = h1_prev - 2*sc_acc
    h1 = h1_prev
    Xdr = XYZ
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
    R21 = np.dot(R2, R1)
    eul = np.array([np.arctan2(R21[2, 1], R21[2, 2]), - np.arcsin(R21[2, 0]), 0])
    # filter p.c. points, method #1
    x,fr,f = filter_plane_1(x2)
    f_x2 = f
    # Add color of points
    yg, median_gray = get_pcl_color(Xdr,fr,I)
    # filter p.c. points, method #1
    #x2,fr = filter_plane_2(x1,x2)
    #x = x2
    th_f2 = 30
    prev_group[0] = h1_prev/sc1[0]
    f1,f2,perc1,perc2,z1,z2 = filter_plane_pc(x)
    x_group = x1
    fr1 = fr[f1]
    mean_group1 = np.array([h1-z1[0] , np.median(yg[f1[yg[f1]>0]]),0])
    mean_group1[0:-1] = mean_group1[0:-1]/sc1
    mean_group2=[0,0,0]
    len_sc1 = len(sc1)
    sc_flag = z2[0]>z1[0]
    t2 = sc_flag*sc2 + ~sc_flag*-sc2
    t1 = ~sc_flag*sc2 + sc_flag*-sc2
    xf2 = x2[fr[f2],:]
    if (len(f2)>th_f2) and (abs(z2[1]-z1[1])>0.05):
        mean_group2[0:-1]=[h1-z2[0],np.median(yg[f2[yg[f2]>0]])]
        
    elif len(f2)>th_f2:
        f1a,f2a,perc1a,perc2a,z1,z2=filter_plane_pc(xf2)
        f2 = f2[f1a]
        mean_group2[0:-1]=[h1-z1[0],np.median(yg[f2[yg[f2]>0]])]

    mean_group2[0:-1]=mean_group2[0:-1]/sc1
    mean_group1[len_sc1]=t1
    mean_group2[len_sc1]=t2
    temp1=np.stack((mean_group1, mean_group2))
    temp2=np.stack((prev_group,prev_group))
    if temp2[0][2]==0 or temp1[0][2]==0:
        temp2[:,2]=0
        temp1[:,2]=0
        prev_group[2]=0

    fr2 = fr[f2]
    temp3=np.sqrt(sum((temp1-temp2).T**2))

    #plot3(xf2[:,0],xf2[:,1],xf2[:,2],'s')
    #axis([1,4,-1.5,1.5,-0.2,0.2]),view(0,0),
    th_plane=2.6
    #print(mean_group1,mean_group2,prev_group,temp3,0)
    # TODO: Subject to change - unnecesary if's
    if (temp3[0]<th_plane) and (temp3[0]<temp3[1]):
        prev_group=mean_group1
    elif len(f2)>10 and (temp3[1]< (1.5*th_plane)) and (temp3[1]<temp3[0]):
        prev_group=mean_group2
        f1=f2
    elif (temp3[0]<(1.5*th_plane)) and (temp3[1]<th_plane*2):
        prev_group=mean_group1
        print(123)
    # no group was found and the no_plane flag is up 
    elif (temp3[0]>th_plane) and (temp3[1]>th_plane):
        print(1234)
        no_plane=1
    #print(mean_group1,mean_group2,prev_group,temp3,0)

        # if z2[0]>z1[0]:
        #     t2=sc2;t1=-sc2
    # hold on;x1=x2(fr(f1),:);plot3(x1(:,1),x1(:,2),x1(:,3),'or');
    # n1 represents the plane with the best fit to the cluster
    n1 = remove_mean_of_points(x)
    n1 = n1 / np.linalg.norm(n1)
    # to find the rotation angles required to rotate the n1 plane to become parallel to x-y plane
    tetax = np.arctan2(n1[1], np.sqrt(n1[2] ** 2 + n1[0] ** 2))
    tetay = -np.arctan2(n1[0], np.sqrt(n1[2] ** 2 + n1[1] ** 2))
    tetaz = 0
    #Rz = [[np.cos(tetaz), -np.sin(tetaz), 0], [np.sin(tetaz), np.cos(tetaz), 0], [0, 0, 1]]
    Ry = [[np.cos(tetay), 0, np.sin(tetay)], [0, 1, 0], [-np.sin(tetay), 0, np.cos(tetay)]]
    Rx = [[1, 0, 0], [0, np.cos(tetax), -np.sin(tetax)], [0, np.sin(tetax), np.cos(tetax)]]
    # 3rd rotation and translation of Xdr
    R3 = np.dot(Ry, Rx)
    R = np.dot(R3, R21)
    x3 = (np.dot(R, Xdr.T)).T + height_vec
    h1 = h1 - np.mean(x3[fr, 2])
    x3[:, 2] = x3[:, 2] - np.mean(x3[fr, 2])
    eul = [np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(R[2, 0])]
    # in case no plane found, find height estimation 

    if no_plane:
        print("no_plane")
        h1 = 0.5*np.median(h1_que) + 0.5*h1_prev
        high=[0,0,h1]
        height_vec = np.tile(high, (len(Xdr), 1))
        x3 = np.dot(R,Xdr.T).T + height_vec
        m_x3 = np.nan_to_num(mean(x3[f_x2[abs(x3[f_x2,2])<0.03],2]))
        x3[:,2] = x3[:,2] - m_x3
        h1=h1-m_x3
    f=np.argwhere((abs(x3[:,1])<1.5) * (abs(x3[:,0])<4) * (abs(x3[:,2])<0.1))
    if len(f):
        f=f[np.random.randint(len(f),size = 250)]
    # plt.plot(x3[:,1],x3[:,2],'.')
    # plt.plot(x3[f,1],x3[f,2],'o')
    #TODO: Continue from here, change #10 in matlab

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
    return pcloud,h1,eul,fr,f,prev_group,fr1,fr2,x_group

if __name__ == '__main__':
    from scipy.io import loadmat
    os.chdir('../')
    frames = loadmat('rec2.mat')
    frames = frames["Frames"]
    h0 = 0
    for frame in frames:
        if h0 == 0:
            h0,eul0,md_yg = plane_init(frame[0],frame[1],frame[3][0][0],frame[3][0][1],1.2)
            h0 = 1.15
        else :
            plane_fit2(frame[0],frame[1],frame[3][0][0],frame[3][0][1],h0)
