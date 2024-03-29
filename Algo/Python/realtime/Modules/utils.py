
from collections import deque
import os
import cv2
# for better runtime
os.environ['NUMPY_EXPERIMENTAL_ARRAY_FUNCTION'] = '0'
from matplotlib import pyplot as plt
import numpy as np
from numpy import cos,sin,dot,array,copy,pi,tile,diff,percentile,polyfit,arctan,dot,mean,arcsin,arctan2,std,matmul
from numpy.core.fromnumeric import size
from scipy import signal as sig
from pstats import SortKey
import scipy.io as io
import collections

sc = 20
weg_obst=5
weg_tex=3
thz0 = 60 / sc
thz0_mean = 40 / sc
th1=0.03
th2=0.7
dxmin = 1500 / sc
dxmax = 4500 / sc
sizemx = int(5000 / sc) 
sizemy = int(5200 * 92/100 / sc) + 1
kkx = kky = 6
rangex_mpc2=np.arange(dxmin ,dxmax).astype(int)
rangex_mpc1=np.arange(dxmin + 400 / sc ,dxmax - 400 / sc).astype(int)
rangey_mpc1=np.arange(600 / sc ,sizemy - 600 / sc).astype(int)
dlen = 3500 // sc
dwi = 316 // sc
k3=5000 / sc
yaw_reg = np.arange(-1.2,1.201,0.4) * pi/180
half_sizey = sizemy // 2
h1_mean = 1.16
total_fit_frames = 40
P=[[1/0.002,0,0],[0,1/0.002,0],[0,0,1]]
sc1=np.array([0.028,10])*2
h1_frames = deque(maxlen=total_fit_frames)
roll_frames = deque(maxlen=total_fit_frames)
pitch_frames = deque(maxlen=total_fit_frames)
INIT_H1 = 1.2
FPS_IMU = 200
FPS = 6
h_qe = collections.deque(maxlen=10)

def round_int(x):
    return np.intc(x)

# Functions for Scan match module
def choose_mean_range2(b=None,thz0=None,dxmin=None,dxmax=None,sizemx=None,sizemy=None):
    # with cProfile.Profile() as pr:
    #use only values with x>0
    b1=b[b[:,0] > 0,:]
    #sort the P.C. vector based on ascending x-axis values and name the new variable x
    idxs1=np.argsort(b1[:,0])
    x=b1[idxs1,:]
    #sort the x vector based on ascending y-axis values and name the new variable x
    idxs2=np.argsort(x[:,1])
    x=x[idxs2,:]
    #smooth z-axis values of vector x
    win_size = 20
    conv = np.ones(win_size)/win_size
    x[:,2] = np.convolve(x[:,2],conv,mode='same')
    #choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles above the ground)
    f = (x[:,0] < dxmax) * (x[:,2] < -thz0) * (x[:,0] > dxmin) * (abs(x[:,1]) < sizemy / 2)
    #here we define the pixels for the x-y-axes
    ba = round_int(x[f,:])
    px2a= ba[:,0]
    py2a= ba[:,1] - half_sizey
    py2a[py2a <= -sizemy] += sizemy

    #here we create the image mpc2
    mpc2=np.zeros((sizemy,sizemx))
    mpc2[py2a - 1, px2a] = 1
    
    #choosing specific area within x (next we transform coordinates to pixels to create an image of obstacles above and below the ground) 
    f = (x[:,2] - thz0*2 > 0) * (abs(x[:,1]) < sizemy / 2) * (x[:,0] < dxmax) * (x[:,0] > dxmin)
    ba = round_int(x[f,:])
    px2a = ba[:,0]
    py2a = ba[:,1] - half_sizey
    py2a[py2a <= -sizemy] += sizemy
    mpc2nofloor=np.zeros((sizemy,sizemx))
    mpc2nofloor[py2a-1, px2a] = x[f,2]
    
    #choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles below the ground)
    f= (abs(x[:,2]) < thz0) * (abs(x[:,1]) < sizemy / 2) * (x[:,0] < dxmax) * (x[:,0] > dxmin)
    ba = round_int(x[f,:])
    px2a= ba[:,0]
    py2a= ba[:,1] - half_sizey
    py2a[py2a <= -sizemy] += sizemy
    mpc2floor=np.zeros((sizemy,sizemx))
    mpc2floor[py2a - 1, px2a ]=1
    # ps = pstats.Stats(pr).sort_stats(SortKey.CUMULATIVE)
    # ps.print_stats()
    return mpc2, mpc2nofloor, mpc2floor

def correct_reg_angle2(b1a=None,rtb1=None,mpc2=None,yaw1=None,sizemx=None,sizemy=None):
#this function is similar to the function correct_reg_angle2,
#except here we rotate the points of b1 by a range of angles 
#in search for best fit between mpc1 and mpc2
    px=b1a[:,0]
    py=b1a[:,1]
    pz=b1a[:,2]
    pxb=rtb1[:,0]
    pyb=rtb1[:,1]
    pzb=rtb1[:,2]
    s=[]

    length_of_yaw1 = len(yaw1)

    for j in range(length_of_yaw1):
        tetaz = yaw1[j]
        cos_teta_Z = np.cos(tetaz)
        sin_teta_Z = np.sin(tetaz)
        Rz = [[cos_teta_Z, -sin_teta_Z],[sin_teta_Z,cos_teta_Z]]
        t1 = round_int(dot(Rz,[px.T,py.T]).T)
        px1=t1[:,0]
        py1=t1[:,1] - half_sizey
        py1[py1 <= -(sizemy-1)] += sizemy
        px1[px1 >= (sizemx-1)] -= sizemx
        mpc1=np.zeros((sizemy,sizemx))
        mpc1[py1-1,px1] = pz
        s.append(np.sum(np.sum(mpc2 * mpc1)))
    
    f = np.argwhere(s == max(s))[0]
    tetaz = yaw1[f[0]]
    cos_teta_Z = np.cos(tetaz)
    sin_teta_Z = np.sin(tetaz)
    Rz = [[cos_teta_Z, -sin_teta_Z],[sin_teta_Z,cos_teta_Z]]
    t1=(dot(Rz,[pxb.T,pyb.T])).T
    px1=t1[:,0]
    py1=t1[:,1]
    b1b = np.array([px1.T,py1.T,pzb.T]).T
    t1 = round_int(dot(Rz,[px.T,py.T]).T)
    px1=t1[:,0]
    py1=t1[:,1] - half_sizey
    py1[py1 <= -(sizemy-1)] += sizemy
    px1[px1 >= (sizemx-1)] -= sizemx
    mpc1=np.zeros((sizemy,sizemx))
    mpc1[py1- 1,px1 ]=pz
    mpc1[py1, px1]=pz
    mpc1[py1- 2,px1]=pz
    mpc1[py1 - 1,px1 + 1]=pz
    mpc1[py1 - 1,px1 - 1]=pz
    return mpc1,tetaz,b1b

def xcross2_custom(m1=None,m2=None,dyIMU=None,dxIMU=None,kkx=None,kky=None):
# TODO: Add a correction for dx,dy when its beyond bounds
    try:
        s = np.zeros((kkx*2+1,kky*2+1))
        kx=np.arange(-kkx,kkx+1)
        ky=np.arange(-kky,kky+1)
        for j1 in range(len(ky)):
            ystart = int((m2.shape[0] / 2 - m1.shape[0] / 2 + ky[j1] - dyIMU))
            yend = int((m2.shape[0] / 2 + m1.shape[0] / 2 + ky[j1] - dyIMU)) 
            for j2 in range(len(kx)):
                xstart = int(m2.shape[1] / 2 - m1.shape[1] / 2 + kx[j2] - dxIMU)
                xend = int(m2.shape[1] / 2 + m1.shape[1] / 2 + kx[j2] - dxIMU) 
                m2a = m2[ystart : yend, xstart : xend]
                s[j1][j2]=np.sum(np.sum(m1*m2a))

        # Look for the idle threshold for max intensity of cross correlation - 0.85 - 0.99
        max_thresh = 0.78       
        idxs = np.argwhere(s > np.max(np.max(s))*max_thresh)
        fx,fy = idxs[:,1],idxs[:,0]
        tx1 = np.array([dyIMU - ky[round_int(np.mean(fy))] ,dxIMU - kx[round_int(np.mean(fx))]])
    except:
        pass
    return tx1

def get_dtmpc2(b2,trgb2):
    f = np.where((b2[:,0] > dxmin + 1) * (b2[:,0] < sizemx - 10) * (abs(b2[:,1]) < sizemy - 10))[0]
    b2 = round_int(b2[f,:])
    px2 = b2[:, 0]
    py2 = b2[:, 1] - half_sizey
    py2[py2 <= -sizemy + 1] += sizemy
    pz2 = np.copy(b2[:, 2])
    pz2[pz2 < thz0] = -1
    pz2[pz2 > thz0] = weg_obst
    pz2[abs(b2[:, 2]) < thz0] = 0

    dmpc2 = np.zeros((sizemy, sizemx))
    dmpc2[py2-1, px2] = pz2
    dmpc2[py2, px2] = pz2
    dmpc2[py2-2, px2] = pz2
    dmpc2[py2-1, px2+1] = pz2
    dmpc2[py2-1, px2-1] = pz2
    #find t-frame
    f = np.where((b2[:,0] > dxmin) * (b2[:,0] < sizemx / 4*3 - 10) * (abs(b2[:,1]) < sizemy - 10) * (abs(b2[:,2]) < thz0))[0]
    tb2 = round_int(b2[f,:])
    px2 = tb2[:,0]
    py2 = tb2[:,1] - half_sizey
    py2[py2 <= -sizemy] += sizemy
    pz2=trgb2[f,1]
    tmpc2=np.zeros((sizemy,sizemx))
    tmpc2[py2-1, px2]=pz2
    return dmpc2,tmpc2

def find_dframe_tframe(b1,b2,trgb1=None,trgb2=None,dxmin=None,sizemx=None,sizemy=None,thz0=None,weg_obst=None,yaw1=None):
    # get next depth and texture frames
    f = np.where((b2[:,0] > dxmin + 1) * (b2[:,0] < sizemx - 10) * (abs(b2[:,1]) < sizemy - 10))[0]
    b2 = round_int(b2[f,:])
    px2 = b2[:, 0]
    py2 = b2[:, 1] - half_sizey
    py2[py2 <= -sizemy + 1] += sizemy
    pz2 = np.copy(b2[:, 2])
    pz2[pz2 < thz0] = -1
    pz2[pz2 > thz0] = weg_obst
    pz2[abs(b2[:, 2]) < thz0] = 0
    dmpc2 = np.zeros((sizemy, sizemx))
    dmpc2[py2-1, px2] = pz2
    dmpc2[py2, px2] = pz2
    dmpc2[py2-2, px2] = pz2
    dmpc2[py2-1, px2+1] = pz2
    dmpc2[py2-1, px2-1] = pz2
    #find t-frame
    f = np.where((b2[:,0] > dxmin) * (b2[:,0] < sizemx / 4*3 - 10) * (abs(b2[:,1]) < sizemy - 10) * (abs(b2[:,2]) < thz0))[0]
    tb2 = round_int(b2[f,:])
    px2 = tb2[:,0]
    py2 = tb2[:,1] - half_sizey
    py2[py2 <= -sizemy] += sizemy
    pz2=trgb2[f,1]
    tmpc2=np.zeros((sizemy,sizemx))
    tmpc2[py2-1, px2]=pz2    
    #dmpc2,tmpc2 = get_dtmpc2(b2,trgb2)
    #find current d-frame
    f = np.where((b1[:,0] > dxmin) * (b1[:,0] < sizemx - 10) * (abs(b1[:,1]) < sizemy - 10))[0]
    b1 = b1[f,:]
    px1 = np.copy(b1[:,0])
    py1 = np.copy(b1[:,1])
    pz1 = np.copy(b1[:,2])
    pz1[pz1 < thz0] = -1
    pz1[pz1 > thz0] = weg_obst
    pz1[ np.abs(b1[:,2]) < thz0 ]=0
    tetaz=yaw1
    cos_teta_Z = np.cos(tetaz)
    sin_teta_Z = np.sin(tetaz)
    Rz = [[cos_teta_Z, -sin_teta_Z],[sin_teta_Z,cos_teta_Z]]
    t1 = round_int(dot(Rz,[px1.T,py1.T]).T)
    b1a=np.copy(b1)
    b1a[:,0:2] = t1
    b1a[:,2] = pz1
    px1 = t1[:,0]
    py1 = t1[:,1] - half_sizey
    px1[px1 >= (sizemx-1)] -= sizemx
    py1[py1 <= -sizemy] += sizemy
    b1b = np.copy(b1a)
    b1b[:,2] = b1[:,2]
    dmpc1=np.zeros((sizemy,sizemx))
    dmpc1[py1-1,px1]=pz1
    
    #find current t-frame
    f=np.where((b1[:,0] > dxmin) * (b1[:,0] < sizemx / 4*3 - 10)  * (abs(b1[:,1]) < sizemy - 10) * (abs(b1[:,2]) < thz0))[0]
    tb1_f = np.copy(b1[f,:])
    px1=tb1_f[:,0]
    py1=tb1_f[:,1]
    pz1=trgb1[f,1]
    t1 = round_int(dot(Rz,[px1.T,py1.T]).T)
    px1 = t1[:,0]
    py1 = t1[:,1] - half_sizey
    py1[py1 <= -sizemy] += sizemy
    tmpc1=np.zeros((sizemy,sizemx))
    tmpc1[py1-1, px1]=pz1

    return dmpc1,dmpc2,tmpc1,tmpc2,b1a,b1b

def get_pcl_color(XDR,fr,Img):
    x1a = [-XDR[fr,1].T,-XDR[fr,2].T,XDR[fr,0].T]
    pxy=np.dot(P,x1a)
    pxy[0,:] = pxy[0,:]+640
    pxy[1,:] = pxy[1,:]+360
    pxy = round_int(pxy)
    f_color = np.argwhere(pxy[0,:]>1280 | (pxy[1,:]>720) | (pxy[0,:] < 1))
    pxy[:,f_color]=1
    Ig=cv2.cvtColor(Img,cv2.COLOR_RGB2GRAY)
    yi = pxy[1,:]
    xi = pxy[0,:]-1
    yi[yi > 719] -= 720 
    xi[xi > 1279] -= 1280 
    yg=Ig[yi,xi]
    yg[f_color]=0
    median_gray = np.median(yg)
    return yg ,median_gray

# Performance Metric for Plane fit module
def plane_fit_metric(x2):
    fin=np.argwhere(abs(x2[:,1])<0.5 & abs(x2[:,0]-3)<0.5 & abs(x2[:,2])<0.08)[0]
    fout=np.argwhere(abs(x2[:,1])>0.5 & abs(x2[:,1])<1.5 & abs(x2[:,0]-3)>0.5 & abs(x2[:,2])<0.08)[0]
    s=std(x2[fin,2])/std(x2[fout,3])
    return s