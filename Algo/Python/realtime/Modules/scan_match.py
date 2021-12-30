from Modules.utils import *

def scan_match(pcloud_prev,pcloud_curr,pRGB1_prev,pRGB1_curr,yaw_prev,yaw_curr,dxIMU_i,dyIMU_i,tx_prev,status=0):

    b2 = pcloud_curr*1000 / sc
    trgb2 = -weg_tex * pRGB1_curr
    b1 = pcloud_prev* 1000 / sc
    trgb1 = -weg_tex * pRGB1_prev
    #find d-frame and t-frame for sample (i+1) and same for frame (i) but after yaw rotation
    yaw1 = yaw_prev - yaw_curr
    tetaz0 = yaw1
    # TODO: save b2 for later use instead of a recomputation - meaning saving mpc2,tmpc2
    mpc1,mpc2,tmpc1,tmpc2,b1a,b1b = find_dframe_tframe(b1,b2,trgb1,trgb2,dxmin,sizemx,sizemy,thz0,weg_obst,yaw1)
        #find translation of frame (i) to match frame (i+1)
    m2=mpc2[:,rangex_mpc2] + tmpc2[:,rangex_mpc2]
    m1=mpc1[rangey_mpc1][:,rangex_mpc1] + tmpc1[rangey_mpc1][:,rangex_mpc1]

    tx_curr = xcross2_custom(m1,m2,dyIMU_i,dxIMU_i,kkx,kky)
    tx_prev = tx_curr * (~status+2) + tx_prev * status
    # changeable
    tx_curr = 0.8*tx_curr + 0.2*tx_prev
    #find no floor
    #find segments below ground level (mpc2curbe) for frame (i+1)
    mpc2curbe,mpc2nofloor,mpc2floor=choose_mean_range2(b2,thz0_mean,dxmin*1.5,dxmax,sizemx,sizemy)
    #find a second rotation angle (tetaz1) to match frames (i) and (i+1)
    rtb1a = np.copy(b1a)
    rtb1a[:,0]=rtb1a[:,0] - tx_curr[1]
    rtb1a[:,1]=rtb1a[:,1] - tx_curr[0]
    b1b[:,0:2]=rtb1a[:,0:2]
    rtmpc1,tetaz1,rtb1b=correct_reg_angle2(rtb1a,b1b,mpc2,yaw_reg,sizemx,sizemy)
    # find segments below ground level (mpc1curbe) for frame (i) after rotation (things like the road, holes, etc)
    #and translation (rt)
    mpc1curbe,mpc1nofloor,mpc1floor=choose_mean_range2(rtb1b,thz0_mean,dxmin*1.5,dxmax,sizemx,sizemy)
    #update tetaz and save translation and rotation values to the matrix yawt
    tetaz = tetaz0 + tetaz1 + 0
    yaw_t = [tetaz,tx_curr[0],tx_curr[1],tetaz0]
    #find dframes (mpc2_plus and mpc1_plus) above ground level for frames (i) and (i+1) (dyanmic / static objects)
    x = round_int(b2)
    f = np.where( (x[:,2] > thz0*2) * (x[:,0] > dxmin) * (x[:,0] < dxmax) * (abs(x[:,1]) < sizemy / 2))[0]
    ba= x[f,:]
    px2a=ba[:,0]
    py2a=ba[:,1] - half_sizey
    py2a[py2a <= -sizemy] += sizemy
    mpc_zero= np.zeros((sizemy,sizemx))
    mpc = np.copy(mpc_zero)
    mpc[py2a,px2a] = x[f,2]
    c = sig.convolve2d(mpc,np.ones((3,3)) / 9,mode = 'same')
    x = rtb1b
    f=np.where((x[:,2] > thz0*2) * (x[:,0] > dxmin) * (x[:,0] < dxmax) * (abs(x[:,1]) < sizemy / 2))[0]
    ba = round_int(x[f,:])
    px2a = ba[:,0]
    py2a = ba[:,1] - half_sizey
    py2a[py2a <= -sizemy] += sizemy
    mpc1 = np.copy(mpc_zero)
    mpc1[py2a, px2a ] = x[f,2]
    c = sig.convolve2d(mpc1,np.ones((3,3)) / 9,mode = 'same')
    mpc1_plus=(c > 2)
    #show dframes(pcloud) above and below ground level
    minter_plus = round_int((mpc2*mpc1_plus) > 0)
    minter_minus = round_int(mpc1curbe* mpc2curbe)
    return yaw_t,tx_curr,minter_plus,minter_minus