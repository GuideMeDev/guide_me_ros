from Modules.utils import *

def plane_init(I,XYZ,roll,pitch,h0 = INIT_H1):
    # find plane for the first frame
    median_yg = 0
    Xdr = XYZ
    tetax=roll+0*np.pi/180
    tetay=-(pitch+np.pi/2);tetaz=0
    Rz=[[np.cos(tetaz),-np.sin(tetaz),0],[np.sin(tetaz),np.cos(tetaz),0],[0,0,1]]
    Ry=[[cos(tetay),0,sin(tetay)],[0,1,0],[-sin(tetay),0,cos(tetay)]]
    Rx=[[1,0,0],[0,cos(tetax),-sin(tetax)],[0,sin(tetax),cos(tetax)]]
    high=[0,0,h0]
    R1_1= np.dot(np.dot(Rz,Ry), Rx)
    height_vec = np.tile(high, (len(Xdr), 1))
    x = (R1_1 @ Xdr.T).T + height_vec

    c1 = abs(x[:,0]-3) < 1
    c2 = abs(x[:,1]) < 0.6
    c3 = abs(x[:,2]) < 0.2
    idx1 = np.argwhere(c1 * c2 * c3)[:,0]
    mx1 = mean(x[idx1,2])
    h0 = h0-mx1
    x[:,2] = x[:,2] - mx1
    c3 = abs(x[:,2]) < 0.2
    idx2 = np.where(c1 * c2)[0]
    idx3 = np.where(c1 * c2 * c3)[0]
    ratio = len(idx3)/len(idx2)
    
    if ratio>0.8:
        n1 = polyfit(x[idx3,1],x[idx3,2],1)
        roll0=roll-np.arctan(n1[0])
        n2 = polyfit(x[idx3,0],x[idx3,2],1);pitch0=pitch-np.arctan(n2[0])
        eul0=[roll0,-(pitch0+pi/2),0,np.arctan(n1[0]),np.arctan(n2[0])]
        tetax=eul0[0];tetay=eul0[1];tetaz=eul0[2]
        Rz=[[np.cos(tetaz),-np.sin(tetaz),0],[np.sin(tetaz),np.cos(tetaz),0],[0,0,1]]
        Ry=[[cos(tetay),0,sin(tetay)],[0,1,0],[-sin(tetay),0,cos(tetay)]]
        Rx=[[1,0,0],[0,cos(tetax),-sin(tetax)],[0,sin(tetax),cos(tetax)]]
        high=[0,0,h0]
        R1_2 = np.dot(np.dot(Rz,Ry), Rx)
        height_vec = np.tile(high, (len(Xdr), 1))
        x = (R1_2 @ Xdr.T).T + height_vec
        idxs = abs(x[:,2]) < 0.25
        mean0 = np.mean(x[idxs,2])
        x[:,2] = x[:,2] - mean0
        h0 = h0-mean0
        std0 = np.std(x[idx2,2])
        # --------
        if std0<0.07:
            _,median_yg = get_pcl_color(Xdr,idx2,I)
            
    return h0,eul0,median_yg
