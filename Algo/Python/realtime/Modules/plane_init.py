from Modules.utils import *

def plane_init(I,XYZ,roll,pitch,h0 = INIT_H1):
    # fig = plt.figure(figsize=(6, 10))
    # ax1 = fig.add_subplot(2, 1, 1)
    # ax2 = fig.add_subplot(2, 1, 2)
    # ax1.imshow(I)

    # ax2.axis([-2, 2, -0.9, 1.5])
    # ax_img = ax1.imshow(I)
    # ax3 = ax2.plot([], [], '.')[0]
    # ax4 = ax2.plot([], [], '*')[0]
    # find plane for the first frame
    Xdr = XYZ
    tetax=roll+0*np.pi/180;tetay=-(pitch+np.pi/2);tetaz=0
    Rz=[[np.cos(tetaz),-np.sin(tetaz),0],[np.sin(tetaz),np.cos(tetaz),0],[0,0,1]]
    Ry=[[cos(tetay),0,sin(tetay)],[0,1,0],[-sin(tetay),0,cos(tetay)]]
    Rx=[[1,0,0],[0,cos(tetax),-sin(tetax)],[0,sin(tetax),cos(tetax)]]
    high=[0,0,h0]
    R1= np.dot(np.dot(Rz,Ry), Rx)
    height_vec = np.tile(high, (len(Xdr), 1))
    x = (R1 @ Xdr.T).T + height_vec

    c1 = abs(x[:,0]-3) < 1
    c2 = abs(x[:,1]) < 0.6
    c3 = abs(x[:,2]) < 0.2
    idx1 = np.where(c1 * c2 * c3)[0]
    mx1 = np.mean(x[idx1,2])
    h0 = h0-mx1
    x[:,2] = x[:,2] - mx1
    # % plot(x(f,1),x(f,3),'.')
    c3 = abs(x[:,2]) < 0.2
    idx2 = np.where(c1 * c2)[0]
    idx3 = np.where(c1 * c2 * c3)[0]
    ratio = len(idx3)/len(idx2)
    # ax2.plot(x[:, 1], x[:, 2],'.')
    # %subplot(2,1,1);imshow(I{1,i})
    # % subplot(2,1,2);plot3(x1(:,1),x1(:,2),x1(:,3),'.');axis([1,9,-2,2,-0.9,1.5]);view([270,0])
    # % subplot(2,1,2);plot(x1(:,2),x1(:,3),'.');axis([-2,2,-0.9,1.5]);%view([270,0])
    # % plot(x1(:,1),x1(:,3),'.')
    # % pause
    # ax3.set_data(x[:, 1], x[:, 2])
    # ax_img.set_data(I)
    # fig.canvas.draw()
    # fig.canvas.flush_events()
    
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
        R1= np.dot(np.dot(Rz,Ry), Rx)
        height_vec = np.tile(high, (len(Xdr), 1))
        x = (R1 @ Xdr.T).T + height_vec
        std0 = np.std(x[idx2,2])     
        #ax4.set_data(x[fr, 1], x[fr, 2])
        # --------
        if std0<0.07:
            # subplot(2,2,1);imshow(I{1,i})
            # % subplot(2,1,2);plot3(x1(:,1),x1(:,2),x1(:,3),'.');axis([1,9,-2,2,-0.9,1.5]);view([270,0])
            # subplot(2,2,3);plot(x1(:,2),x1(:,3),'.');axis([-2,2,-0.9,1.5]);%view([270,0])
            # subplot(2,2,4);plot(x1(:,1),x1(:,3),'.');axis([2,5,-0.9,1.5]);%view([270,0])
            pass
        
    # plt.show()
    # plt.close()
    return h0,eul0
