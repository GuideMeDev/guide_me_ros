from Modules.Control import Control
from Modules.scan_match import scan_match
from Modules.SLAM import SLAM
from Modules.plane_fit import plane_fit
from Modules.translation_filter import TF_x, TF_y
from Modules.user_feedback import send_feedback
from utils import *
import cProfile, pstats, io
from multiprocessing import Pool    # for scan_match module - running multiproccess to lower running time. (Optional)
import threading
import mat73

def get_feedback(mbypass):
    obst=mbypass[:,:,0] + mbypass[:,:,2]
    scan=mbypass[:,:,1]
    t=(scan == 1)*obst
    obs_overlap = 25
    # if there is significant overlap with an obstacle
    if sum(sum(t)) > obs_overlap:
    # search for overlap between the obstacle and the left and right bypass segments
        s = [sum(sum(scan*obst == 0.2)),sum(sum(scan*obst == 0.1))]
        s= s == min(s)
        s = sum(s*[1,2])
    # if the left segment has smaller overlap witht the obstacle, choose this bypass direction
        if s == 1:
            send_feedback('2'.encode())
            #otherwise, choose the right bypass direction
        else:
            send_feedback('1'.encode())
    # to stop the signal
    else:
            send_feedback('0'.encode())
    

def run_algo(RGB,XYZ,roll,pitch,imu_acc,yaw,pclRGB):
    #
    pRGB1_prev = pclRGB1[0]
    yaw_prev = yaw[0]
    sm_status = 0
    h1_prev = INIT_H1
    tx_prev = np.zeros(2)
    xplus=np.array([]).reshape(0,2)
    xminus=np.array([]).reshape(0,2)
    #
    dummy_img = np.zeros((720,1280,3))
    dummy_glvl = np.zeros((260,250,3))
    dummy_ctrl = np.zeros((380,360,3))

    gs_kw = dict(width_ratios=[5, 5,7,7])
    nav_arrow = []
    fig, (ax1, ax2,ax3,ax4) = plt.subplots(1, 4,figsize=(12,7),gridspec_kw=gs_kw)
    
    ax3.axis(np.array([-120,120,-80,250])/1e3*25)
    ax2_data = ax2.imshow(dummy_glvl)
    ax3_data1 = ax3.plot([],[],'.b',markersize=0.9)[0]
    ax3_data2 = ax3.plot([],[],'.C1',markersize=0.9)[0]
    ax3_data3 = ax3.plot([0,0],[0,6],'r')[0]
    ax3.plot(0/sc,0/sc,'o',linewidth=9,color = 'g',markersize=12)
    ax3.plot(np.array([-50,0,50])/1e3,np.array([210,350,210])/1e3,linewidth=5,color='r')
    ax1_data = ax1.imshow(dummy_img)
    axCtrl_data = ax4.imshow(dummy_ctrl)
    ax1.title.set_text("Sensory Input")
    ax2.title.set_text("Below/Above Ground")
    ax3.title.set_text("Perspective")
    ax4.title.set_text("Control")

    # First run - get first frame
    dxinternal = TF_x(imu_acc,pitch)
    dyinternal = TF_y(imu_acc,pitch)
    pc_prev,h1_prev = plane_fit(RGB[0],np.array(XYZ[0]),roll[0],pitch[0])
    fig.show()
    # Second run - real-time simulated (translation filt missing)
    for i in range(1,len(RGB)):
        rgb_frame = RGB[i]
        pRGB1_curr = pclRGB[i]
        yaw_curr = yaw[i]
        # PLANE FIT
        pc_curr,h1_prev = plane_fit(rgb_frame,XYZ[i],roll[i],pitch[i],h1_prev)
        #dxinternal,dyinternal = translation_filter(imu_acc)
        # SCAN MATCH
        yaw_t,tx_prev,minter_plus,minter_minus = scan_match(pc_prev,pc_curr,pRGB1_prev,pRGB1_curr,yaw_prev,yaw_curr,dxinternal[i]*1e3/sc,dyinternal[i]*1e3/sc,tx_prev,sm_status)
        sm_status = 1
        rgb_frame[:,int(rgb_frame.shape[1] / 2 - 5): int(rgb_frame.shape[1] / 2 + 5),:] = 0
        #Show sensory camera input (with a black line in the center)
        ax1_data.set_data(rgb_frame)
        #building above & bellow ground image frame
        glvl = minter_plus
        glvl = np.dstack((glvl,np.zeros((sizemy,sizemx)),minter_minus))
        #Show frames of above and bellow ground level.
        ax2_data.set_data(glvl)
        # SLAM
        xplus,xminus = SLAM(yaw_t,minter_plus,minter_minus,xplus,xminus)
        tetaz = 90*pi/180 - 1*pi/180
        Rz = [[cos(tetaz),- sin(tetaz)],[sin(tetaz),cos(tetaz)]]
        x1plus=  dot(Rz,xplus.T).T / 1e3 * sc
        x1minus = dot(Rz,xminus.T).T / 1e3 * sc
        ax3_data1.set_data(x1plus[:,0],x1plus[:,1])
        ax3_data2.set_data(x1minus[:,0],x1minus[:,1])
        # CONTROL
        mbypass = Control(xplus,xminus)
        axCtrl_data.set_data(mbypass.astype(float))
        # User feedback module
        # get_feedback(mbypass)
        obst=mbypass[:,:,0] + mbypass[:,:,2]
        scan=mbypass[:,:,1]
        t=(scan == 1)*obst
        obs_overlap = 25
        # if there is significant overlap with an obstacle
        if sum(sum(t)) > obs_overlap:
        # search for overlap between the obstacle and the left and right bypass segments
            s = [sum(sum(scan*obst == 0.2)),sum(sum(scan*obst == 0.1))]
            s= s == min(s)
            s = sum(s*[1,2])
        # if the left segment has smaller overlap witht the obstacle, choose this bypass direction
            if s == 1:
                [s.remove() for s in nav_arrow]
                nav_arrow = []
                xloc= np.array([10,80])
                yloc= np.array([32,30])
                nav_arrow.append(ax4.annotate('Turn Left', xy = (xloc[0] , yloc[0]), xytext = (xloc[1] , yloc[1]),size = 12,weight = 'bold',color = 'red', arrowprops = dict(facecolor ='red',width = 5)))
                #otherwise, choose the right bypass direction
            else:
                [s.remove() for s in nav_arrow]
                nav_arrow = []
                xloc= np.array([360,180])
                yloc= np.array([32,30])
                nav_arrow.append(ax4.annotate('Turn Right', xy = (xloc[0] , yloc[0]), xytext = (xloc[1] , yloc[1]),size = 12,weight = 'bold',color = 'red', arrowprops = dict(facecolor ='red',width = 5)))
        elif nav_arrow:
                [s.remove() for s in nav_arrow]
                nav_arrow = []
        # update next frames
        pc_prev = pc_curr
        pRGB1_prev = pRGB1_curr
        yaw_prev = yaw_curr
        fig.canvas.draw()
        fig.canvas.flush_events()


# Testing with data:
# Loading proccessed data for testing the modules
acc = mat73.loadmat('trans_input.mat')
vi = mat73.loadmat('vi.mat')
imu_acc = acc['acc_raw']
vi = vi['vi']
ros_data = np.load('obj_fin.npy',allow_pickle=True).item()
# translation_data = np.load('obj_dxdy.npy',allow_pickle=True).item()
# dxinternal = translation_data['dxinternal']
# dyinternal = translation_data['dyinternal']
XYZ = ros_data['XYZ']
I = ros_data['I'] # RGB camera frames
roll = ros_data['roll'];pitch = ros_data['pitch'];yaw = ros_data['yaw']
pclRGB1 = ros_data['pRGB1']

if __name__ == "__main__":
    
    pr = cProfile.Profile()
    pr.enable()

    run_algo(I,XYZ,roll,pitch,imu_acc,yaw,pclRGB1)

    pr.disable()
    s = io.StringIO()
    sortby = SortKey.CUMULATIVE
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    print(s.getvalue())
    
