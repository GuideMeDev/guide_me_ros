from Modules.plane_init import *
from Modules.user_feedback import get_feedback, send_feedback
from rospy_sub_ver2 import *
from Modules.utils import *
from Modules.translation_filter import *
from Modules.plane_fit import *
from Modules.scan_match import *
from Modules.SLAM import *
from Modules.Control import *
import cProfile, pstats, io
import traceback

FRAMES_NUM = 60
FRAMES_COUNT = 0

def checkData(pqueue):
    while True:
        send_feedback("0".encode())
        time.sleep(0.16)
        send_feedback("0".encode())
        time.sleep(0.16)
        send_feedback("1".encode())
        time.sleep(0.16)
        send_feedback("1".encode())
        time.sleep(0.16)

def run_algo(pqueue):
    prev_group = [0,0,0]
    sm_status = 0
    vi_prev = None
    dv_prev = None
    h1_prev = INIT_H1
    feedback_prev = b'0'
    eul = []
    xyz_prev = []
    tx_prev = np.zeros(2)
    xplus=np.array([]).reshape(0,2)
    xminus=np.array([]).reshape(0,2)
    # Plane init : detect floor plane
    while not len(eul):
        data_list = pqueue.get()
        rgb_img,xyz,acc_raw,euler,pRGB1_prev = data_list[0],data_list[1],data_list[2],data_list[3],data_list[4]
        pitch_fit = pitch_prev = euler[1]
        yaw_prev = euler[2]
        roll_fit = roll = euler[0]
        try:
            h1_prev,eul,median_yg = plane_init(rgb_img,xyz,roll,pitch_prev)
            roll_fit = eul[0]; pitch_fit = eul[1]
            prev_group[1] = median_yg/sc1[1]
        except:
            print("PLANE INIT FAILED")
    
    send_feedback(b'1')
    time.sleep(0.25)
    send_feedback(b'0')
    # First Frame Plane Fit
    while not len(xyz_prev):
        data_list = pqueue.get()
        rgb_img,xyz,acc_raw,euler,pRGB1_prev = data_list[0],data_list[1],data_list[2],data_list[3],data_list[4]
        try:
            xyz_prev,h1_prev,eul,fr,prev_group = plane_fit(rgb_img,xyz,roll_fit,pitch_fit,prev_group,h_qe,h1_prev)
            roll_fit = eul[0]; pitch_fit = eul[1]
        except:
            print("PLANE FIT FAILED")
            
    # Start Algo flow
    msg_time = time.time()
    while time.time() - msg_time < 4:
        while not pqueue.empty():
            try:
            # Getting recent sensors data from RT_writer() function in another process (with the Queue)
                #qt = time.time()
                # No wait for better runtime
                data_list = pqueue.get_nowait()
                rgb_img,xyz,acc_raw,euler,pRGB1_curr = data_list[0],data_list[1],data_list[2],data_list[3],data_list[4]
                pitch_curr = euler[1]
                roll = euler[0]
                yaw_curr = euler[2]
                # Initializing waiting time for queue data
                # Translation Filter
                dxinternal,vi_prev = TF_x(acc_raw,pitch_curr,pitch_prev,vi_prev)
                dyinternal,dv_prev = TF_y(acc_raw,pitch_curr,pitch_prev,dv_prev)
                # Plane Fit
                xyz_curr,h1_prev,eul,fr,prev_group = plane_fit(rgb_img,xyz,roll_fit,pitch_fit,prev_group,h_qe,h1_prev)
                roll_fit = eul[0]; pitch_fit = eul[1]
                # Scan Match
                yaw_t,tx_prev,minter_plus,minter_minus = scan_match(xyz_prev,xyz_curr,pRGB1_prev,pRGB1_curr,yaw_prev,yaw_curr,dxinternal*1e3/sc,dyinternal*1e3/sc,tx_prev,sm_status)
                # Slam
                xplus,xminus = SLAM(yaw_t,minter_plus,minter_minus,xplus,xminus)
                # Control
                mbypass = Control(xplus,xminus)
                feedback_prev = get_feedback(mbypass,feedback_prev)
                xyz_prev = xyz_curr
                pRGB1_prev = pRGB1_curr
                yaw_prev = yaw_curr
                pitch_prev = pitch_curr

                msg_time = time.time()
            except Exception:
                eul = []
                print(traceback.format_exc())
    send_feedback(b'0')
# +++++++++++++++++++++++++++++++++++++++++++++++

def run_algo_graph(pqueue,gqueue):
    sm_status = 0
    vi_prev = None
    dv_prev = None
    h1_prev = INIT_H1
    feedback_prev = b'0'
    eul = []
    xyz_prev = []
    prev_group = [0,0,0]
    tx_prev = np.zeros(2)
    xplus=np.array([]).reshape(0,2)
    xminus=np.array([]).reshape(0,2)
    # Starting graphics process
    graphs_p = Process(target=run_graphs, args=((gqueue),))
    graphs_p.start()
    #
    # Plane Init
    while not len(eul):
        data_list = pqueue.get()
        rgb_img,xyz,acc_raw,euler,pRGB1_prev = data_list[0],data_list[1],data_list[2],data_list[3],data_list[4]
        pitch_fit = pitch_prev = euler[1]
        yaw_prev = euler[2]
        roll_fit = roll = euler[0]
        try:
            h1_prev,eul,median_yg = plane_init(rgb_img,xyz,roll,pitch_prev)
            roll_fit = eul[0]; pitch_fit = eul[1]
            prev_group[1] = median_yg/sc1[1]
        except:
            print("PLANE INIT FAILED")
    
    send_feedback(b'1')
    time.sleep(1)
    send_feedback(b'0')
    # First Frame Plane Fit
    while not len(xyz_prev):
        data_list = pqueue.get()
        rgb_img,xyz,acc_raw,euler,pRGB1_prev = data_list[0],data_list[1],data_list[2],data_list[3],data_list[4]
        roll_prev = euler[0]; pitch_prev = euler[1]
        try:
            xyz_prev,h1_prev,eul,fr,prev_group = plane_fit(rgb_img,xyz,roll_fit,pitch_fit,prev_group,h_qe,h1_prev)
            roll_fit = eul[0]; pitch_fit = eul[1]
        except:
            print("PLANE FIT FAILED")
            
    # Start Algo flow
    msg_time = time.time()
    while time.time() - msg_time < 10:
        while not pqueue.empty():
            try:
            # Getting recent sensors data from RT_writer() function in another process (with the Queue)
                # No wait for better runtime
                data_list = pqueue.get_nowait()
                rgb_img,xyz,acc_raw,euler,pRGB1_curr = data_list[0],data_list[1],data_list[2],data_list[3],data_list[4]
                pitch_curr = euler[1]
                roll = euler[0]
                yaw_curr = euler[2]
                # Translation Filter
                dxinternal,vi_prev = TF_x(acc_raw,pitch_curr,pitch_prev,vi_prev)
                dyinternal,dv_prev = TF_y(acc_raw,pitch_curr,pitch_prev,dv_prev)
                # Plane Fit
                h1_prev = h1_prev#+(np.mean(acc_raw[-33::,2]) - np.mean(acc_raw[-66:-33,2]))*0.01
                tetax = roll_fit#+(roll-roll_prev)
                tetay = pitch_fit#+(pitch_curr-pitch_prev)
                xyz_curr,h1_prev,eul,fr,prev_group = plane_fit(rgb_img,xyz,tetax,tetay,prev_group,h_qe,h1_prev)
                roll_fit = eul[0]; pitch_fit = eul[1]
                # Scan Match
                yaw_t,tx_prev,minter_plus,minter_minus = scan_match(xyz_prev,xyz_curr,pRGB1_prev,pRGB1_curr,yaw_prev,yaw_curr,dxinternal*1e3/sc,dyinternal*1e3/sc,tx_prev,sm_status)
                # Slam
                xplus,xminus = SLAM(yaw_t,minter_plus,minter_minus,xplus,xminus)
                # Control
                mbypass = Control(xplus,xminus)
                feedback_prev = get_feedback(mbypass,feedback_prev)
                xyz_prev = xyz_curr
                pRGB1_prev = pRGB1_curr
                yaw_prev = yaw_curr
                roll_prev = roll
                pitch_prev = pitch_curr
                # roll,pitch and height memory of plane_fit
                h_qe.append(h1_prev)
                # Initializing waiting time for queue data
                msg_time = time.time()
                # For Graphics
                gqueue.put([rgb_img,xyz_curr,fr,xplus,xminus,mbypass])
                #d_list.append([int(h1_prev*100),int(pitch_fit*180/pi)])
            except Exception:
                eul = []
                #print(traceback.format_exc())
    # from scipy.io import savemat
    # savemat("data.mat", {"Frames":d_list})
    # send_feedback(b'0')

# Graphical showcase of the algorithm
def run_graphs(gqueue):
    #
    dummy_img = np.zeros((720,1280,3))
    #dummy_glvl = np.zeros((260,250,3))
    dummy_ctrl = np.zeros((380,360,3))

    gs_kw = dict(width_ratios=[5, 5,7,7])
    nav_arrow = []
    
    fig, (ax1, ax2,ax3,ax4) = plt.subplots(1, 4,figsize=(16,9),gridspec_kw=gs_kw)
    ax3.axis(np.array([-120,120,-80,250])/1e3*25)
    ax2.axis([-2, 2, -0.5, 1])
    ax2_data1 = ax2.plot([],[],'.')[0]
    ax2_data2 = ax2.plot([],[],'.r')[0]
    ax3_data1 = ax3.plot([],[],'.b',markersize=0.9)[0]
    ax3_data2 = ax3.plot([],[],'.C1',markersize=0.9)[0]
    ax3_data3 = ax3.plot([0,0],[0,6],'r')[0]
    ax3.plot(0/sc,0/sc,'o',linewidth=9,color = 'g',markersize=12)
    ax3.plot(np.array([-50,0,50])/1e3,np.array([210,350,210])/1e3,linewidth=5,color='r')
    ax1_data = ax1.imshow(dummy_img)
    axCtrl_data = ax4.imshow(dummy_ctrl)
    ax1.title.set_text("Sensory Input")
    ax2.title.set_text("Plane Fit")
    ax3.title.set_text("Perspective")
    ax4.title.set_text("Control")
    
    fig.show()
    msg_time = time.time()
    while time.time() - msg_time < 10:
        while not gqueue.empty():
            try:
            # Getting recent sensors data from RT_writer() function in another process (with the Queue)
                data_list = gqueue.get()
                rgb_img,x_plane,fr,xplus,xminus,mbypass = data_list[0],data_list[1],data_list[2],data_list[3],data_list[4],data_list[5]
                
                # Graphics
                rgb_img[:,int(rgb_img.shape[1] / 2 - 5): int(rgb_img.shape[1] / 2 + 5),:] = 0
                ax1_data.set_data(rgb_img)
                #building above & bellow ground image frame
                # glvl = minter_plus
                # glvl = np.dstack((glvl,np.zeros((sizemy,sizemx)),minter_minus))
                #Show frames of above and bellow ground level.
                ax2_data1.set_data(x_plane[:,1],x_plane[:,2])
                ax2_data2.set_data(x_plane[fr,1],x_plane[fr,2])
                tetaz = 90*pi/180 - 1*pi/180
                Rz = [[cos(tetaz),- sin(tetaz)],[sin(tetaz),cos(tetaz)]]
                x1plus=  dot(Rz,xplus.T).T / 1e3 * sc
                x1minus = dot(Rz,xminus.T).T / 1e3 * sc
                ax3_data1.set_data(x1plus[:,0],x1plus[:,1])
                ax3_data2.set_data(x1minus[:,0],x1minus[:,1])
                axCtrl_data.set_data(mbypass.astype(float))
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
                        # otherwise, choose the right bypass direction
                    else:
                        [s.remove() for s in nav_arrow]
                        nav_arrow = []
                        xloc= np.array([360,180])
                        yloc= np.array([32,30])
                        nav_arrow.append(ax4.annotate('Turn Right', xy = (xloc[0] , yloc[0]), xytext = (xloc[1] , yloc[1]),size = 12,weight = 'bold',color = 'red', arrowprops = dict(facecolor ='red',width = 5)))
                elif nav_arrow:
                        [s.remove() for s in nav_arrow]
                        nav_arrow = []

                msg_time = time.time()
                #ax1.relim()
                #ax1.autoscale_view(True,True,True)
                fig.canvas.draw()
                fig.canvas.flush_events()

            except Exception:
                #print(traceback.format_exc())
                pass

def RT_algo(pqueue,set_graphs = 1):
    # Profiling code (runtime/memory)
    gqueue = Queue()
    with cProfile.Profile() as pr:
        if set_graphs==0:
            run_algo(pqueue)
        else:
            run_algo_graph(pqueue,gqueue)
    ps = pstats.Stats(pr).sort_stats(SortKey.CUMULATIVE)
    ps.print_stats()

# times = np.array(times)
# plt.show()
# plt.plot(times[:,0],'or',markerfacecolor='none',label = 'madgwick')
# plt.plot(times[:,1],'+b',label = 'rgb')
# plt.plot(times[:,2],'.g',label = 'pcl')
# # for x in times[:,3]:
# #     plt.plot(x,'*c')
# plt.legend()
# plt.show()
