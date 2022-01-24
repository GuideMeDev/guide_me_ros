from scipy.signal import savgol_filter
from Modules.utils import *

scale_f = 1

def smooth(y, box_pts):
    box = np.ones(box_pts) / box_pts
    y_smooth = np.convolve(y, box, mode = 'same')
    return y_smooth
    
def find_peaks(ax):
    ax_diff = (np.diff(ax)>0).astype(int)
    amin=np.where(np.diff(ax_diff)>0)[0];amin=amin+1
    amin=amin[ax[amin]<-0.3]
    amin=amin[(FPS_IMU*scale_f-amin)>FPS_IMU/12]
    amin=amin[(ax[amin+4]-ax[amin]>0.01)]
    amin_i = np.where(np.diff(amin)<FPS_IMU/5)[0]
    amin[amin_i+1]=amin[amin_i]
    amin[amin_i+1]=amin[amin_i]
    amax=np.where(np.diff(ax_diff)<0)[0];amax=amax+1
    amax=amax[ax[amax]>0.3]
    amax=amax[(FPS_IMU*scale_f-amax)>FPS_IMU/12]
    amax=amax[(ax[amax]-ax[amax+4]>0.01)]
    amax_i = np.where(np.diff(amax)<FPS_IMU/5)[0]
    amax[amax_i+1]=amax[amax_i]
    amax[amax_i+1]=amax[amax_i]
    return amax,amin

def calc_dx(vi_curr,vi_prev,pitch_curr,pitch_prev):
    dxexternal = 0
    dxinternal = 0
    dt=1/6
    h=1.2
    vi1 = vi_prev
    vi2 = vi_curr
    tetai_1 = pitch_curr
    tetai = pitch_prev
    xi_1 = np.arctan(np.pi/2-tetai_1)*h
    xi = np.arctan(np.pi/2-tetai)*h
    dxexternal = (vi1+vi2)/2*dt
    dxinternal = (xi+dxexternal)-xi_1
    return dxinternal

# get x-translation
def TF_x(acc_raw,pitch_curr,pitch_prev,vi_prev = None):
    secs = 3
    dt1 = 0
    dt2 = 0
    vi = 0
    dv = 0
    dxinternal = 0
    # fig, (ax1, ax2,ax3) = plt.subplots(1, 3,figsize=(12,7))
    # dummy_ = np.zeros(200)
    """
    When plotting a line with one input, this input is the y-data axis. and the x-input is automatically created
    as the respective index of the element.
    which is why i deliever the range of fps_imu as the x-data (for ax1_data1).
    """
    #
    # ax1_data1 = ax1.plot([],[],'.-')[0] 
    # ax1_data2 = ax1.plot([],[],'o')[0]
    # ax1_data3 = ax1.plot([],[],'o')[0]
    # ax2_data = ax2.plot([],[],'-')[0]
    # ax3_data = ax3.plot([],[],'-')[0]
    # fig.show()
    #
    frequency_constraints = [0.001, 0.08]  # cutoff
    t4_i = -FPS_IMU*scale_f+1
    # time_lowess = np.arange(0, FPS_IMU)/FPS_IMU
    smooth_ker = int(FPS_IMU/4) + 1
    t2=np.zeros((FPS*secs,3));t3=np.zeros((FPS*secs,3))
    
    t1=acc_raw
    t1a=np.sqrt(t1[:,0]**2+t1[:,1]**2)
    t4=t1a
    t4=t4-np.mean(t4)
    t4 = savgol_filter(t4, 87, 3)
    #ax1.set(xlim=(1, len(t4)), ylim=(-4, 5))
    #ax1_data1.set_data(fps_range,t4)
    ax=t4[t4_i::]
    amax,amin = find_peaks(ax)
    # ax1_data2.set_data(amin+(len(t4)-FPS_IMU*scale_f),ax[amin])
    # ax1_data3.set_data(amax+(len(t4)-FPS_IMU*scale_f),ax[amax])

    f = np.where(len(ax)>=amax)[0]
    fmin = np.where(len(ax)>=amin)[0]
    if len(f)>0 and len(fmin)>0:
        amaxi = max(amax)
        amini = max(amin)
        amaxmin = ax[amaxi] - ax[amini]
        if amaxmin < 1.4 or not amini or not amaxi or ax[amaxi]<0.5 or ax[amini]>-0.5 or max([amini,amaxi])<FPS_IMU/2:
            amaxmin = 0
        k1=0.4 # Z axis, which is x generally
        L = k1*(amaxmin)**0.25 # ith step length
        dt = (2*abs(amaxi-amini)/FPS_IMU) # delta time of each step
        if dt!=dt1:
            dt2=dt1;dt1=dt
        dt3=0.7*dt+0.3*dt2
        v0 = L/dt3 # initial velocity
        k2=0.07 # for X axis - which is y axis generally
        dv = (k2*(ax[-1]-(ax[amaxi]+ax[amini])/2)) # Acceleration
        vi = v0+dv; # overall velocity
        dt1=dt
        # ax2.plot(v0)
        # ax3.plot(dv)
        # fig.canvas.draw()
        # fig.canvas.flush_events()
        # ax2.clear()
        # ax3.clear()
    
    #plt.close()
    # For velocity plotting, uncomment :
    # plt.plot(time_sampled,vi_6)
    # plt.plot(time_sampled,vi_6-vi[time_sampled])
    # plt.show()
    if vi_prev != None:
        #dxinternal = calc_dx(vi,vi_prev,pitch_curr,pitch_prev)
        dxinternal = calc_dx(vi,vi_prev,pitch_curr,pitch_prev)
    return dxinternal,vi

# get y-translation
def TF_y(acc_raw,pitch_curr,pitch_prev,dv_prev = None):
    secs = 3
    dt1 = 0
    dt2 = 0
    vi = 0
    dv = 0
    dyinternal = 0
    # fig, (ax1, ax2,ax3) = plt.subplots(1, 3,figsize=(12,7))
    # dummy_ = np.zeros(200)
    """
    When plotting a line with one input, this input is the y-data axis. and the x-input is automatically created
    as the respective index of the element.
    which is why i deliever the range of fps_imu as the x-data (for ax1_data1).
    """
    #
    # ax1_data1 = ax1.plot([],[],'.-')[0] 
    # ax1_data2 = ax1.plot([],[],'o')[0]
    # ax1_data3 = ax1.plot([],[],'o')[0]
    # ax2_data = ax2.plot([],[],'-')[0]
    # ax3_data = ax3.plot([],[],'-')[0]
    # fig.show()
    #
    frequency_constraints = [0.001, 0.08]  # cutoff
    t4_i = -FPS_IMU*scale_f+1
    # time_lowess = np.arange(0, FPS_IMU)/FPS_IMU
    smooth_ker = int(FPS_IMU/4) + 1
    t2=np.zeros((FPS*secs,3));t3=np.zeros((FPS*secs,3))
    # ax2.set(xlim=(1,len(acc_raw_sampled)), ylim=(0,2))
    # ax3.set(xlim=(1,len(acc_raw_sampled)), ylim=(-0.5,0.5))
    
    
    t1=acc_raw
    t1a=np.sqrt(t1[:,0]**2+t1[:,1]**2)
    t4=t1a
    t4=t4-np.mean(t4)
    t4 = savgol_filter(t4, 87, 3)
    # ax1.set(xlim=(1, len(t4)), ylim=(-4, 5))
    # ax1_data1.set_data(fps_range,t4)
    ax=t4[t4_i::]
    amax,amin = find_peaks(ax)
    # ax1_data2.set_data(amin+(len(t4)-FPS_IMU*scale_f),ax[amin])
    # ax1_data3.set_data(amax+(len(t4)-FPS_IMU*scale_f),ax[amax])

    f = np.where(len(ax)>=amax)[0]
    fmin = np.where(len(ax)>=amin)[0]
    if len(f)>0 and len(fmin)>0:
        amaxi = max(amax)
        amini = max(amin)
        amaxmin = ax[amaxi] - ax[amini]
        if amaxmin < 1.4 or not amini or not amaxi or ax[amaxi]<0.5 or ax[amini]>-0.5 or max([amini,amaxi])<FPS_IMU/2:
            amaxmin = 0
        k1=0.46 # Z axis, which is x generally
        L = k1*(amaxmin)**0.25 # ith step length
        dt = (2*abs(amaxi-amini)/FPS_IMU) # delta time of each step
        if dt!=dt1:
            dt2=dt1;dt1=dt
        dt3=0.7*dt+0.3*dt2
        v0 = L/dt3 # initial velocity
        k2=0.07 # for X axis - which is y axis generally
        dv = (k2*(ax[-1]-(ax[amaxi]+ax[amini])/2)) # Acceleration
        vi = v0+dv; # overall velocity
        dt1=dt
        # ax2.plot(v0)
        # ax3.plot(dv)
        # fig.canvas.draw()
        # fig.canvas.flush_events()
        # ax2.clear()
        # ax3.clear()
    
    # plt.close()
    # For velocity plotting, uncomment :
    # plt.plot(time_sampled,vi_6)
    # plt.plot(time_sampled,vi_6-vi[time_sampled])
    # plt.show()
    if dv_prev != None:
        #dxinternal = calc_dx(vi,vi_prev,pitch_curr,pitch_prev)
        dyinternal = calc_dx(dv,dv_prev,pitch_curr,pitch_prev)
    return dyinternal,dv
