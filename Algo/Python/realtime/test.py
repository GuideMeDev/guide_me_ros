from scipy.io import loadmat
from Modules.utils import *
import collections
from Modules.plane_init import plane_init
#from Modules.plane_fit import plane_fit
from Modules.plane_fit22 import plane_fit2
from Modules.plane_fit import plane_fit
import matplotlib.gridspec as gridspec
import time

frames = loadmat('rec7.mat')
frames = frames["Frames"]
h1_prev = 0
count = 0
h1_list = []
i_c = 0
pitch_fit_mean = []
key_v = None
h_1 = []
#fig = plt.figure(figsize=(6, 10))
#ax1 = fig.add_subplot(1, 1, 1)
plt.figure(figsize=(12,8))
gs = gridspec.GridSpec(2, 2)
#plt.suptitle("Method 1")
prev_group = [0,0,0]
h_qe = collections.deque(maxlen=10)
for i in range(200):
    frame = frames[i]
    i_c+=1
    acc_raw = frame[2]
    pitch_curr = frame[3][0][1]
    roll_curr = frame[3][0][0]
    
    if h1_prev == 0:
        h1_prev,eul,md_yg = plane_init(frame[0],frame[1],frame[3][0][0],frame[3][0][1])
        roll_fit = eul[0]; pitch_fit = eul[1]
        pitch_mean = 21*np.pi/180
        prev_group[1] = md_yg/sc1[1]
    else :
        if count==0:
            x3,h1_prev,eul,fr,f,prev_group,fr1,fr2,x_group = plane_fit2(frame[0],frame[1],eul[0],eul[1],sc_acc,prev_group,h_qe,h1_prev)
            roll_fit = eul[0]; pitch_fit = eul[1]

            count = 1
        #h1_prev = h1_prev+(np.mean(acc_raw[-33::,2]) - np.mean(acc_raw[-66:-33,2]))*0.01
        #tetax = roll_fit+(roll_curr-roll_prev)
        #tetay = pitch_fit#+(pitch_curr-pitch_prev)
        #fig.show()
        x3,h1_prev,eul,fr,f,prev_group,fr1,fr2,x_group = plane_fit2(frame[0],frame[1],roll_fit,pitch_fit,sc_acc,prev_group,h_qe,h1_prev)
        roll_fit = eul[0]; pitch_fit = eul[1]
        h_1.append(h1_prev)
        # plt.plot(x3[:,1],x3[:,2],'.')
        # plt.plot(x3[fr,1],x3[fr,2],'.g')
        #h1_list.append([h1_prev,pitch_fit])
        plt.suptitle(f"h1:{h1_prev*100:.2f}, pitch:{pitch_fit*180/np.pi:.2f},roll:{roll_fit*180/np.pi:.2f}")
        plt.subplot(gs[0, 0])
        plt.imshow(frame[0])
        plt.subplot(gs[0, 1])
        plt.axis([-2, 2, -0.2, 0.2])
        plt.plot(x3[:,1],x3[:,2],'.')
        #plt.plot(x3[fr,1],x3[fr,2],'.r')
        plt.plot(x3[f,1],x3[f,2],'+r')

        plt.subplot(gs[1, :])
        plt.plot(x_group[fr,1],x_group[fr,2],'.')
        plt.plot(x_group[fr1,1],x_group[fr1,2],'or',mfc='none')
        plt.plot(x_group[fr2,1],x_group[fr2,2],'sg',mfc='none')
        plt.axis([-1.5, 1.5, -0.2, 0.2])
        plt.pause(0.1)
        # if key_v != 'q':
        #     key_v = input("press q to loop")
        plt.clf() # to clear previous plots efficiently
    roll_prev = roll_curr; pitch_prev = pitch_curr
    sc_acc = np.mean(acc_raw)/70
    h_qe.append(h1_prev)
np.save("h1_scikit",np.array(h_1))

pass