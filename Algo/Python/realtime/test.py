from scipy.io import loadmat
from Modules.utils import *
from Modules.plane_init import plane_init
#from Modules.plane_fit import plane_fit
from Modules.plane_fit2 import plane_fit2
frames = loadmat('mo_rec.mat')
frames = frames["Frames"]
h0 = 0
#fig = plt.figure(figsize=(6, 10))
#ax1 = fig.add_subplot(1, 1, 1)
plt.figure(figsize=(12,8))
plt.suptitle("Method 1")

for frame in frames:
    if h0 == 0:
        h0,eul0 = plane_init(frame[0],frame[1],frame[3][0][0],frame[3][0][1])
        h0 = 1.15
    else :
        
        #fig.show()
        x3,h,x,fr = plane_fit2(frame[0],frame[1],eul0[0],eul0[1],h0)
        # plt.plot(x3[:,1],x3[:,2],'.')
        # plt.plot(x3[fr,1],x3[fr,2],'.g')
        plt.suptitle("Method 2")
        plt.subplot(1,2,1)
        plt.imshow(frame[0])
        plt.subplot(1,2,2)
        plt.plot(x3[:,1],x3[:,2],'.')
        plt.plot(x3[fr,1],x3[fr,2],'.r')
        plt.axis([-2, 2, -0.5, 1])
        plt.pause(0.1)
        plt.clf() # to clear previous plots efficiently