from utils import *
def Control(xplus,xminus):
    x = xplus; xcurbe = xminus
    #find the control matrix t2 based on the pixels in the SLAM of x within specific region
    f= (x[:,0] > 1) & (x[:,0] < sizemx) & (abs(x[:,1]) < sizemy / 2)
    px = (x[f,0]).astype(int).astype(int)
    py = ((x[f,1]) + sizemy / 2).astype(int)
    t2 = np.zeros((sizemy,sizemx))
    t2[py, px] = 1
    #find the control matrix t2curbe based on the pixels in the SLAM of xcurbe within specific region
    f = (xcurbe[:,0] > 1) & (xcurbe[:,0] < sizemx) & (abs(xcurbe[:,1]) < sizemy / 2)
    py = np.around((xcurbe[f,0])).astype(int)
    px = np.around((xcurbe[f,1]) + sizemy / 2).astype(int)
    py[py>=260] -= 260
    t2curbe=np.zeros((sizemy,sizemx))
    t2curbe[px - 1,py]=1
    #create the control matrix
    mbypass = np.fliplr(np.fliplr(t2curbe).T)
    mbypass3 = np.fliplr(np.fliplr(t2).T) 

    #to change "below" level to orange color
    mbypass2 = np.fliplr(np.fliplr(t2curbe).T) / 2
    mbypass = np.dstack([mbypass, mbypass2,mbypass3])

    #to create in the control matrix the expected user trajectory
    mbypass[-dlen::,int((sizemy/2-dwi)):int((sizemy/2+dwi)),1] = 1 
    #to create in the control matrix the expected user bypass trajectory from
    #left and right sides of the obstacle
    mbypass[-dlen::,int((sizemy/2-2*dwi)) : int((sizemy/2-dwi)),1]=0.2
    mbypass[-dlen::,int((sizemy/2+dwi)) : int((sizemy/2+2*dwi)),1]=0.1
    # figure(2),subplot(2,2,[2,4]);imshow(double(mbypass))
    return mbypass