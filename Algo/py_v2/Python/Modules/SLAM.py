from utils import *

def SLAM(yawt_curr,minter_plus,minter_minus,xplus,xminus):
    c = sig.convolve2d(minter_plus,np.ones((4,4)) / 16,mode = 'same')
    c = (c > 0.5)
    cminter_plus=copy(c)
    #find convolved minter_minus (the filtered D-frame), by using low pass filter on minter_minus
    c = sig.convolve2d(minter_minus,np.ones((4,4)) / 16,mode = 'same')
    c = (c > 0.5)
    cminter_minus=copy(c)
    #step 1, transformation from pixels to coordinates, step 2, translation back in time of frame (i+1) to frame (i)
    (pyplus,pxplus) = np.where(cminter_plus > 0)
    pyplus = pyplus - sizemy / 2
    px1plus = pxplus + yawt_curr[2]
    py1plus = pyplus + yawt_curr[1]
    #same as above for pycurbe and pxcurbe, and for pyfloor and pxfloor
    (pyminus,pxminus) = np.where(cminter_minus > 0)
    pyminus=pyminus - sizemy / 2
    px1minus = pxminus + yawt_curr[2]
    py1minus = pyminus + yawt_curr[1]
    #rotation back in time of frame (i+1) to frame (i) of plus and of minus
    tetaz = -yawt_curr[0]
    Rz = [[cos(tetaz),-sin(tetaz)],[sin(tetaz),cos(tetaz)]]
    #adding frames (i) plus and minus to the SLAM of xplus and xminus
    t = dot(Rz,[px1plus.T,py1plus.T]).T
    xplus = np.concatenate(([[0,0]],xplus,t),axis = 0)
    t = dot(Rz,[pxminus.T,py1minus.T]).T
    xminus=np.concatenate((xminus,t))
    xplus = xplus[xplus[:,0] > -50]
    xminus = xminus[xminus[:,0] > -50]
    #moving forward in time (rotation and translation) both SLAM of x and SLAM of xcurbe
    tetaz = yawt_curr[0]
    Rz = [[cos(tetaz),- sin(tetaz)],[sin(tetaz),cos(tetaz)]]
    xplus = dot(Rz,xplus.T).T
    xminus = dot(Rz,xminus.T).T
    xplus[:,0] = xplus[:,0] - yawt_curr[2]
    xplus[:,1] = xplus[:,1] - yawt_curr[1]
    xminus[:,0] = xminus[:,0] - yawt_curr[2]
    xminus[:,1] = xminus[:,1] - yawt_curr[1]
    #add the dframe (i+1) to the SLAM
    xplus = np.concatenate((xplus,np.array([pxplus.T,pyplus.T]).T))
    xminus = np.concatenate((xminus,np.array([pxminus.T,pyminus.T]).T))
    return xplus,xminus

