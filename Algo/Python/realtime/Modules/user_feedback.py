import serial # pip install pyserial
import time
from Modules.utils import*

ser = serial.Serial('/dev/ttyUSB0', 9600)

def send_feedback(x):
    global ser
    ser.write(x)
    
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
            #print('2 left')
            send_feedback(b'2') # LEFT
            #send_feedback('2'.encode())
            #otherwise, choose the right bypass direction
        else:
            #print('1 rigth')
            #send_feedback('1'.encode())
            send_feedback(b'1') # RIGHT
    # to stop the signal
    else:
            #print('0 stop ')
            send_feedback(b'0')
            #send_feedback('0'.encode())


