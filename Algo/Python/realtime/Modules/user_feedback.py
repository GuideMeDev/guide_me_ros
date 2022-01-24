import serial # pip install pyserial
import time
from Modules.utils import*

ser = serial.Serial('/dev/ttyUSB0', 9600,timeout=0,write_timeout=0.1)

def send_feedback(x):
    global ser
    ser.write(x)
    
def get_feedback(mbypass,fb_prev):
    obst=mbypass[:,:,0] + mbypass[:,:,2]
    scan=mbypass[:,:,1]
    t=(scan == 1)*obst
    obs_overlap = 25
    # stops the signal
    feedbk = b'0'
    # if there is significant overlap with an obstacle
    if sum(sum(t)) > obs_overlap:
    # search for overlap between the obstacle and the left and right bypass segments
        s = [sum(sum(scan*obst == 0.2)),sum(sum(scan*obst == 0.1))]
        s= s == min(s)
        s = sum(s*[1,2])
    # if the left segment has smaller overlap witht the obstacle, choose this bypass direction
        if s == 1:
            # Left
            feedbk = b'2'
            #print('2 left')
            #send_feedback('2'.encode())
            #otherwise, choose the right bypass direction
        else:
            # Right
            feedbk = b'1'
            #print('1 rigth')
            #send_feedback('1'.encode())
            send_feedback(b'1') # RIGHT

    if feedbk == fb_prev:
        #print(f"here{feedbk}")        
        send_feedback(feedbk)
    return feedbk

    print("----sent----")


