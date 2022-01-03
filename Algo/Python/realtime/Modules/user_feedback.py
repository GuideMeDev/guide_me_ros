import serial # pip install pyserial
import time
from Modules.utils import *

def send_feedback(x):
    ser = serial.Serial('COM4', 9600)
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
            send_feedback('2'.encode())
            #otherwise, choose the right bypass direction
        else:
            send_feedback('1'.encode())
    # to stop the signal
    else:
            send_feedback('0'.encode())
    
    
# while True:
#     input_value = input('Enter pixel position: ')
#     send_feedback(input_value.encode())

