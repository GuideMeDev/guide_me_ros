import serial # pip install pyserial
import time

def send_feedback(x):
    ser = serial.Serial('COM4', 9600)
    ser.write(x)

# while True:
#     input_value = input('Enter pixel position: ')
#     send_feedback(input_value.encode())

