import serial # pip install pyserial
import time

ser = serial.Serial('COM4', 9600)
def send_feedback(x):
    ser.write(x)

while True:
    input_value = input('Enter pixel position: ')
    send_feedback(input_value.encode())

