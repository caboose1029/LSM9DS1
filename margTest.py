import serial
import matplotlib.pyplot as plt
import time
import numpy as np
import collections
from ahrs.filters import QUEST
from ahrs import Quaternion

# Assuming Pico is connected to 'COM4' and communicates at 115200 baud rate
ser = serial.Serial('COM4', 115200)
ser.flushInput()
quest = QUEST()
# Time-based variable declarations
samples_per_second = 1000
max_samples = 10
start_time = time.time()
        
# Initialize deques for the new data and times
accel_values = np.zeros((10,3))
mag_values = np.zeros((10,3))
gyro_values = np.zeros((10,3))
Q = [1., 0., 0., 0.,]

def read_values():
    for i in range(10):
        ser_bytes = ser.readline()  # Read the newest output from the Pico
        decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")  # Decode the bytes to a string
        data = decoded_bytes.split(',')  # Assuming data is in the form 'accel_x,accel_y,accel_z'
        accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z = [float(i) for i in data]  # Convert each part into a float

            # Append the new sensor values and the elapsed time to the lists
        accel_values[i] = [accel_x, accel_y, accel_z]
        mag_values[i] = [mag_x, mag_y, mag_z]
        gyro_values[i] = [gyro_x, gyro_y, gyro_z]
    acc_data = [np.average(accel_values[:,0]), np.average(accel_values[:,1]), np.average(accel_values[:, 2])]
    mag_data = [np.average(mag_values[:,0]), np.average(mag_values[:,1]), np.average(mag_values[:, 2])]
    #gyro_data =  [np.average(gyro_values[:,0]), np.average(gyro_values[:,1]), np.average(gyro_values[:, 2])]
    return(acc_data, mag_data)

def quaternion_estimate(acc_data, mag_data):
    t = 0
    Q[t] = quest.estimate(acc=acc_data, mag=mag_data)
    q = Quaternion(Q[t])
    angle = q.to_angles() * 180 / np.pi
    #angles = DataLogger.add_data_point(angle)
    t += t
    return angle
    
try:
    while True:
        try:
            acc_data, mag_data = read_values()
            angle = quaternion_estimate(acc_data, mag_data)
            print(angle)
            
        except Exception as e:
            print(e)
            break
except KeyboardInterrupt:
    print("Program Interrupted")
finally:
    ser.close()  # Remember to close the connection when finished

