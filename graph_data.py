import serial
import matplotlib.pyplot as plt
import time
import numpy as np
import collections

# Assuming Pico is connected to 'COM4' and communicates at 115200 baud rate
ser = serial.Serial('COM4', 115200)
ser.flushInput()

# Initialize plot, turning interactive mode ON for real time plot
plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='accel_x')
line2, = ax.plot([], [], label='accel_y')
line3, = ax.plot([], [], label='accel_z')

# Time-based variable declarations
samples_per_second = 10
max_samples = 5 * samples_per_second
start_time = time.time()

# Initialize deques for the new data and times
accel_x_values = collections.deque(maxlen=max_samples)
accel_y_values = collections.deque(maxlen=max_samples)
accel_z_values = collections.deque(maxlen=max_samples)
elapsed_times = collections.deque(maxlen=max_samples)

try:
    while True:
        try:
            ser_bytes = ser.readline()  # Read the newest output from the Pico
            decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")  # Decode the bytes to a string
            data = decoded_bytes.split(',')  # Assuming data is in the form 'accel_x,accel_y,accel_z'
            accel_x, accel_y, accel_z = [float(i) for i in data]  # Convert each part into a float

            # Append the new sensor values and the elapsed time to the lists
            accel_x_values.append(accel_x)
            accel_y_values.append(accel_y)
            accel_z_values.append(accel_z)
            elapsed_time = time.time() - start_time
            elapsed_times.append(elapsed_time)
            
            # Update line data and axes limits
            line1.set_data(list(elapsed_times), list(accel_x_values))
            line2.set_data(list(elapsed_times), list(accel_y_values))
            line3.set_data(list(elapsed_times), list(accel_z_values))
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.1)
            

        except Exception as e:
            print(e)
            break
except KeyboardInterrupt:
    print("Program Interrupted")
finally:
    ser.close()  # Remember to close the connection when finished
