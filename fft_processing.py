import serial
import matplotlib.pyplot as plt
import time
import numpy as np
import collections
plt.rcParams['figure.figsize'] = [6,4]
plt.rcParams.update({'font.size': 8})
plt.rcParams['backend'] = 'Qt5Agg' # May need to install PyQt5 in pip for this backend

dt = 0.001
t = np.arange(0,5,dt)
n = len(t)

ser = serial.Serial('COM4', 115200)
ser.flushInput()

accel_x_values = []
accel_y_values = []
accel_z_values = []
mag_x_values = []
mag_y_values = []
mag_z_values = []
gyro_x_values = []
gyro_y_values = []
gyro_z_values = []
elapsed_times = []



            # Append the new sensor values and the elapsed time to the lists
start_time = time.time()
            
            
for i in range(0,n):
    ser_bytes = ser.readline()  # Read the newest output from the Pico
    decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")  # Decode the bytes to a string
    data = decoded_bytes.split(',')  # Assuming data is in the form 'accel_x,accel_y,accel_z'
    accel_x, accel_y, accel_z,mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z = [float(i) for i in data]  # Convert each part into a float
    
    accel_x_values.append(accel_x)
    accel_y_values.append(accel_y)
    accel_z_values.append(accel_z)
            
ser.close()

# FFT

f_hat_x = np.fft.fft(accel_x_values,n)
PSDx = f_hat_x * np.conj(f_hat_x) / n
freqx = (1/(dt*n)) * np.arange(n)

f_hat_y = np.fft.fft(accel_y_values,n)
PSDy = f_hat_y * np.conj(f_hat_y) / n
freqy = (1/(dt*n)) * np.arange(n)

f_hat_z = np.fft.fft(accel_z_values,n)
PSDz = f_hat_z * np.conj(f_hat_z) / n
freqz = (1/(dt*n)) * np.arange(n)

L = np.arange(1,np.floor(n/2),dtype='int')

# fig,ax = plt.subplots(2,1)

# plt.sca(ax[0])
# plt.plot(t,accel_x_values,color='k',linewidth=1,label='Noisy Data')
# plt.xlim(t[0],t[-1])
# plt.legend()

# plt.sca(ax[1])
# plt.plot(freq[L],PSD[L],color='c',linewidth=1,label='Noisy Data')
# plt.xlim(freq[L[0]],freq[L[-1]])
# plt.legend()
# plt.show()

# X Filter
indices = PSDx > 20
PSDclean = PSDx * indices
f_hat_x = indices * f_hat_x
f_filtered_x = np.fft.ifft(f_hat_x)

# Y Filter
indices = PSDy > 20
PSDclean = PSDy * indices
f_hat_y = indices * f_hat_y
f_filtered_y = np.fft.ifft(f_hat_y)

# Z Filter
indices = PSDz > 20
PSDclean = PSDz * indices
f_hat_z = indices * f_hat_z
f_filtered_z = np.fft.ifft(f_hat_z)

# Plot results
fig,ax = plt.subplots(2,1)

plt.sca(ax[0])
plt.plot(t,accel_x_values,color='c',linewidth=0.75,label='Noisy Data')
plt.plot(t,f_filtered_x,color='k',linewidth=1,label='Filtered Data')
plt.xlim(t[0],t[-1])
plt.legend()

plt.sca(ax[1])
plt.plot(freqx[L],PSDx[L],color='c',linewidth=1,label='Noisy Data')
plt.plot(freqx[L],PSDclean[L],color='k',linewidth=0.75,label='Filtered Data')
plt.xlim(freqx[L[0]],freqx[L[-1]])
plt.legend()

fig,ax = plt.subplots(2,1)

plt.sca(ax[0])
plt.plot(t,accel_y_values,color='c',linewidth=0.75,label='Noisy Data')
plt.plot(t,f_filtered_y,color='k',linewidth=1,label='Filtered Data')
plt.xlim(t[0],t[-1])
plt.legend()

plt.sca(ax[1])
plt.plot(freqy[L],PSDy[L],color='c',linewidth=1,label='Noisy Data')
plt.plot(freqy[L],PSDclean[L],color='k',linewidth=0.75,label='Filtered Data')
plt.xlim(freqy[L[0]],freqy[L[-1]])
plt.legend()
plt.show()

fig,ax = plt.subplots(2,1)

plt.sca(ax[0])
plt.plot(t,accel_z_values,color='c',linewidth=0.75,label='Noisy Data')
plt.plot(t,f_filtered_z,color='k',linewidth=1,label='Filtered Data')
plt.xlim(t[0],t[-1])
plt.legend()

plt.sca(ax[1])
plt.plot(freqz[L],PSDz[L],color='c',linewidth=1,label='Noisy Data')
plt.plot(freqz[L],PSDclean[L],color='k',linewidth=0.75,label='Filtered Data')
plt.xlim(freqz[L[0]],freqz[L[-1]])
plt.legend()
plt.show()