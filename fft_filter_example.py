import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = [6,4]
plt.rcParams.update({'font.size': 8})
plt.rcParams['backend'] = 'Qt5Agg' # May need to install PyQt5 in pip for this backend


# Create signals at frequencies 50 and 120
dt = 0.001
t = np.arange(0,1,dt)
f1 = np.sin(2*np.pi*50*t)
f2 = np.sin(2*np.pi*120*t)
f = f1 + f2
f_clean = f


# Add random noise to the added signals
f = f + 2.5*np.random.randn(len(t))


# FFT
n = len(t)
f_hat = np.fft.fft(f,n)
PSD = f_hat * np.conj(f_hat) / n
freq = (1/(dt*n)) * np.arange(n)
L = np.arange(1,np.floor(n/2),dtype='int')


# Filter
indices = PSD > 100
PSDclean = PSD * indices
f_hat = indices * f_hat
f_filtered = np.fft.ifft(f_hat)


# Plot results
fig, ax = plt.subplots(3,1)

plt.sca(ax[0])
plt.plot(t,f,color='c',linewidth=0.75,label='Noisy Data')
plt.plot(t,f_clean,color='k',linewidth=1,label='Clean Data')
plt.xlim(t[0],t[-1])
plt.legend()

plt.sca(ax[1])
plt.plot(t,f_filtered,color='k',linewidth=1,label='Filtered')
plt.xlim(t[0],t[-1])
plt.legend()

plt.sca(ax[2])
plt.plot(freq[L],PSD[L],color='c',linewidth=1,label='Noisy Data')
plt.plot(freq[L],PSDclean[L],color='k',linewidth=0.75,label='Filtered Data')
plt.xlim(freq[L[0]],freq[L[-1]])
plt.legend()

plt.show()