import scipy
import numpy as np
import matplotlib.pyplot as plt
from cmath import phase, atan
from math import pi

detect_chirp = scipy.fromfile(open('../data/detect'), dtype=scipy.complex64)
sync_chirp = scipy.fromfile(open('../data/sync'), dtype=scipy.complex64)


up_chirp = detect_chirp
down_chirp = sync_chirp

down_chirp_conj = []
up_chirp_conj = []
for i in range(1, len(up_chirp)):
    down_chirp_conj.append(down_chirp[i]*np.conj(down_chirp[i-1]))
    up_chirp_conj.append(up_chirp[i]*np.conj(up_chirp[i-1]))

down_chirp_freq = np.array([phase(down_freq) for down_freq in down_chirp_conj])
up_chirp_freq = [phase(up_freq) for up_freq in up_chirp_conj]

x = [i for i in range(len(up_chirp_freq))]

plt.plot(x, up_chirp_freq, 'r.-')
plt.plot(x, down_chirp_freq, 'b.-')
plt.show()

# plt.plot(x, up_chirp[:-1], 'r--')
plt.plot(x, down_chirp[:-1], 'b--')
plt.show()

print(np.where(np.array(down_chirp_freq) < 0)[0][0])
print(up_chirp[len(up_chirp)-1])
print('read rpp0')
