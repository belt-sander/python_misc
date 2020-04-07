#!/usr/bin/env python

### garbage plotting script ###

import numpy as np
import matplotlib.pyplot as plt

d = np.genfromtxt('test.txt', delimiter=',')

t = d[:,0]
lf = d[:,1]
rf = d[:,2]
lr = d[:,3]
rr = d[:,4]

plt.plot(t, lf, label='lf m/s')
plt.plot(t, rf, label='rf m/s')
plt.plot(t, lr, label='lr m/s')
plt.plot(t, rr, label='rr m/s')
plt.legend()

plt.show()
