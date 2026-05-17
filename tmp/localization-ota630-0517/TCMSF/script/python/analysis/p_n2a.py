import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


msf_state = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"))

plt.figure
plt.subplot(7, 1, 1)
plt.plot(msf_state[:, 0], msf_state[:, 19] * 180 / np.pi)
plt.plot(msf_state[:, 0], msf_state[:, 20] * 180 / np.pi)
plt.plot(msf_state[:, 0], msf_state[:, 21] * 180 / np.pi)
plt.subplot(7, 1, 2)
plt.plot(msf_state[:, 0], msf_state[:, 22])
plt.plot(msf_state[:, 0], msf_state[:, 23])
plt.plot(msf_state[:, 0], msf_state[:, 24])
plt.subplot(7, 1, 3)
plt.plot(msf_state[:, 0], msf_state[:, 25])
plt.plot(msf_state[:, 0], msf_state[:, 26])
plt.plot(msf_state[:, 0], msf_state[:, 27])
plt.subplot(7, 1, 4)
plt.plot(msf_state[:, 0], msf_state[:, 28])
plt.plot(msf_state[:, 0], msf_state[:, 29])
plt.plot(msf_state[:, 0], msf_state[:, 30])
plt.subplot(7, 1, 5)
plt.plot(msf_state[:, 0], msf_state[:, 31])
plt.plot(msf_state[:, 0], msf_state[:, 32])
plt.plot(msf_state[:, 0], msf_state[:, 33])
plt.subplot(7, 1, 6)
plt.plot(msf_state[:, 0], msf_state[:, 34])
plt.plot(msf_state[:, 0], msf_state[:, 35])
plt.plot(msf_state[:, 0], msf_state[:, 36])
plt.subplot(7, 1, 7)
plt.plot(msf_state[:, 0], msf_state[:, 37])
plt.plot(msf_state[:, 0], msf_state[:, 38])
plt.plot(msf_state[:, 0], msf_state[:, 39])
plt.show()
