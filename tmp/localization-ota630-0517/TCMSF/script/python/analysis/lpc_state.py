import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


lpc_state = np.array(pd.read_csv("data/tmp/lpc_debug_state.csv"))

SUBPLOTS_NUM = 7
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(lpc_state[:, 0], lpc_state[:, 1])
plt.axhline(0.3)
plt.ylim(0.0, 0.4)
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(lpc_state[:, 0], lpc_state[:, 2])
plt.axhline(0.2)
plt.ylim(0.0, 0.4)
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(lpc_state[:, 0], lpc_state[:, 3])
plt.axhline(0.4)
plt.subplot(SUBPLOTS_NUM, 1, 4)
plt.plot(lpc_state[:, 0], lpc_state[:, 4])
plt.axhline(0.4)
plt.subplot(SUBPLOTS_NUM, 1, 5)
plt.plot(lpc_state[:, 0], lpc_state[:, 5])
plt.axhline(1.5)
plt.subplot(SUBPLOTS_NUM, 1, 6)
plt.plot(lpc_state[:, 0], lpc_state[:, 6])
plt.subplot(SUBPLOTS_NUM, 1, 7)
plt.plot(lpc_state[:, 0], lpc_state[:, 7] * 180.0 / np.pi)
plt.show()
