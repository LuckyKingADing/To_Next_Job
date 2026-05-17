import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


imu = np.array(pd.read_csv("data/Record_data/LOG/ST25_1579/parsed/imuinfo.csv"))

SUBNUM = 3

plt.subplots(SUBNUM, 1, sharex=True)
plt.subplot(SUBNUM, 1, 1)
plt.scatter(imu[:, 0], imu[:, 1], s=0.3)
plt.subplot(SUBNUM, 1, 2)
plt.scatter(imu[:, 0], imu[:, 2], s=0.3)
plt.subplot(SUBNUM, 1, 3)
plt.scatter(imu[:, 0], imu[:, 3], s=0.3)
plt.show()
