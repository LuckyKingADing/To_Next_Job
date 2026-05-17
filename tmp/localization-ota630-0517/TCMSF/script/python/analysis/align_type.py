import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


msf_veh = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)

plt.plot(msf_veh[:, 0], msf_veh[:, 52])
plt.legend(["inner align type"])
plt.show()

align_state = np.array(pd.read_csv("data/tcmsf/msf_veh_frame.csv"))

plt.subplots(2, 1, sharex=True)
plt.subplot(2, 1, 1)
plt.plot(msf_veh[:, 0], msf_veh[:, 52])
plt.legend(["inner align type"])
plt.subplot(2, 1, 2)
plt.plot(align_state[:, 0], align_state[:, 1])
plt.legend(["debounced align type"])
plt.show()
