import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt
from scipy.signal import correlate


def InterpState(state, t, t_ref):
    state_ = np.zeros([t_ref.shape[0], state.shape[1]])
    for i in range(state.shape[1]):
        state_[:, i] = np.interp(t_ref, t, state[:, i])
    return state_


tc = np.array(pd.read_csv("data/tcmsf/tcmsf.csv"))
dr = np.array(pd.read_csv("data/tcmsf/dr.csv"))
gnss = np.array(pd.read_csv("data/tcmsf/gnss.csv"))

tc_itp = InterpState(tc, tc[:, 1], dr[:, 1])

plt.subplots(3, 1, sharex=True)
plt.subplot(3, 1, 1)
plt.scatter(tc_itp[:, 1], tc_itp[:, 18], s=0.3)
plt.scatter(tc[:, 1], tc[:, 18], s=0.3)
plt.scatter(gnss[:, 1], gnss[:, 8], s=0.3)
plt.legend(["TCMSF heading"])
plt.subplot(3, 1, 2)
plt.scatter(dr[:, 1], dr[:, 9], s=0.3)
plt.legend(["DR heading"])
plt.subplot(3, 1, 3)
plt.scatter(dr[:, 1], dr[:, 9] + tc_itp[:, 18], s=0.3)
plt.legend(["delta heading"])

plt.subplots(3, 1, sharex=True)
plt.subplot(3, 1, 1)
plt.scatter(tc_itp[:, 1], tc_itp[:, 20], s=0.3)
plt.scatter(gnss[:, 1], gnss[:, 5], s=0.3)
plt.legend(["TC_V_E", "G_V_E"])
plt.subplot(3, 1, 2)
plt.scatter(tc_itp[:, 1], tc_itp[:, 21], s=0.3)
plt.scatter(gnss[:, 1], gnss[:, 6], s=0.3)
plt.subplot(3, 1, 3)
plt.scatter(tc_itp[:, 1], tc_itp[:, 22], s=0.3)
plt.scatter(gnss[:, 1], gnss[:, 7], s=0.3)

plt.subplots(2, 1, sharex=True, sharey=True)
plt.subplot(2, 1, 1)
plt.plot(tc[1:-1:, 1], tc[1:-1, 1] - tc[0:-2, 1])
plt.legend(["TCMSF dt"])
plt.subplot(2, 1, 2)
plt.plot(dr[1:-1:, 1], dr[1:-1, 1] - dr[0:-2, 1])
plt.legend(["DR dt"])
plt.show()
