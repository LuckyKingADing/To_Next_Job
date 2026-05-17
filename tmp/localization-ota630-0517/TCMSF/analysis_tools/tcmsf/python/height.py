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

plt.plot(tc[:, 1], tc[:, 19])
plt.show()
