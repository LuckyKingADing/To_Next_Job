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


imu = np.array(pd.read_csv("data/tcmsf/imu.csv"))
dr = np.array(pd.read_csv("data/tcmsf/dr.csv"))


dr_interp = InterpState(dr, dr[:, 1], imu[:, 1])

# 使用两帧dr反推前后帧角速度
dr_w_z = (dr_interp[1:-1, 9] - dr_interp[0:-2, 9]) / (
    dr_interp[1:-1, 1] - dr_interp[0:-2, 1]
)

# 使用两帧imu计算前后帧角速度均值
imu_w_z = (imu[1:-1, 7] + imu[0:-2, 7]) / 2.0


# 计算标准化互相关（消除量纲影响）
corr = correlate(
    (dr_w_z - dr_w_z.mean()) / dr_w_z.std(),
    (imu_w_z - imu_w_z.mean()) / imu_w_z.std(),
    mode="full",
)

# 获取最大相关性的时移
lags = np.arange(-len(dr_w_z) + 1, len(imu_w_z))  # 时移量范围
best_lag = lags[np.argmax(corr)]  # 最佳对齐偏移量
print(f"最佳对齐偏移量: {best_lag}个时间单位")

plt.figure("angle")
plt.plot(dr_interp[:, 1], dr_interp[:, 9])
plt.legend(["dr angle"])

plt.figure("angle vel")
plt.plot(dr_w_z)
plt.plot(imu_w_z)
plt.legend(["dr", "imu_z"])
# plt.show()

plt.figure("correlate")
plt.plot(lags[np.argmax(corr)], corr[np.argmax(corr)], "o")
plt.text(lags[np.argmax(corr)] + 100, corr[np.argmax(corr)], lags[np.argmax(corr)])
plt.plot(lags, corr)
plt.show()
