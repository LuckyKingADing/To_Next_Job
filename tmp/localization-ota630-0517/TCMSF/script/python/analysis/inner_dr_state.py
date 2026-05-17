import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


def rot2d(x, y, theta):
    x_ = x * math.cos(theta) - y * math.sin(theta)
    y_ = x * math.sin(theta) + y * math.cos(theta)
    return x_, y_


def GPStoXY(lat, lon, ref_lat, ref_lon):
    CONSTANTS_RADIUS_OF_EARTH = 6371000.0  # meters (m)
    # input GPS and Reference GPS in degrees
    # output XY in meters (m) X:North Y:East
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    ref_lat_rad = np.radians(ref_lat)
    ref_lon_rad = np.radians(ref_lon)

    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    ref_sin_lat = np.sin(ref_lat_rad)
    ref_cos_lat = np.cos(ref_lat_rad)

    cos_d_lon = np.cos(lon_rad - ref_lon_rad)

    x = (
        ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon
    ) * CONSTANTS_RADIUS_OF_EARTH
    y = cos_lat * np.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH
    return x, y


def InterpPos(lat, lon, lat0, lon0, t, t_ref):
    [y, x] = GPStoXY(lat, lon, lat0, lon0)
    return [np.interp(t_ref, t, x), np.interp(t_ref, t, y)]


def InterpState(state, t, t_ref):
    state_ = np.zeros([t_ref.shape[0], state.shape[1]])
    for i in range(state.shape[1]):
        state_[:, i] = np.interp(t_ref, t, state[:, i])
    return state_


inner_dr = np.array(pd.read_csv("data/tmp/inner_dr_state.csv"))
tcmsf = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"))

veh_state = np.array(pd.read_csv("data/tmp/veh_debug_state.csv"))


inner_dr = InterpState(inner_dr, inner_dr[:, 0], tcmsf[:, 0])

inner_dr = inner_dr[1000:-1, :]
tcmsf = tcmsf[1000:-1, :]


veh_interp = InterpState(veh_state, veh_state[:, 0], tcmsf[:, 0])


inner_dr[:, 1:3] = inner_dr[:, 1:3] - inner_dr[0, 1:3]

[tcmsf_y, tcmsf_x] = GPStoXY(
    inner_dr[:, 10], inner_dr[:, 11], inner_dr[0, 10], inner_dr[0, 11]
)


# 选两个点对准
# 因为DR是100Hz，这里差不多选了两个相差30米的点，来对准初态
fit_size = 5
skip = 50

ref_x = np.zeros(fit_size)
ref_y = np.zeros(fit_size)

dr_fit_x = np.zeros(fit_size)
dr_fit_y = np.zeros(fit_size)

for i in range(fit_size):
    ref_x[i] = tcmsf_x[i * skip]
    ref_y[i] = tcmsf_y[i * skip]
    dr_fit_x[i] = inner_dr[i * skip, 1]
    dr_fit_y[i] = inner_dr[i * skip, 2]


def eqn(theta):
    x_ = np.zeros(fit_size)
    y_ = np.zeros(fit_size)
    for i in range(fit_size):
        x_[i], y_[i] = rot2d(dr_fit_x[i], dr_fit_y[i], theta)
    # 优化的目的是对准两个轨迹的航向，使用叉乘是对准航向最优的方式
    # return np.sum(np.abs(x_ * ref_y - y_ * ref_x))  # 横向距离（垂足）
    return np.sum(np.abs(np.abs(x_ - ref_x) + np.abs(y_ - ref_y)))  # 横向距离（垂足）


# 这个地方初值可以设为0或者pi，因为投影点优化的时候，0和pi可能是等效的。为了使优化的结果是实际最优解，可以考虑调整初值
mymin = minimize(eqn, 0.0)

theta = mymin.x[0]

print(mymin)

# +++++++++++++

# 使用航向，将DR轨迹的初始状态与真值对齐
[pos_dr_x_rot, pos_dr_y_rot] = rot2d(inner_dr[:, 1], inner_dr[:, 2], theta)


plt.subplots(3, 1, sharex=True)
plt.subplot(3, 1, 1)
plt.plot(inner_dr[:, 0], inner_dr[:, 7])
plt.plot(tcmsf[:, 0], tcmsf[:, 5])
plt.legend(["roll_dr", "roll_tcmsf"])
plt.subplot(3, 1, 2)
plt.plot(inner_dr[:, 0], inner_dr[:, 8])
plt.plot(tcmsf[:, 0], -tcmsf[:, 4])
plt.legend(["pitch_dr", "pitch_tcsmf"])
plt.subplot(3, 1, 3)
plt.plot(inner_dr[:, 0], inner_dr[:, 9])
plt.plot(tcmsf[:, 0], tcmsf[:, 6])
plt.legend(["yaw_dr", "yaw_tcmsf"])

plt.subplots(3, 1, sharex=True)
plt.subplot(3, 1, 1)
plt.plot(inner_dr[:, 0], inner_dr[:, 4])
plt.subplot(3, 1, 2)
plt.plot(inner_dr[:, 0], inner_dr[:, 5])
plt.subplot(3, 1, 3)
plt.plot(inner_dr[:, 0], inner_dr[:, 6])

plt.figure("height")
plt.plot(inner_dr[:, 0], inner_dr[:, 3])
plt.plot(tcmsf[:, 0], tcmsf[:, 3])
plt.legend(["height_dr", "height_tcmsf"])

plt.figure("traj")
plt.plot(pos_dr_x_rot, pos_dr_y_rot)
plt.plot(tcmsf_x, tcmsf_y)
plt.legend(["traj_dr", "traj_tcmsf"])
plt.gca().set_aspect("equal")

plt.subplots(3, 1, sharex=True, sharey=True)
plt.subplot(3, 1, 1)
plt.plot(tcmsf[:, 0], tcmsf[:, 50])
plt.plot(veh_interp[:, 0], -veh_interp[:, 19])
plt.legend(["acc r imu", "acc r veh"])
plt.subplot(3, 1, 2)
plt.plot(tcmsf[:, 0], tcmsf[:, 51])
plt.plot(veh_interp[:, 0], veh_interp[:, 20])
plt.legend(["acc f imu", "acc f veh"])
plt.subplot(3, 1, 3)
plt.plot(tcmsf[:, 0], tcmsf[:, 84])
plt.legend(["acc u"])

plt.show()
