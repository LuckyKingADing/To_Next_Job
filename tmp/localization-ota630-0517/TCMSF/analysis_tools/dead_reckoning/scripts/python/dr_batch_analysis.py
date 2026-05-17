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


dr = np.array(pd.read_csv("data/parsed/dr.csv"))
imu = np.array(pd.read_csv("data/parsed/imu.csv"))

tcmsf = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"))
loc = tcmsf[:, [0, 0, 1, 2, 3, 7, 8, 9, 13, 14, 15, 10, 11, 12, 6]]
# loc = np.array(pd.read_csv("data/parsed/msf.csv"))

dr_interp = InterpState(dr, dr[:, 1], loc[:, 1])

# 调这个数值，可以改变初始点
# 最开始融合定位状态不是很稳定，调整IDX去掉头部的一些数据
IDX = 6000


loc = loc[IDX:-1, :]
dr_interp = dr_interp[IDX:-1, :]


loc[loc[:, 14] > 180, 14] = loc[loc[:, 14] > 180, 14] - 360

pos_dr = dr_interp[:, 2:4]
[pos_loc_y, pos_loc_x] = GPStoXY(loc[:, 2], loc[:, 3], loc[0, 2], loc[0, 3])


pos_dr_x = -(pos_dr[:, 0] - pos_dr[0, 0] + pos_loc_x[0])
pos_dr_y = -(pos_dr[:, 1] - pos_dr[0, 1] + pos_loc_y[0])


fit_size = 5
skip = 50


def align(start_idx, init_angle=180.0):

    # ----------------

    ref_x = np.zeros(fit_size)
    ref_y = np.zeros(fit_size)

    dr_fit_x = np.zeros(fit_size)
    dr_fit_y = np.zeros(fit_size)

    for i in range(fit_size):
        ref_x[i] = pos_loc_x[start_idx + i * skip] - pos_loc_x[start_idx]
        ref_y[i] = pos_loc_y[start_idx + i * skip] - pos_loc_y[start_idx]
        dr_fit_x[i] = pos_dr_x[start_idx + i * skip] - pos_dr_x[start_idx]
        dr_fit_y[i] = pos_dr_y[start_idx + i * skip] - pos_dr_y[start_idx]

    def eqn(theta):
        x_ = np.zeros(fit_size)
        y_ = np.zeros(fit_size)
        for i in range(fit_size):
            x_[i], y_[i] = rot2d(dr_fit_x[i], dr_fit_y[i], theta)
        # 优化的目的是对准两个轨迹的航向，使用叉乘是对准航向最优的方式
        return np.sum(np.abs(x_ * ref_y - y_ * ref_x))  # 横向距离（垂足）
        # return np.sum(np.abs(np.abs(x_ - ref_x) + np.abs(y_ - ref_y)))  # 距离

    mymin = minimize(eqn, init_angle)

    theta = mymin.x[0]

    print(mymin)

    return theta

    # +++++++++++++


plt.rcParams["legend.loc"] = "upper left"
plt.rcParams["figure.figsize"] = (8, 8)

LENGTH_ = 4000

info = np.zeros((math.ceil(pos_dr_x.shape[0] / LENGTH_), 2))

count = 0

for idx in range(0, pos_dr_x.shape[0] - LENGTH_, LENGTH_):
    count = count + 1
    theta = align(idx)
    [pos_dr_x_rot, pos_dr_y_rot] = rot2d(
        pos_dr_x[idx : idx + LENGTH_], pos_dr_y[idx : idx + LENGTH_], theta
    )

    plt.figure("traj_cut")
    plt.clf()
    # plt.plot(pos_loc_x[idx : idx + LENGTH_], pos_loc_y[idx : idx + LENGTH_])
    # # plt.plot(pos_dr_x[idx : idx + LENGTH_], pos_dr_y[idx : idx + LENGTH_])
    # plt.plot(pos_dr_x_rot, pos_dr_y_rot)

    plt.plot(
        pos_loc_y[idx : idx + LENGTH_] - pos_loc_y[idx],
        pos_loc_x[idx : idx + LENGTH_] - pos_loc_x[idx],
    )
    # plt.plot(pos_dr_x[idx : idx + LENGTH_], pos_dr_y[idx : idx + LENGTH_])
    plt.plot(pos_dr_y_rot - pos_dr_y_rot[0], pos_dr_x_rot - pos_dr_x_rot[0])

    plt.legend(["MSF", "DR"])

    plt.gca().set_aspect("equal")

    dx = (
        pos_dr_x_rot[-1] - pos_dr_x_rot[0] - (pos_loc_x[idx + LENGTH_] - pos_loc_x[idx])
    )
    dy = (
        pos_dr_y_rot[-1] - pos_dr_y_rot[0] - (pos_loc_y[idx + LENGTH_] - pos_loc_y[idx])
    )
    delta_dis_ = math.sqrt(dx * dx + dy * dy)
    plt.text(
        pos_dr_y_rot[-1] - pos_dr_y_rot[0],
        pos_dr_x_rot[-1] - pos_dr_x_rot[0],
        delta_dis_,
    )
    info[count - 1, :] = [delta_dis_, idx]
    plt.savefig(f"data/fig/traj-{count:010d}")
    if delta_dis_ > 5.0:
        plt.show()

    plt.close()

    # dhdg_ = loc[idx : idx + LENGTH_, 14] + dr_interp[idx : idx + LENGTH_, 9]
    # dhdg_[dhdg_ > 180.0] = dhdg_[dhdg_ > 180.0] - 360.0
    # dhdg_[dhdg_ < -180.0] = dhdg_[dhdg_ < -180.0] + 360.0

    # SUBPLOTS_NUM = 3
    # plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
    # plt.subplot(SUBPLOTS_NUM, 1, 1)
    # plt.plot(loc[idx : idx + LENGTH_, 1], loc[idx : idx + LENGTH_, 14])
    # plt.legend(["msf heading"])
    # plt.subplot(SUBPLOTS_NUM, 1, 2)
    # plt.plot(dr_interp[idx : idx + LENGTH_, 1], -dr_interp[idx : idx + LENGTH_, 9])
    # plt.legend(["dr heading"])
    # plt.subplot(SUBPLOTS_NUM, 1, 3)
    # plt.plot(dr_interp[idx : idx + LENGTH_, 1], dhdg_)
    # plt.legend(["delta heading"])
    # plt.savefig(f"data/fig/plot-{count:010d}")

np.savetxt("data/fig/info.csv", info, delimiter=",", fmt="%s")


SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(imu[2:-1, 8], imu[2:-1, 0] - imu[1:-2, 0])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(imu[:, 8], imu[:, 1])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(imu[2:-1, 8], imu[2:-1, 8] - imu[1:-2, 8])
plt.show()
