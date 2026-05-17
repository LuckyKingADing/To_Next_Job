import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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


def vector_angle_2d_simple(v1, v2, degrees=False):
    """
    专门针对二维向量的简化版本
    使用atan2函数直接计算带符号的角度[5,8](@ref)
    """
    if len(v1) != 2 or len(v2) != 2:
        raise ValueError("此函数仅适用于二维向量")

    v1 = np.array(v1)
    v2 = np.array(v2)

    # 计算每个向量与x轴的夹角[5](@ref)
    angle1 = math.atan2(v1[1], v1[0])
    angle2 = math.atan2(v2[1], v2[0])

    # 计算角度差[8](@ref)
    angle_diff = angle2 - angle1

    # 将角度规范化到[-π, π]范围内
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi

    if degrees:
        return math.degrees(angle_diff)
    else:
        return angle_diff


dr = np.array(pd.read_csv("data/tcmsf/dr.csv"))
# dr = np.array(pd.read_csv("data/tcmsf/dr_replay.csv"))
loc = np.array(pd.read_csv("data/tcmsf/tcmsf.csv"))
imu = np.array(pd.read_csv("data/tcmsf/imu.csv"))
veh = np.array(pd.read_csv("data/tcmsf/vehicle.csv"))
gnss = np.array(pd.read_csv("data/tcmsf/gnss.csv"))


# ---------------------
# 规整化一下，尽量拉齐起始时间
min_timestamp = 0.0
if dr[0, 1] > min_timestamp:
    min_timestamp = dr[0, 1]
if loc[0, 1] > min_timestamp:
    min_timestamp = loc[0, 1]
if imu[0, 1] > min_timestamp:
    min_timestamp = imu[0, 1]
if veh[0, 1] > min_timestamp:
    min_timestamp = veh[0, 1]
if gnss[0, 1] > min_timestamp:
    min_timestamp = gnss[0, 1]


loc = loc[loc[:, 1] > min_timestamp, :]
dr = dr[dr[:, 1] > min_timestamp, :]
imu = imu[imu[:, 1] > min_timestamp, :]
veh = veh[veh[:, 1] > min_timestamp, :]
gnss = gnss[gnss[:, 1] > min_timestamp, :]

# -------------------------
# -------------------------
# 剔除不正常的GNSS坐标
condition_ = (gnss[:, 12] > 10.0) & (gnss[:, 13] > 10.0)
gnss = gnss[condition_, :]
# -------------------------


def YawChangeIdx(yaw):
    dyaw = np.abs(yaw[1:] - yaw[0:-1])
    return np.where(dyaw > 180.0)[0] + 1


def MakeYawContinous(yaw):
    MsfYawChange = YawChangeIdx(yaw)
    print(MsfYawChange)
    for idx in MsfYawChange:
        yaw[idx:] = yaw[idx:] - (yaw[idx] - yaw[idx - 1]) + (yaw[idx + 1] - yaw[idx])


MakeYawContinous(dr[:, 9])
MakeYawContinous(dr[:, 14])


dr_interp = InterpState(dr, dr[:, 1], loc[:, 1])
imu_interp = InterpState(imu, imu[:, 1], loc[:, 1])

gnss_interp = InterpState(gnss, gnss[:, 1], loc[:, 1])

print(dr_interp[0, 9], " ", dr_interp[0, 14])

# 调这个数值，可以改变初始点
IDX = 0

loc = loc[IDX:-1, :]
dr_interp = dr_interp[IDX:-1, :]


loc[loc[:, 18] > 180, 18] = loc[loc[:, 18] > 180, 18] - 360

pos_dr = dr_interp[:, 2:5]
[pos_loc_y, pos_loc_x] = GPStoXY(loc[:, 6], loc[:, 7], loc[0, 6], loc[0, 7])

[pos_gnss_y, pos_gnss_x] = GPStoXY(
    gnss_interp[:, 12], gnss_interp[:, 13], loc[0, 6], loc[0, 7]
)

pos_dr_x = -(pos_dr[:, 0] - pos_dr[0, 0] + pos_loc_x[0])
pos_dr_y = -(pos_dr[:, 1] - pos_dr[0, 1] + pos_loc_y[0])


pos_loc_z = loc[:, 30] - loc[0, 30]

pos_dr_z = pos_dr[:, 2] - pos_dr[0, 2] + pos_loc_z[0]

# ----------------

fit_size = 4
skip = 50

ref_x = np.zeros(fit_size)
ref_y = np.zeros(fit_size)

dr_fit_x = np.zeros(fit_size)
dr_fit_y = np.zeros(fit_size)

for i in range(fit_size):
    ref_x[i] = pos_loc_x[i * skip]
    ref_y[i] = pos_loc_y[i * skip]
    dr_fit_x[i] = pos_dr_x[i * skip]
    dr_fit_y[i] = pos_dr_y[i * skip]


def eqn(theta):
    x_ = np.zeros(fit_size)
    y_ = np.zeros(fit_size)
    for i in range(fit_size):
        x_[i], y_[i] = rot2d(dr_fit_x[i], dr_fit_y[i], theta)
    # 优化的目的是对准两个轨迹的航向，使用叉乘是对准航向最优的方式
    return np.sum(np.abs(x_ * ref_y - y_ * ref_x))  # 横向距离（垂足）
    # return np.sum(np.abs(np.abs(x_ - ref_x) + np.abs(y_ - ref_y)))  # 距离


v1 = np.array([dr_fit_x[1] - dr_fit_x[0], dr_fit_y[1] - dr_fit_y[0]])
v2 = np.array([ref_x[1] - ref_x[0], ref_y[1] - ref_y[0]])
mymin = minimize(eqn, vector_angle_2d_simple(v1, v2))

theta = mymin.x[0]

print(mymin)

# +++++++++++++

# 等间隔点对比
skip_ = 200
size_ = np.int64(np.ceil(pos_loc_x.size / skip_))
print(size_)
ref_x_ = np.zeros(size_)
ref_y_ = np.zeros(size_)
dr_x_ = np.zeros(size_)
dr_y_ = np.zeros(size_)
for i in range(size_):
    ref_x_[i] = pos_loc_x[i * skip_]
    ref_y_[i] = pos_loc_y[i * skip_]
    dr_x_[i] = pos_dr_x[i * skip_]
    dr_y_[i] = pos_dr_y[i * skip_]

[dr_rot_x_, dr_rot_y_] = rot2d(dr_x_, dr_y_, theta)


# ------------------------


wheel_scale_ = 1.0

[pos_dr_x_rot, pos_dr_y_rot] = rot2d(pos_dr_x, pos_dr_y, theta)

plt.rcParams["legend.loc"] = "upper left"

plt.figure("cmp")

# plt.plot(pos_loc_x, pos_loc_y)
# # plt.plot(pos_dr_x, pos_dr_y)
# plt.plot(pos_dr_x_rot, pos_dr_y_rot)


plt.plot(pos_loc_x, pos_loc_y)
# plt.scatter(pos_gnss_y, pos_gnss_x, s=0.1)
# plt.plot(pos_dr_x, pos_dr_y)
plt.plot(pos_dr_x_rot * wheel_scale_, pos_dr_y_rot * wheel_scale_)

[dr_fit_x_rot, dr_fit_y_rot] = rot2d(dr_fit_x, dr_fit_y, theta)
plt.scatter(ref_x, ref_y, s=10, c="yellow")
plt.scatter(dr_fit_x_rot, dr_fit_y_rot, s=10, c="green")
plt.scatter(ref_x_, ref_y_, s=10, c="blue")
plt.scatter(dr_rot_x_ * wheel_scale_, dr_rot_y_ * wheel_scale_, s=10, c="red")
plt.legend(["MSF", "DR"])

plt.gca().set_aspect("equal")
# plt.show()

# fig = plt.figure(figsize=(10, 8))  # 创建一个图形窗口
# ax = fig.add_subplot(111, projection="3d")  # 添加一个3D子图[2,4](@ref)
# ax.plot(pos_loc_y, pos_loc_x, pos_loc_z, label="loc")
# ax.plot(pos_dr_y_rot, pos_dr_x_rot, pos_dr_z, label="dr")
# ax.set_box_aspect([1, 1, 1])  # 三个方向的比例均为1:1:1
# plt.show()


MakeYawContinous(loc[:, 18])
MakeYawContinous(loc[:, 26])


SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(dr_interp[:, 1], dr_interp[:, 4] - dr_interp[0, 4])
plt.legend(["dr height"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(loc[:, 1], loc[:, 30] - loc[0, 30])
plt.legend(["msf height"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(loc[:, 1], loc[:, 30] - dr_interp[:, 4])
plt.legend(["delta height"])

SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(loc[:, 1], loc[:, 18])
plt.legend(["msf heading"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(dr_interp[:, 1], -dr_interp[:, 9])
plt.legend(["dr heading"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(dr_interp[:, 1], loc[:, 18] + dr_interp[:, 9])
plt.legend(["delta heading"])
# plt.show()

SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(loc[:, 1], loc[:, 24])
plt.legend(["msf pitch"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(dr_interp[:, 1], -dr_interp[:, 13])
plt.legend(["dr pitch"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(dr_interp[:, 1], loc[:, 24] + dr_interp[:, 13])
plt.legend(["delta pitch"])
# plt.show()

SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(loc[:, 1], loc[:, 25])
plt.legend(["msf roll"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(dr_interp[:, 1], dr_interp[:, 12])
plt.legend(["dr roll"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(dr_interp[:, 1], loc[:, 25] - dr_interp[:, 12])
plt.legend(["delta roll"])
# plt.show()

SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(loc[:, 1], loc[:, 27])
plt.legend(["BG_x"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(loc[:, 1], loc[:, 28])
plt.legend(["BG_y"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(loc[:, 1], loc[:, 29])
plt.legend(["BG_z"])


SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(loc[:, 1], loc[:, 26])
plt.legend(["msf yaw"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(dr_interp[:, 1], dr_interp[:, 14])
plt.legend(["dr yaw"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(dr_interp[:, 1], loc[:, 26] - dr_interp[:, 14])
plt.legend(["delta yaw"])
plt.show()

# SUBPLOTS_NUM = 3
# plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
# plt.subplot(SUBPLOTS_NUM, 1, 1)
# plt.plot(imu[2:-1, 8], imu[2:-1, 0] - imu[1:-2, 0])
# plt.subplot(SUBPLOTS_NUM, 1, 2)
# plt.plot(imu[:, 8], imu[:, 1])
# plt.subplot(SUBPLOTS_NUM, 1, 3)
# plt.plot(imu[2:-1, 8], imu[2:-1, 8] - imu[1:-2, 8])
# plt.show()
