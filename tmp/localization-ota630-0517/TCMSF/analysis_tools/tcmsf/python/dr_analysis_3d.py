import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


import numpy as np


def kabsch_algorithm(A, B):
    """
    使用Kabsch算法计算两组3D点之间的最优刚体变换（旋转R和平移t）
    使得 ||B - (R @ A + t)||^2 最小

    参数:
        A: 源点云，形状为(N, 3)的numpy数组
        B: 目标点云，形状为(N, 3)的numpy数组

    返回:
        R: 3x3旋转矩阵
        t: 3x1平移向量
        T: 4x4齐次变换矩阵
    """
    assert A.shape == B.shape, "点云A和B必须具有相同的形状"
    assert A.shape[1] == 3, "点云必须是三维坐标"

    # 计算质心
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # 去中心化
    A_centered = A - centroid_A
    B_centered = B - centroid_B

    # 计算协方差矩阵 H = A_centered.T @ B_centered
    H = A_centered.T @ B_centered

    # 对H进行奇异值分解(SVD)
    U, S, Vt = np.linalg.svd(H)

    # 计算旋转矩阵 R = V @ U.T
    R = Vt.T @ U.T

    # 处理反射情况（确保是纯旋转，行列式为1）
    if np.linalg.det(R) < 0:
        # 如果行列式为负，修正反射
        Vt[2, :] *= -1  # 将Vt的第三行乘以-1
        R = Vt.T @ U.T

    # 计算平移向量 t = centroid_B - R @ centroid_A
    t = centroid_B - R @ centroid_A

    # 构建4x4齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return R, t, T


def transform_points(points, T):
    """
    使用变换矩阵T变换点云

    参数:
        points: 要变换的点云，形状为(N, 3)
        T: 4x4变换矩阵

    返回:
        transformed_points: 变换后的点云，形状为(N, 3)
    """
    # 转换为齐次坐标
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))

    # 应用变换
    transformed_homogeneous = (T @ points_homogeneous.T).T

    # 转换回笛卡尔坐标
    transformed_points = transformed_homogeneous[:, :3]

    return transformed_points


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


dr = np.array(pd.read_csv("data/tcmsf/dr.csv"))
dr = np.array(pd.read_csv("data/tcmsf/dr_replay.csv"))
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


def scope_by_time(left_t, right_t, data):
    timestamp = data[:, 1]
    condition_ = (timestamp > left_t) & (timestamp < right_t)
    return data[condition_, :]


LEFT_T = 1761824900.881000042 - 1.0
RIGHT_T = 1761824960.963092327 - 10.0

loc = scope_by_time(LEFT_T, RIGHT_T, loc)
dr = scope_by_time(LEFT_T, RIGHT_T, dr)
imu = scope_by_time(LEFT_T, RIGHT_T, imu)
veh = scope_by_time(LEFT_T, RIGHT_T, veh)
gnss = scope_by_time(LEFT_T, RIGHT_T, gnss)


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

pos_dr = dr_interp[:, 2:5] - dr_interp[0, 2:5]
[pos_loc_y, pos_loc_x] = GPStoXY(loc[:, 4], loc[:, 5], loc[0, 4], loc[0, 5])

pos_loc_z = loc[:, 6] - loc[0, 6]

[pos_gnss_y, pos_gnss_x] = GPStoXY(
    gnss_interp[:, 12], gnss_interp[:, 13], loc[0, 4], loc[0, 5]
)

pos_dr_x = -(pos_dr[:, 0] - pos_dr[0, 0] + pos_loc_x[0])
pos_dr_y = -(pos_dr[:, 1] - pos_dr[0, 1] + pos_loc_y[0])
pos_dr_z = -(pos_dr[:, 2] - pos_dr[0, 2] + pos_loc_z[0])

# ----------------

fit_size = 20
skip = 50

ref_x = np.zeros(fit_size)
ref_y = np.zeros(fit_size)
ref_z = np.zeros(fit_size)

dr_fit_x = np.zeros(fit_size)
dr_fit_y = np.zeros(fit_size)
dr_fit_z = np.zeros(fit_size)

for i in range(fit_size):
    ref_x[i] = pos_loc_x[i * skip]
    ref_y[i] = pos_loc_y[i * skip]
    ref_z[i] = pos_loc_z[i * skip]
    dr_fit_x[i] = pos_dr_x[i * skip]
    dr_fit_y[i] = pos_dr_y[i * skip]
    dr_fit_z[i] = pos_dr_z[i * skip]

ref_pos_ = np.column_stack((ref_x, ref_y, ref_z))
dr_pos_ = np.column_stack((dr_fit_x, dr_fit_y, dr_fit_z))


# 使用Kabsch算法计算变换矩阵
R, t, T = kabsch_algorithm(dr_pos_, ref_pos_)

print("\n计算出的旋转矩阵R:")
print(R)
print("\n计算出的平移向量t:")
print(t)
print("\n完整的4x4变换矩阵T:")
print(T)

# +++++++++++++


dr_transformed = transform_points(pos_dr, T)

pos_dr_x_rot = dr_transformed[:, 0]
pos_dr_y_rot = dr_transformed[:, 1]


plt.rcParams["legend.loc"] = "upper left"

plt.figure("cmp")

# plt.plot(pos_loc_x, pos_loc_y)
# # plt.plot(pos_dr_x, pos_dr_y)
# plt.plot(pos_dr_x_rot, pos_dr_y_rot)


plt.plot(pos_loc_y, pos_loc_x)
plt.scatter(pos_gnss_y, pos_gnss_x, s=0.1)
# plt.plot(pos_dr_x, pos_dr_y)
plt.plot(-pos_dr_y_rot, -pos_dr_x_rot)

dr_pos_transformed = transform_points(dr_pos_, T)
dr_fit_x_rot = dr_pos_transformed[:, 0]
dr_fit_y_rot = dr_pos_transformed[:, 1]

plt.scatter(ref_y, ref_x, s=10, c="blue")
plt.scatter(dr_fit_y_rot, dr_fit_x_rot, s=10, c="red")
plt.legend(["MSF", "GNSS", "DR"])

plt.gca().set_aspect("equal")
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
