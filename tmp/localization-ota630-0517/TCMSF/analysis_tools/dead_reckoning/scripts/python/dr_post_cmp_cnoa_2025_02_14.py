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


# 读取DR的轨迹
dr = np.array(pd.read_csv("data/parsed/dr.csv"))
# 读取后处理的轨迹
post = np.array(
    pd.read_csv("data/post_results/post_pva_result_imu_frd_to_ned.txt", delimiter=" ")
)

# 调这个数值，可以改变初始点
IDX = 700

post = post[IDX:-1, :]

# 线性插值
dr_interp = InterpState(dr, dr[:, 1], post[:, 0])

# 将经纬度映射到平面上
# 以初始的经纬度为原点
[pos_post_y, pos_post_x] = GPStoXY(post[:, 1], post[:, 2], post[0, 1], post[0, 2])


# dr_interp = dr_interp[IDX:-1, :]
pos_dr = dr_interp[:, 2:4]

# 把DR的轨迹起点映射到后处理真值的起点上
pos_dr_x = -(pos_dr[:, 0] - pos_dr[0, 0] + pos_post_x[0])
pos_dr_y = -(pos_dr[:, 1] - pos_dr[0, 1] + pos_post_y[0])

# ----------------
# 因为DR是只保证相对精度的，并没有确定的坐标系
# 所以，如果要考查DR的精度，第一步就是要对准DR和参考真值的初始状态
# 此处对准DR轨迹和后处理真值轨迹的初始状态
# 对准的方式：通过优化的方式计算初始状态的航向角差异

# 选两个点对准
# 因为DR是100Hz，这里差不多选了两个相差30米的点，来对准初态
fit_size = 5
skip = 50

last_point_timestamp = dr_interp[fit_size * skip, 1]

ref_x = np.zeros(fit_size)
ref_y = np.zeros(fit_size)

dr_fit_x = np.zeros(fit_size)
dr_fit_y = np.zeros(fit_size)

for i in range(fit_size):
    ref_x[i] = pos_post_x[i * skip]
    ref_y[i] = pos_post_y[i * skip]
    dr_fit_x[i] = pos_dr_x[i * skip]
    dr_fit_y[i] = pos_dr_y[i * skip]


def eqn(theta):
    x_ = np.zeros(fit_size)
    y_ = np.zeros(fit_size)
    for i in range(fit_size):
        x_[i], y_[i] = rot2d(dr_fit_x[i], dr_fit_y[i], theta)
    # 优化的目的是对准两个轨迹的航向，使用叉乘是对准航向最优的方式
    # return np.sum(np.abs(x_ * ref_y - y_ * ref_x))  # 横向距离（垂足）
    return np.sum(np.abs(np.abs(x_ - ref_x) + np.abs(y_ - ref_y)))  # 横向距离（垂足）


# 这个地方初值可以设为0或者pi，因为投影点优化的时候，0和pi可能是等效的。为了使优化的结果是实际最优解，可以考虑调整初值
mymin = minimize(eqn, np.pi)

theta = mymin.x[0]

print(mymin)

# +++++++++++++

# 使用航向，将DR轨迹的初始状态与真值对齐
[pos_dr_x_rot, pos_dr_y_rot] = rot2d(pos_dr_x, pos_dr_y, theta)

[dr_fit_x_rot, dr_fit_y_rot] = rot2d(dr_fit_x, dr_fit_y, theta)

# #
# NUM = pos_dr_x_rot.size()
# dis = np.zeros(NUM, 1)
# for i in range(NUM - 1):
#     np.sqrt(
#         np.power(pos_dr_x_rot[i + 1] - pos_dr_x_rot[i], 2)
#         + np.power(pos_dr_y_rot[i + 1] - pos_dr_y_rot[i], 2)
#     )


# 绘图

plt.rcParams["legend.loc"] = "upper left"

plt.figure("cmp")

plt.plot(pos_post_x, pos_post_y)
plt.plot(pos_dr_x_rot, pos_dr_y_rot)
plt.scatter(ref_x, ref_y, s=10, c="blue")
plt.scatter(dr_fit_x_rot, dr_fit_y_rot, s=10, c="red")
plt.text(
    dr_fit_x_rot[-1], dr_fit_y_rot[-1], "timestamp " + "%14.4f" % last_point_timestamp
)
plt.legend(["localization POST", "DR", "align point POST", "align point DR"])

plt.gca().set_aspect("equal")
# plt.show()

post[post[:, 9] > 180, 9] = post[post[:, 9] > 180, 9] - 360

# 看下航向情况
plt.subplots(3, 1, sharex=True)
plt.subplot(3, 1, 1)
plt.plot(dr_interp[:, 0], dr_interp[:, 9])
plt.legend(["DR heading"])
plt.subplot(3, 1, 2)
plt.plot(post[:, 0], post[:, 9])
plt.legend(["post heading"])
plt.subplot(3, 1, 3)
plt.plot(dr_interp[:, 0], dr_interp[:, 9] + post[:, 9])
plt.legend(["delta heading, dr - post"])
plt.show()
