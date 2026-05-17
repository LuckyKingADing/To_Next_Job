import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.gridspec as gridspec


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


def CumulativeIntegral(timestamps, data):
    # 计算每个小区间的梯形面积
    # 差分计算相邻时间点的间隔
    dt = np.diff(timestamps)
    # 计算相邻状态值的平均值
    avg_values = (data[:-1] + data[1:]) / 2
    # 计算每个微小时间区间内的积分量（面积）
    incremental_areas = avg_values * dt

    # 计算累积积分（从起始时间到当前时间的总积分）
    # 第一个点的累积积分为0，因为从它自身开始还没有面积
    cumulative_integral = np.cumsum(incremental_areas)
    # 在积分结果数组前补0，使其长度与原始时间戳、状态值数组一致
    cumulative_integral = np.insert(cumulative_integral, 0, 0)

    return cumulative_integral


def point_to_line_segment_distance(point, line_start, line_end):
    """
    计算点到线段的垂直投影距离

    参数:
    point: 目标点 [x, y]
    line_start: 线段起点 [x, y]
    line_end: 线段终点 [x, y]

    返回:
    distance: 垂直投影距离
    projection_point: 投影点坐标
    """
    # 将点转换为向量
    line_vec = line_end - line_start
    point_vec = point - line_start

    # 计算线段长度的平方
    line_len_squared = np.dot(line_vec, line_vec)

    # 避免除零错误（如果线段起点终点重合）
    if line_len_squared == 0:
        return np.linalg.norm(point - line_start), line_start

    # 计算投影比例 t = (AP · AB) / (AB · AB)
    t = np.dot(point_vec, line_vec) / line_len_squared

    # 将投影点限制在线段范围内
    t = np.clip(t, 0, 1)

    # 计算投影点坐标
    projection_point = line_start + t * line_vec

    # 计算距离
    distance = np.linalg.norm(point - projection_point)

    return distance, projection_point


def curve_to_curve_projection_distance(curve_a, curve_b, window_size=1):
    """
    计算曲线A上各点到曲线B上相同时戳附近线段投影距离

    参数:
    curve_a: 曲线A的点序列，形状为(n, 2)
    curve_b: 曲线B的点序列，形状为(n, 2)
    window_size: 搜索窗口大小，决定使用附近多少个点构造线段

    返回:
    distances: 投影距离数组，形状为(n,)
    projection_points: 投影点坐标数组，形状为(n, 2)
    """
    n_points = curve_a.shape[0]
    distances = np.zeros(n_points)
    projection_points = np.zeros((n_points, 2))

    for i in range(n_points):
        # 确定曲线B上的线段端点索引
        start_idx = max(0, i - window_size)
        end_idx = min(n_points - 1, i + window_size)

        # 避免起点终点相同
        if start_idx == end_idx:
            if end_idx < n_points - 1:
                end_idx += 1
            else:
                start_idx -= 1

        # 获取线段端点
        line_start = curve_b[start_idx]
        line_end = curve_b[end_idx]

        # 计算点到线段的投影距离
        dist, proj_pt = point_to_line_segment_distance(curve_a[i], line_start, line_end)

        distances[i] = dist
        projection_points[i] = proj_pt

    return distances, projection_points


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


MSF_REPLAY = False
DR_REPLAY = False
DATA_PATH_PREFIX = "data/tcmsf/"

dr = np.array(pd.read_csv(DATA_PATH_PREFIX + "dr.csv"))
loc = np.array(pd.read_csv(DATA_PATH_PREFIX + "tcmsf.csv"))
imu = np.array(pd.read_csv(DATA_PATH_PREFIX + "imu.csv"))
veh = np.array(pd.read_csv(DATA_PATH_PREFIX + "vehicle.csv"))
gnss = np.array(pd.read_csv(DATA_PATH_PREFIX + "gnss_02.csv"))

if DR_REPLAY:
    dr = np.array(pd.read_csv(DATA_PATH_PREFIX + "dr_replay.csv"))

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

min_timestamp = min_timestamp + 2.0

loc_replay = []
loc_replay_interp = []
pos_loc_replay_interp_x = []
pos_loc_replay_interp_y = []
if MSF_REPLAY:
    loc_replay = np.array(pd.read_csv(DATA_PATH_PREFIX + "replay/tcmsf.csv"))
    if loc_replay[0, 1] > min_timestamp:
        min_timestamp = loc_replay[0, 1]
        loc_replay = loc_replay[loc_replay[:, 1] > min_timestamp, :]

print(min_timestamp)

loc = loc[loc[:, 1] > min_timestamp, :]
dr = dr[dr[:, 1] > min_timestamp, :]
imu = imu[imu[:, 1] > min_timestamp, :]
veh = veh[veh[:, 1] > min_timestamp, :]
gnss = gnss[gnss[:, 1] > min_timestamp, :]

if MSF_REPLAY:
    loc_replay_interp = InterpState(loc_replay, loc_replay[:, 1], loc[:, 1])
    [pos_loc_replay_interp_y, pos_loc_replay_interp_x] = GPStoXY(
        loc_replay_interp[:, 6],
        loc_replay_interp[:, 7],
        loc_replay_interp[0, 6],
        loc_replay_interp[0, 7],
    )

# -------------------------
# -------------------------
# 剔除不正常的GNSS坐标
condition_ = (gnss[:, 2] > 10.0) & (gnss[:, 3] > 10.0)
gnss = gnss[condition_, :]
# -------------------------


def YawChangeIdx(yaw):
    dyaw = np.abs(yaw[1:] - yaw[0:-1])
    return np.where(dyaw > 180.0)[0] + 1


def MakeYawContinous(yaw):
    MsfYawChange = YawChangeIdx(yaw)
    for idx in MsfYawChange:
        yaw[idx:] = yaw[idx:] - (yaw[idx] - yaw[idx - 1]) + (yaw[idx + 1] - yaw[idx])


MakeYawContinous(dr[:, 9])
MakeYawContinous(dr[:, 14])


dr_interp = InterpState(dr, dr[:, 1], loc[:, 1])
imu_interp = InterpState(imu, imu[:, 1], loc[:, 1])

loc_interp = InterpState(loc, loc[:, 1], gnss[:, 1])

veh_interp = InterpState(veh, veh[:, 1], imu[:, 1])
dr_interp_imu = InterpState(dr, dr[:, 1], imu[:, 1])


loc[loc[:, 18] > 180, 18] = loc[loc[:, 18] > 180, 18] - 360

pos_dr = dr_interp[:, 2:5]
[pos_loc_y, pos_loc_x] = GPStoXY(loc[:, 6], loc[:, 7], loc[0, 6], loc[0, 7])


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
    return np.sum(
        np.abs(x_ * ref_y - y_ * ref_x)
        # + 0.1 * (np.abs(np.abs(x_ - ref_x) + np.abs(y_ - ref_y)))
    )  # 横向距离（垂足）


# 使用前两个点，计算一个航向初值
v1 = np.array([dr_fit_x[1] - dr_fit_x[0], dr_fit_y[1] - dr_fit_y[0]])
v2 = np.array([ref_x[1] - ref_x[0], ref_y[1] - ref_y[0]])
mymin = minimize(eqn, vector_angle_2d_simple(v1, v2))

theta = mymin.x[0]


# +++++++++++++

# 等间隔点对比
skip_ = 200
size_ = np.int64(np.ceil(pos_loc_x.size / skip_))
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


[pos_gnss_y, pos_gnss_x] = GPStoXY(gnss[:, 2], gnss[:, 3], loc[0, 6], loc[0, 7])

[pos_loc_interp_y, pos_loc_interp_x] = GPStoXY(
    loc_interp[:, 6], loc_interp[:, 7], loc_interp[0, 6], loc_interp[0, 7]
)

# ======================================

plt.rcParams["legend.loc"] = "upper left"

# ----------------------------------------------
fig = plt.figure("Sensor Info", figsize=(16, 8))
gs = gridspec.GridSpec(4, 2, width_ratios=[1, 1], height_ratios=[1, 1, 1, 1])

# 左1
ax_left1 = fig.add_subplot(gs[0, 0])
ax_left1.plot(imu[:, 1], imu[:, 2])
ax_left1.plot(imu[:, 1], imu[:, 3])
ax_left1.legend(["acc_x (g)", "acc_y (g)"])

# 左2
ax_left3 = fig.add_subplot(gs[1, 0])
ax_left3.plot(imu[:, 1], imu[:, 4])
ax_left3.legend(["acc_z (g)"])

# 左3
ax_left1 = fig.add_subplot(gs[2, 0])
ax_left1.plot(imu[:, 1], imu[:, 5])
ax_left1.plot(imu[:, 1], imu[:, 6])
ax_left1.legend(["gyro_x (deg/s)", "gyro_y (deg/s)"])

# 左4
ax_left3 = fig.add_subplot(gs[3, 0])
ax_left3.plot(imu[:, 1], imu[:, 7])
ax_left3.legend(["gyro_z (deg/s)"])

# 右1
ax_right1 = fig.add_subplot(gs[0, 1])
ax_right1.plot(veh[:, 1], veh[:, 2])
ax_right1.plot(veh[:, 1], veh[:, 3])
ax_right1.legend(["spd_rl(m/s)", "spd_rr(m/s)"])

# 右2
ax_right2 = fig.add_subplot(gs[1, 1])
ax_right2.plot(veh[:, 1], veh[:, 4])
ax_right2.legend(["yaw_rate(deg/s)"])

# 右3
ax_right2 = fig.add_subplot(gs[2, 1])
ax_right2.plot(veh_interp[:, 1], veh_interp[:, 4] - imu[:, 7])
ax_right2.legend(["yaw_rate_diff(deg/s)"])

# 右4
ax_right2 = fig.add_subplot(gs[3, 1])
ax_right2.plot(veh[:, 1], veh[:, 4])
ax_right2.plot(imu[:, 1], imu[:, 7])
ax_right2.legend(["chassis_yaw_rate", "imu_gyro_z"])

# ---------------------------------------
fig = plt.figure("MSF vs DR", figsize=(24, 8))

# 创建3x3的网格，定义复杂的宽度比例
gs = gridspec.GridSpec(4, 3, width_ratios=[1, 1, 1], height_ratios=[1, 1, 1, 1])

# 左侧大图（占据两行一列）
ax_main = fig.add_subplot(gs[:, 0])
ax_main.plot(pos_loc_x, pos_loc_y)
ax_main.plot(pos_dr_x_rot * wheel_scale_, pos_dr_y_rot * wheel_scale_)

[dr_fit_x_rot, dr_fit_y_rot] = rot2d(dr_fit_x, dr_fit_y, theta)
ax_main.scatter(ref_x, ref_y, s=10, c="yellow")
ax_main.scatter(dr_fit_x_rot, dr_fit_y_rot, s=10, c="green")
ax_main.scatter(ref_x_, ref_y_, s=10, c="blue")
ax_main.scatter(dr_rot_x_ * wheel_scale_, dr_rot_y_ * wheel_scale_, s=10, c="red")
ax_main.legend(["MSF", "DR"])
plt.gca().set_aspect("equal")
ax_main.set_box_aspect(1)

# 中第一个子图
ax_middle1 = fig.add_subplot(gs[0, 1])
ax_middle1.plot(dr_interp[:, 1], loc[:, 24] + dr_interp[:, 13])
ax_middle1.plot(dr_interp[:, 1], loc[:, 24])
ax_middle1.plot(dr_interp[:, 1], -dr_interp[:, 13])
ax_middle1.axhline(y=loc[0, 24] + dr_interp[0, 13] + 0.3, ls="--", color="black")
ax_middle1.axhline(y=loc[0, 24] + dr_interp[0, 13] - 0.3, ls="--", color="black")
ax_middle1.legend(["delta pitch(deg)", "pitch loc", "pitch dr"])

# 中第二个子图
ax_middle2 = fig.add_subplot(gs[1, 1])
ax_middle2.plot(dr_interp[:, 1], loc[:, 25] - dr_interp[:, 12])
ax_middle2.plot(dr_interp[:, 1], loc[:, 25])
ax_middle2.plot(dr_interp[:, 1], dr_interp[:, 12])
ax_middle2.axhline(y=loc[0, 25] - dr_interp[0, 12] + 0.3, ls="--", color="black")
ax_middle2.axhline(y=loc[0, 25] - dr_interp[0, 12] - 0.3, ls="--", color="black")
ax_middle2.legend(["delta roll(deg)", "roll loc", "roll dr"])

# 中第三个子图
ax_middle3 = fig.add_subplot(gs[2, 1])
ax_middle3.plot(dr_interp[:, 1], loc[:, 26] - dr_interp[:, 14])
ax_middle3.axhline(y=loc[0, 26] - dr_interp[0, 14] + 0.3, ls="--", color="black")
ax_middle3.axhline(y=loc[0, 26] - dr_interp[0, 14] - 0.3, ls="--", color="black")
ax_middle3.legend(["delta yaw(deg)"])

# 中第四个子图
ax_middle4 = fig.add_subplot(gs[3, 1])
ax_middle4.plot(loc[:, 1], loc[:, 30] - dr_interp[:, 4])
ax_middle4.legend(["delta height"])

# 右侧第一个子图
ax_right1 = fig.add_subplot(gs[0, 2])
ax_right1.plot(loc_interp[:, 1], loc_interp[:, 31])
ax_right1.legend(["msf ego vel lat (m/s)"])


# 右侧第二个子图
ax_right2 = fig.add_subplot(gs[1, 2])
ax_right2.plot(
    loc_interp[:, 1], np.abs(CumulativeIntegral(loc_interp[:, 1], loc_interp[:, 31]))
)
dis_, _ = curve_to_curve_projection_distance(
    np.column_stack((pos_loc_x, pos_loc_y)),
    np.column_stack((pos_dr_x_rot, pos_dr_y_rot)),
    5,
)
ax_right2.plot(loc[:, 1], np.abs(dis_))
ax_right2.legend(["msf ego dp lat (m)", "(dr - msf) lat (m)"])

# 右侧第三个子图
ax_right3 = fig.add_subplot(gs[2, 2])
slip_angle_ = np.rad2deg(np.arctan2(loc_interp[:, 31], loc_interp[:, 32]))
ax_right3.plot(loc_interp[:, 1], slip_angle_)
ax_right3.legend(["slip angle (deg)"])

# 右侧第四个子图
ax_right4 = fig.add_subplot(gs[3, 2])
slip_idx_ = (veh_interp[:, 2] + veh_interp[:, 3]) / 2.0 * np.deg2rad(veh_interp[:, 4])
gravity_lat_ = np.sin(np.deg2rad(dr_interp_imu[:, 12])) * 9.8
ax_right4.plot(veh_interp[:, 1], slip_idx_)
ax_right4.plot(dr_interp_imu[:, 1], gravity_lat_)
ax_right4.plot(dr_interp_imu[:, 1], slip_idx_ + gravity_lat_)
ax_right4.plot(loc_interp[:, 1], slip_angle_ * 3.2)
ax_right4.legend(
    ["slip index", "gravity lat", "slip_index - gravity", "slip_angle_*scale"]
)

plt.tight_layout()

# --------------------------------------------------------------


fig = plt.figure("MSF vs GNSS", figsize=(24, 8))

# 创建3x3的网格，定义复杂的宽度比例
gs = gridspec.GridSpec(5, 3, width_ratios=[1, 1, 1], height_ratios=[1, 1, 1, 1, 1])

# 左侧大图（占据两行一列）
ax_main = fig.add_subplot(gs[:, 0])
ax_main.plot(pos_loc_x, pos_loc_y)
ax_main.plot(pos_gnss_x, pos_gnss_y)
ax_main.scatter(
    pos_dr_x_rot[::20] * wheel_scale_,
    pos_dr_y_rot[::20] * wheel_scale_,
    s=0.2,
    c="gray",
)
ax_main.scatter(pos_loc_interp_x[::10], pos_loc_interp_y[::10], s=5, c="blue")
ax_main.scatter(pos_gnss_x[::10], pos_gnss_y[::10], s=5, c="red")

ax_main.legend(["MSF", "GNSS", "DR"])
plt.gca().set_aspect("equal")
ax_main.set_box_aspect(1)

# 中第一个子图
ax_middle1 = fig.add_subplot(gs[0, 1])
ax_middle1.plot(loc_interp[:, 1], loc_interp[:, 20] - gnss[:, 5])
ax_middle1.legend(["delta vel_E(m/s)"])

# 中第二个子图
ax_middle2 = fig.add_subplot(gs[1, 1])
ax_middle2.plot(loc_interp[:, 1], loc_interp[:, 21] - gnss[:, 6])
ax_middle2.legend(["delta vel_N(m/s)"])

# 中第三个子图
ax_middle3 = fig.add_subplot(gs[2, 1])
ax_middle3.plot(loc_interp[:, 1], loc_interp[:, 22] - gnss[:, 7])
ax_middle3.legend(["delta vel_U(m/s)"])

# 中第四个子图
ax_middle4 = fig.add_subplot(gs[3, 1])
ax_middle4.plot(loc_interp[:, 1], loc_interp[:, 18] - gnss[:, 8])
ax_middle4.legend(["delta heading(deg)"])

# 中第五个子图
ax_middle4 = fig.add_subplot(gs[4, 1])
ax_middle4.plot(loc_interp[:, 1], loc_interp[:, 31])
ax_middle4.legend(["ego vel lat"])


# 右第一个子图
ax_right1 = fig.add_subplot(gs[0, 2])
ax_right1.plot(loc[:, 1], loc[:, 27])
ax_right1.plot(loc[:, 1], loc[:, 28])
ax_right1.plot(loc[:, 1], loc[:, 29])
ax_right1.legend(["bg_x(deg/s)", "bg_y(deg/s)", "bg_z(deg/s)"])

# 右第二个子图
ax_right2 = fig.add_subplot(gs[1, 2])
ax_right2.plot(loc[:, 1], loc[:, 34])
ax_right2.plot(loc[:, 1], loc[:, 35])
ax_right2.plot(loc[:, 1], loc[:, 36])
ax_right2.legend(["ba_x(m/s^2)", "ba_y(m/s^2)", "ba_z(m/s^2)"])

# 右第三个子图
ax_right3 = fig.add_subplot(gs[2, 2])
ax_right3.plot(loc[:, 1], loc[:, 37])
ax_right3.plot(loc[:, 1], loc[:, 39])
ax_right3.legend(["imu pitch bias(deg)", "imu yaw bias(deg)"])

# 右第四个子图
ax_right4 = fig.add_subplot(gs[3, 2])
ax_right4.plot(loc[:, 1], loc[:, 38])
ax_right4.legend(["wheel bias"])

# 右第五个子图
ax_right5 = fig.add_subplot(gs[4, 2])
ax_right5.plot(loc[:, 1], loc[:, 16])
ax_right5.legend(["align status"])

plt.tight_layout()

# --------------------------------------------------------

if MSF_REPLAY:
    fig = plt.figure("MSF vs MSF_Replay", figsize=(24, 8))

    # 创建3x3的网格，定义复杂的宽度比例
    gs = gridspec.GridSpec(5, 3, width_ratios=[1, 1, 1], height_ratios=[1, 1, 1, 1, 1])

    # 左侧大图（占据两行一列）
    ax_main = fig.add_subplot(gs[:, 0])
    ax_main.plot(pos_loc_replay_interp_x, pos_loc_replay_interp_y)
    ax_main.plot(pos_loc_x, pos_loc_y)
    ax_main.plot(pos_dr_x_rot, pos_dr_y_rot)
    ax_main.scatter(
        pos_dr_x_rot[::20] * wheel_scale_,
        pos_dr_y_rot[::20] * wheel_scale_,
        s=0.2,
        c="gray",
    )

    ax_main.legend(["MSF_Replay", "MSF", "DR"])
    plt.gca().set_aspect("equal")
    ax_main.set_box_aspect(1)

    # 中1
    ax_mid1 = fig.add_subplot(gs[0, 1])
    ax_mid1.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 24])
    ax_mid1.plot(loc[:, 1], loc[:, 24])
    ax_mid1.legend(["msf_replay_pitch", "msf_pitch"])

    # 中2
    ax_mid2 = fig.add_subplot(gs[1, 1])
    ax_mid2.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 25])
    ax_mid2.plot(loc[:, 1], loc[:, 25])
    ax_mid2.legend(["msf_replay_roll", "msf_roll"])

    # 中3
    ax_mid3 = fig.add_subplot(gs[2, 1])
    ax_mid3.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 26])
    ax_mid3.plot(loc[:, 1], loc[:, 26])
    ax_mid3.legend(["msf_replay_yaw", "msf_yaw"])

    # 中4
    ax_mid4 = fig.add_subplot(gs[3, 1])
    ax_mid4.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 30])
    ax_mid4.plot(loc[:, 1], loc[:, 30])
    ax_mid4.legend(["msf_replay_height", "msf_height"])

    # 中5
    ax_mid5 = fig.add_subplot(gs[4, 1])
    ax_mid5.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 31])
    ax_mid5.plot(loc[:, 1], loc[:, 31])
    ax_mid5.legend(["msf_replay_vel_R", "msf_vel_R"])

    # 右1
    ax_right1 = fig.add_subplot(gs[0, 2])
    ax_right1.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 24] - loc[:, 24])
    ax_right1.legend(["msf_pitch diff"])

    # 右2
    ax_right2 = fig.add_subplot(gs[1, 2])
    ax_right2.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 25] - loc[:, 25])
    ax_right2.legend(["msf_roll diff"])

    # 右3
    ax_right3 = fig.add_subplot(gs[2, 2])
    ax_right3.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 26] - loc[:, 26])
    ax_right3.legend(["msf_yaw diff"])

    # 右4
    ax_right4 = fig.add_subplot(gs[3, 2])
    ax_right4.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 30] - loc[:, 30])
    ax_right4.legend(["msf_height diff"])

    # 右5
    ax_right5 = fig.add_subplot(gs[4, 2])
    ax_right5.plot(loc_replay_interp[:, 1], loc_replay_interp[:, 31] - loc[:, 31])
    ax_right5.legend(["msf_vel_R diff"])

plt.show()
