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


msf_imu = np.array(pd.read_csv("data/tmp/msf_state.csv"))
msf_veh = np.array(pd.read_csv("data/tmp/msf.csv"))
gnss = np.array(pd.read_csv("data/tmp/gnss.csv"))
ins = np.array(pd.read_csv("modules/localization/src/TCMSF/test/msf_test/data/ins.csv"))

gnss = gnss[gnss[:, 1] != 0.0, :]

[ins_x, ins_y] = InterpPos(
    ins[:, 1], ins[:, 2], msf_imu[0, 1], msf_imu[0, 2], ins[:, 0], gnss[:, 0]
)
[msf_imu_x, msf_imu_y] = InterpPos(
    msf_imu[:, 1],
    msf_imu[:, 2],
    msf_imu[0, 1],
    msf_imu[0, 2],
    msf_imu[:, 0],
    gnss[:, 0],
)

[gnss_y, gnss_x] = GPStoXY(gnss[:, 1], gnss[:, 2], msf_imu[0, 1], msf_imu[0, 2])

plt.figure("traj")
# plt.plot(gnss_x, gnss_y, c="orange")
plt.scatter(gnss_x, gnss_y, c="orange", s=10)

plt.plot(ins_x, ins_y, c="red")
plt.scatter(ins_x, ins_y, c="red", s=10)

plt.scatter(msf_imu_x, msf_imu_y, c="green", s=10)
plt.plot(msf_imu_x, msf_imu_y, c="green")

for i in range(0, gnss[:, 0].shape[0], 100):
    plt.annotate(gnss[i, 0], xy=(msf_imu_x[i], msf_imu_y[i]))

plt.gca().set_aspect("equal")

ins_state_interp = InterpState(ins[:, 4:10], ins[:, 0], gnss[:, 0])
msf_imu_state_interp = InterpState(msf_imu[:, 4:11], msf_imu[:, 0], gnss[:, 0])
msf_veh_state_interp = InterpState(msf_veh[:, 4:14], msf_veh[:, 0], gnss[:, 0])
msf_veh_state_interp[:, 4:7] = msf_veh_state_interp[:, 4:7] * 180 / np.pi

# plt.figure("veh att")
# plt.plot(msf_veh_state_interp[:, 4])
# plt.plot(msf_veh_state_interp[:, 5])
# plt.plot(msf_veh_state_interp[:, 6])
# plt.plot(msf_imu_state_interp[:, 0])
# plt.plot(msf_imu_state_interp[:, 1])
# plt.plot(msf_imu_state_interp[:, 2])
# plt.show()

# msf_imu_vel_norm = np.linalg.norm(msf_imu_state_interp[:, 3:6], axis=1)

msf_imu_state_std_interp = InterpState(msf_imu[:, 19:40], msf_imu[:, 0], gnss[:, 0])


plt.subplots(5, 1, sharex=True)
plt.subplot(5, 1, 1)
# 因为MSF输出的坐标系定义与PBOX输出的不一致，所以欧拉角会有一些偏差
pitch_mean_veh = (msf_veh_state_interp[:, 4] - ins_state_interp[:, 3] * 180 / np.pi)[
    8000:-1:100
].mean()
plt.text(gnss[100, 0], 1.5, "veh pitch steady state: " + str(pitch_mean_veh))

pitch_mean = (msf_imu_state_interp[:, 0] - ins_state_interp[:, 3] * 180 / np.pi)[
    8000:-1:100
].mean()
plt.text(gnss[100, 0], 1, "pitch steady state: " + str(pitch_mean))
plt.plot(
    gnss[:, 0],
    msf_imu_state_interp[:, 0] - ins_state_interp[:, 3] * 180 / np.pi,
)
plt.plot(
    gnss[:, 0],
    msf_veh_state_interp[:, 4] - ins_state_interp[:, 3] * 180 / np.pi,
)
plt.plot(gnss[:, 0], msf_imu_state_std_interp[:, 0] * 180 / np.pi, c="black")
plt.plot(gnss[:, 0], -msf_imu_state_std_interp[:, 0] * 180 / np.pi, c="black")
plt.ylim([-2, 2])

plt.subplot(5, 1, 2)
roll_mean_veh = (msf_veh_state_interp[:, 5] - ins_state_interp[:, 4] * 180 / np.pi)[
    8000:-1:100
].mean()
plt.text(gnss[100, 0], 1.5, "veh roll steady state: " + str(roll_mean_veh))

roll_mean = (msf_imu_state_interp[:, 1] - ins_state_interp[:, 4] * 180 / np.pi)[
    8000:-1:100
].mean()
plt.text(gnss[100, 0], 1, "roll steady state: " + str(roll_mean))
plt.plot(
    gnss[:, 0],
    msf_imu_state_interp[:, 1] - ins_state_interp[:, 4] * 180 / np.pi,
)
plt.plot(gnss[:, 0], msf_imu_state_std_interp[:, 1] * 180 / np.pi, c="black")
plt.plot(gnss[:, 0], -msf_imu_state_std_interp[:, 1] * 180 / np.pi, c="black")
plt.ylim([-2, 2])

plt.subplot(5, 1, 3)
# MSF（车体）的航向与PBOX的航向基本一致。
yaw_mean_veh = (msf_veh_state_interp[:, 6] - ins_state_interp[:, 5] * 180 / np.pi)[
    8000:-1:100
].mean()
plt.text(gnss[100, 0], 1.5, "veh yaw steady state: " + str(yaw_mean_veh))

yaw_mean = (msf_imu_state_interp[:, 2] - ins_state_interp[:, 5] * 180 / np.pi)[
    8000:-1:100
].mean()
plt.text(gnss[100, 0], 1, "yaw steady state: " + str(yaw_mean))
# plt.plot(gnss[:, 0], msf_imu_vel_norm)
plt.ylim([-2, 2])
dyaw = msf_imu_state_interp[:, 2] - ins_state_interp[:, 5] * 180 / np.pi
dyaw[dyaw > 180] -= 360
dyaw[dyaw < -180] += 360
dyaw[dyaw > 10] = 0


dyaw_veh = msf_veh_state_interp[:, 6] - ins_state_interp[:, 5] * 180 / np.pi
dyaw_veh[dyaw_veh > 180] -= 360
dyaw_veh[dyaw_veh < -180] += 360
dyaw_veh[dyaw_veh > 10] = 0

plt.plot(gnss[:, 0], dyaw)
plt.plot(gnss[:, 0], dyaw_veh)
plt.plot(gnss[:, 0], msf_imu_state_std_interp[:, 2] * 180 / np.pi, c="black")
plt.plot(gnss[:, 0], -msf_imu_state_std_interp[:, 2] * 180 / np.pi, c="black")


plt.subplot(5, 1, 4)
plt.plot(gnss[:, 0], msf_veh_state_interp[:, 7] - ins_state_interp[:, 0])
plt.plot(gnss[:, 0], msf_veh_state_interp[:, 8] - ins_state_interp[:, 1])
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 3] - ins_state_interp[:, 0])
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 4] - ins_state_interp[:, 1])
# plt.plot(gnss[:, 0], msf_veh_state_interp[:, 7] - msf_imu_state_interp[:, 3])
# plt.plot(gnss[:, 0], msf_veh_state_interp[:, 8] - msf_imu_state_interp[:, 4])
plt.plot(gnss[:, 0], msf_imu_state_std_interp[:, 3], c="black")
plt.plot(gnss[:, 0], -msf_imu_state_std_interp[:, 4], c="black")
plt.ylim([-0.3, 0.3])
plt.subplot(5, 1, 5)
plt.plot(gnss[:, 0], msf_imu_x - ins_x)
plt.plot(gnss[:, 0], msf_imu_y - ins_y)
plt.plot(gnss[:, 0], msf_imu_state_std_interp[:, 6], c="black")
plt.plot(gnss[:, 0], -msf_imu_state_std_interp[:, 6], c="black")
plt.ylim([-40.0, 40.0])
plt.show()
