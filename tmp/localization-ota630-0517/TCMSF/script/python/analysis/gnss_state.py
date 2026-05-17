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


alg_imu = np.array(pd.read_csv("data/tmp/alg_debug_state.csv"))
msf_veh = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)
gnss = np.array(pd.read_csv("data/tmp/gps_debug_state.csv"), dtype=float)
veh_state = np.array(pd.read_csv("data/tmp/veh_debug_state.csv"))
imu_state = np.array(pd.read_csv("data/tmp/imu_debug_state.csv"))
kcp_state = np.array(pd.read_csv("data/tmp/kcp_debug_state.csv"))

# plt.figure("time")
# plt.scatter(msf_veh[2:-1, 0], msf_veh[2:-1, 0] - msf_veh[1:-2, 0], s=3)
# plt.legend(["msf dt"])
# plt.show()


gnss_std = np.array(pd.read_csv("data/tmp/gps_std.csv"), dtype=float)

# vis_state = np.array(pd.read_csv("data/tmp/vis_debug_state.csv"))

msf_veh_interp = InterpState(msf_veh, msf_veh[:, 0], veh_state[:, 0])

[msf_veh_x, msf_veh_y] = InterpPos(
    msf_veh[:, 61],
    msf_veh[:, 62],
    msf_veh[0, 61],
    msf_veh[0, 62],
    msf_veh[:, 0],
    gnss[:, 0],
)

[gnss_y, gnss_x] = GPStoXY(gnss[:, 10], gnss[:, 11], msf_veh[0, 61], msf_veh[0, 62])


plt.rcParams["legend.loc"] = "upper right"

# VEH_SUB_PLOT_NUM = 3
# plt.subplots(VEH_SUB_PLOT_NUM, 1, sharex=True)
# plt.subplot(VEH_SUB_PLOT_NUM, 1, 1)
# plt.plot(veh_state[:, 0], veh_state[:, 10] * (1 + veh_state[-1, 2]))
# plt.plot(msf_veh[:, 0], msf_veh[:, 53])
# plt.subplot(VEH_SUB_PLOT_NUM, 1, 2)
# plt.plot(kcp_state[:, 0], kcp_state[:, 2])
# plt.plot(kcp_state[:, 0], kcp_state[:, 3])
# plt.plot(kcp_state[:, 0], kcp_state[:, 4])
# plt.subplot(VEH_SUB_PLOT_NUM, 1, 3)
# plt.plot(kcp_state[:, 0], kcp_state[:, 5])
# plt.plot(kcp_state[:, 0], kcp_state[:, 6])
# plt.plot(kcp_state[:, 0], kcp_state[:, 7])
# plt.show()

plt.subplots(2, 1, sharex=True)
plt.subplot(2, 1, 1)
plt.plot(gnss[:, 0], gnss[:, 3])
plt.plot(msf_veh[:, 0], msf_veh[:, 3])
plt.subplot(2, 1, 2)
plt.plot(imu_state[:, 0], imu_state[:, 1])
plt.plot(imu_state[:, 0], imu_state[:, 2])
plt.plot(imu_state[:, 0], imu_state[:, 3])
plt.show()

# quit()

SUBPLOT_NUM = 3

plt.subplots(SUBPLOT_NUM, 1, sharex=True)
plt.subplot(SUBPLOT_NUM, 1, 1)
plt.scatter(msf_veh[:, 0], msf_veh[:, 70], s=3, c=msf_veh[:, 77])
plt.axhline(y=0, c="black")
plt.legend(["pos"])
plt.subplot(SUBPLOT_NUM, 1, 2)
plt.scatter(msf_veh[:, 0], msf_veh[:, 73], s=3, c=msf_veh[:, 77])
plt.axhline(y=0, c="black")
plt.legend(["pos mean"])
plt.subplot(SUBPLOT_NUM, 1, 3)
plt.scatter(msf_veh[:, 0], msf_veh[:, 76], s=3, c=msf_veh[:, 77])
plt.axhline(y=0, c="black")
plt.legend(["pos std"])
plt.show()

GNSS_STATE_PLOT_NUM = 3
MSF_STATE_PLOT_NUM = 9

STATE_INFO_PLOT_NUM = GNSS_STATE_PLOT_NUM + MSF_STATE_PLOT_NUM
plt.subplots(STATE_INFO_PLOT_NUM, 1, sharex=True)
plt.subplot(STATE_INFO_PLOT_NUM, 1, 1)
plt.scatter(gnss[:, 0], gnss[:, 9], s=3, c=gnss[:, 9])
plt.legend(["rtk state"])
plt.subplot(STATE_INFO_PLOT_NUM, 1, 2)
plt.plot(gnss[:, 0], gnss[:, 8])
plt.plot(gnss[:, 0], gnss[:, 34])
plt.legend(["sat num"])
# plt.subplot(STATE_INFO_PLOT_NUM, 1, 3)
# plt.plot(gnss[:, 0], gnss[:, 12])
# plt.plot(gnss[:, 0], gnss[:, 13])
# plt.legend(["pos cov E", "pos cov N"])
# plt.subplot(STATE_INFO_PLOT_NUM, 1, 4)
# plt.plot(gnss[:, 0], gnss[:, 15])
# plt.plot(gnss[:, 0], gnss[:, 16])
# plt.legend(["vel cov E", "vel cov N"])
plt.subplot(STATE_INFO_PLOT_NUM, 1, 3)
plt.plot(gnss[:, 0], gnss[:, 19])
plt.legend(["RTK Overall Status"])

plt.subplot(STATE_INFO_PLOT_NUM, 1, 1 + GNSS_STATE_PLOT_NUM)
# plt.plot(msf_veh[:, 0], msf_veh[:, 48], c="red")
# plt.axhline(0.0, c="black")
# plt.axhline(0.3, c="green")
# plt.axhline(-0.3, c="green")
# plt.legend(["ego vel lat"])
plt.plot(msf_veh[:, 0], msf_veh[:, 49])
plt.legend(["ego vel Front"])
plt.subplot(STATE_INFO_PLOT_NUM, 1, 2 + GNSS_STATE_PLOT_NUM)
plt.plot(msf_veh[:, 0], msf_veh[:, 52])
plt.legend(["align type"])
plt.subplot(STATE_INFO_PLOT_NUM, 1, 3 + GNSS_STATE_PLOT_NUM)
plt.plot(msf_veh[:, 0], msf_veh[:, 16] * 180.0 / np.pi)
plt.plot(msf_veh[:, 0], msf_veh[:, 17] * 100.0)
plt.plot(msf_veh[:, 0], msf_veh[:, 18] * 180.0 / np.pi)
plt.scatter(veh_state[:, 0], veh_state[:, 16] * 100.0, c="blue", s=1)
pitch_mean = msf_veh[-100:-1:10, 16].mean() * 180 / np.pi
spd_scale = msf_veh[-100:-1:10, 17].mean()
yaw_mean = msf_veh[-100:-1:10, 18].mean() * 180 / np.pi
plt.legend(
    [
        "veh pch error " + "%.3f" % pitch_mean,
        "veh spd error " + "%.3f" % spd_scale,
        "veh yaw error " + "%.3f" % yaw_mean,
    ]
)
plt.subplot(STATE_INFO_PLOT_NUM, 1, 4 + GNSS_STATE_PLOT_NUM)
plt.plot(msf_veh[:, 0], msf_veh[:, 40])
plt.plot(msf_veh[:, 0], msf_veh[:, 41])
plt.plot(msf_veh[:, 0], msf_veh[:, 42])
plt.legend(["map error R", "map error F", "map error Yaw"])

plt.subplot(STATE_INFO_PLOT_NUM, 1, 5 + GNSS_STATE_PLOT_NUM)
plt.plot(imu_state[:, 0], imu_state[:, 1])
plt.plot(imu_state[:, 0], imu_state[:, 2])
plt.plot(imu_state[:, 0], imu_state[:, 3])
gyro_bias_x = imu_state[-100:-1:10, 1].mean()
gyro_bias_y = imu_state[-100:-1:10, 2].mean()
gyro_bias_z = imu_state[-100:-1:10, 3].mean()
plt.legend(
    [
        "gyro bias x " + "%.3f" % gyro_bias_x,
        "gyro bias y " + "%.3f" % gyro_bias_y,
        "gyro bias z " + "%.3f" % gyro_bias_z,
    ]
)

plt.subplot(STATE_INFO_PLOT_NUM, 1, 6 + GNSS_STATE_PLOT_NUM)
plt.plot(imu_state[:, 0], imu_state[:, 4])
plt.plot(imu_state[:, 0], imu_state[:, 5])
plt.plot(imu_state[:, 0], imu_state[:, 6])
acc_bias_x = imu_state[-100:-1:10, 4].mean()
acc_bias_y = imu_state[-100:-1:10, 5].mean()
acc_bias_z = imu_state[-100:-1:10, 6].mean()
plt.legend(
    [
        "acc bias x " + "%.3f" % acc_bias_x,
        "acc bias y " + "%.3f" % acc_bias_y,
        "acc bias z " + "%.3f" % acc_bias_z,
    ]
)

# plt.subplot(STATE_INFO_PLOT_NUM, 1, 7 + GNSS_STATE_PLOT_NUM)
# plt.plot(vis_state[:, 0], vis_state[:, 7])
# plt.plot(vis_state[:, 0], vis_state[:, 8])
# # plt.plot(vis_state[:, 0], vis_state[:, 3] * 180.0 / np.pi)
# # plt.plot(vis_state[:, 0], vis_state[:, 6] * 180.0 / np.pi)
# plt.plot(vis_state[:, 0], vis_state[:, 9] * 180.0 / np.pi)
# plt.legend(["vis x", "vis y", "vis yaw inno"])
# plt.axhline(y=0, c="black")

plt.subplot(STATE_INFO_PLOT_NUM, 1, 8 + GNSS_STATE_PLOT_NUM)
plt.plot(msf_veh[:, 0], msf_veh[:, 56])
plt.plot(msf_veh[:, 0], msf_veh[:, 58])
plt.axhline(y=0, c="black")
plt.legend(["acc_horiz"])

plt.subplot(STATE_INFO_PLOT_NUM, 1, 9 + GNSS_STATE_PLOT_NUM)
plt.plot(msf_veh[:, 0], msf_veh[:, 57] * 180.0 / np.pi)
plt.plot(msf_veh[:, 0], msf_veh[:, 59] * 180.0 / np.pi)
plt.axhline(y=0, c="black")
plt.legend(["rotate"])

INNO_SUBPLOTS_NUM = 8
STD_LIM = [-3, 3]
plt.subplots(INNO_SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(INNO_SUBPLOTS_NUM, 1, 1)
plt.scatter(gnss[:, 0], gnss[:, 21], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 7], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 28], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 28], c="black")
plt.plot(gnss[:, 0], gnss[:, 12], c="red")
plt.plot(gnss[:, 0], -gnss[:, 12], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 4], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 4], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego pos x", "N"])
plt.subplot(INNO_SUBPLOTS_NUM, 1, 2)
plt.scatter(gnss[:, 0], gnss[:, 22], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 8], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 29], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 29], c="black")
plt.plot(gnss[:, 0], gnss[:, 13], c="red")
plt.plot(gnss[:, 0], -gnss[:, 13], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 5], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 5], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego pos y", "E"])
plt.subplot(INNO_SUBPLOTS_NUM, 1, 3)
plt.scatter(gnss[:, 0], gnss[:, 23], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 9], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 30], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 30], c="black")
plt.plot(gnss[:, 0], gnss[:, 14], c="red")
plt.plot(gnss[:, 0], -gnss[:, 14], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 6], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 6], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego pos z", "U"])
plt.subplot(INNO_SUBPLOTS_NUM, 1, 4)
plt.scatter(gnss[:, 0], gnss[:, 24], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 16], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 31], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 31], c="black")
plt.plot(gnss[:, 0], gnss[:, 15], c="red")
plt.plot(gnss[:, 0], -gnss[:, 15], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 13], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 13], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego vel x", "E"])
plt.subplot(INNO_SUBPLOTS_NUM, 1, 5)
plt.scatter(gnss[:, 0], gnss[:, 25], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 17], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 32], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 32], c="black")
plt.plot(gnss[:, 0], gnss[:, 16], c="red")
plt.plot(gnss[:, 0], -gnss[:, 16], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 14], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 14], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego vel y", "N"])
plt.subplot(INNO_SUBPLOTS_NUM, 1, 6)
plt.scatter(gnss[:, 0], gnss[:, 26], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 18], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 33], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 33], c="black")
plt.plot(gnss[:, 0], gnss[:, 17], c="red")
plt.plot(gnss[:, 0], -gnss[:, 17], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 15], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 15], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego vel z", "U"])
plt.subplot(INNO_SUBPLOTS_NUM, 1, 7)
plt.scatter(gnss[:, 0], gnss[:, 27], s=3, c=gnss[:, 9])
plt.plot(gnss_std[:, 0], gnss_std[:, 19] * 180.0 / np.pi, c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 19] * 180.0 / np.pi, c="black")
plt.axhline(y=0, c="black")
plt.legend(["ego heading"])
plt.subplot(INNO_SUBPLOTS_NUM, 1, 8)
plt.plot(msf_veh[:, 0], msf_veh[:, 60] * 180.0 / np.pi)
plt.axhline(y=0, c="black")
plt.legend(["gyro z"])

plt.subplots(3, 1, sharex=True)
plt.subplot(3, 1, 1)
plt.plot(msf_veh[:, 0], msf_veh[:, 7])
plt.plot(msf_veh[:, 0], msf_veh[:, 48])
plt.legend(["vel E", "vel_ego R"])
plt.subplot(3, 1, 2)
plt.plot(msf_veh[:, 0], msf_veh[:, 8])
plt.plot(msf_veh[:, 0], msf_veh[:, 49])
plt.legend(["vel N", "vel_ego F"])
plt.subplot(3, 1, 3)
plt.plot(msf_veh[:, 0], msf_veh[:, 9])
plt.legend(["vel U"])

plt.subplots(4, 1, sharex=True)
plt.subplot(4, 1, 1)
plt.scatter(gnss[:, 0], gnss[:, 21], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 7], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 28], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 28], c="black")
plt.plot(gnss[:, 0], gnss[:, 12], c="red")
plt.plot(gnss[:, 0], -gnss[:, 12], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 4], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 4], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego pos x", "N"])
plt.subplot(4, 1, 2)
plt.scatter(gnss[:, 0], gnss[:, 22], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 8], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 29], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 29], c="black")
plt.plot(gnss[:, 0], gnss[:, 13], c="red")
plt.plot(gnss[:, 0], -gnss[:, 13], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 5], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 5], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego pos y", "E"])
plt.subplot(4, 1, 3)
plt.scatter(gnss[:, 0], gnss[:, 24], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 16], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 31], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 31], c="black")
plt.plot(gnss[:, 0], gnss[:, 15], c="red")
plt.plot(gnss[:, 0], -gnss[:, 15], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 13], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 13], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego vel x", "E"])
plt.subplot(4, 1, 4)
plt.scatter(gnss[:, 0], gnss[:, 25], s=3, c=gnss[:, 9])
plt.scatter(gnss_std[:, 0], gnss_std[:, 17], s=3, c="orange")
# plt.plot(gnss[:, 0], gnss[:, 32], c="black")
# plt.plot(gnss[:, 0], -gnss[:, 32], c="black")
plt.plot(gnss[:, 0], gnss[:, 16], c="red")
plt.plot(gnss[:, 0], -gnss[:, 16], c="red")
plt.plot(gnss_std[:, 0], gnss_std[:, 14], c="black")
plt.plot(gnss_std[:, 0], -gnss_std[:, 14], c="black")
plt.axhline(y=0, c="black")
plt.ylim(STD_LIM)
plt.legend(["ego vel y", "N"])

plt.figure("traj")
plt.plot(gnss_x, gnss_y, c="orange")
plt.scatter(gnss_x, gnss_y, c=gnss[:, 9], s=10)

# plt.plot(msf_imu_x, msf_imu_y, c="red")
# plt.scatter(msf_imu_x, msf_imu_y, c="red", s=10)

plt.scatter(msf_veh_x, msf_veh_y, c="green", s=10)
plt.plot(msf_veh_x, msf_veh_y, c="green")

# plt.scatter(msf_veh_x_full, msf_veh_y_full, c="blue", s=10)
# plt.plot(msf_veh_x_full, msf_veh_y_full, c="blue")

for i in range(0, gnss[:, 0].shape[0], 100):
    plt.annotate(gnss[i, 0], xy=(msf_veh_x[i], msf_veh_y[i]))
    plt.annotate(gnss[i, 0], xy=(gnss_x[i], gnss_y[i]), c="red")

plt.gca().set_aspect("equal")

plt.show()
