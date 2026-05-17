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


msf_veh = np.array(pd.read_csv("data/tmp/msf_veh_frame.csv"))
msf_imu = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"))
gnss = np.array(pd.read_csv("data/tmp/gnss.csv"))

gnss = gnss[gnss[:, 1] != 0.0, :]

[msf_veh_x, msf_veh_y] = InterpPos(
    msf_veh[:, 1],
    msf_veh[:, 2],
    msf_veh[0, 1],
    msf_veh[0, 2],
    msf_veh[:, 0],
    gnss[:, 0],
)

[msf_imu_x, msf_imu_y] = InterpPos(
    msf_imu[:, 1],
    msf_imu[:, 2],
    msf_veh[0, 1],
    msf_veh[0, 2],
    msf_imu[:, 0],
    gnss[:, 0],
)

[gnss_y, gnss_x] = GPStoXY(gnss[:, 1], gnss[:, 2], msf_veh[0, 1], msf_veh[0, 2])
[msf_veh_y_full, msf_veh_x_full] = GPStoXY(
    msf_veh[:, 1],
    msf_veh[:, 2],
    msf_veh[0, 1],
    msf_veh[0, 2],
)

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

plt.gca().set_aspect("equal")

plt.show()

moving = (np.abs(gnss[:, 4]) > 5) | (np.abs(gnss[:, 6]) > 5)
gnss = gnss[moving, :]


msf_veh_hdg_interp = np.interp(gnss[:, 0], msf_veh[:, 0], msf_veh[:, 14])


plt.figure("heading cmp")
plt.plot(gnss[:, 0], gnss[:, 7])
plt.plot(gnss[:, 0], msf_veh_hdg_interp)


delta_hdg = gnss[:, 7] - msf_veh_hdg_interp

plt.figure("raw heading")
plt.scatter(gnss[:, 0], delta_hdg, s=3)
plt.axhline(np.mean(delta_hdg))
plt.show()


# msf_imu_state_interp = InterpState(msf_imu, msf_imu[:, 0], gnss[:, 0])

# plt.subplots(6, 1, sharex=True)
# plt.subplot(6, 1, 1)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 10] * 180 / np.pi)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 28] * 180 / np.pi, c="black")
# plt.plot(gnss[:, 0], -msf_imu_state_interp[:, 28] * 180 / np.pi, c="black")
# plt.ylim([-0.1, 0.1])

# plt.subplot(6, 1, 2)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 11] * 180 / np.pi)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 29] * 180 / np.pi, c="black")
# plt.plot(gnss[:, 0], -msf_imu_state_interp[:, 29] * 180 / np.pi, c="black")
# plt.ylim([-0.1, 0.1])

# plt.subplot(6, 1, 3)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 12] * 180 / np.pi)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 30] * 180 / np.pi, c="black")
# plt.plot(gnss[:, 0], -msf_imu_state_interp[:, 30] * 180 / np.pi, c="black")
# plt.ylim([-0.1, 0.1])

# plt.subplot(6, 1, 4)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 16] * 180 / np.pi)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 34] * 180 / np.pi, c="black")
# plt.plot(gnss[:, 0], -msf_imu_state_interp[:, 34] * 180 / np.pi, c="black")
# plt.ylim([-1.0, 1.0])

# plt.subplot(6, 1, 5)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 17])
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 35], c="black")
# plt.plot(gnss[:, 0], -msf_imu_state_interp[:, 35], c="black")
# plt.ylim([-0.02, 0.02])

# plt.subplot(6, 1, 6)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 18] * 180 / np.pi)
# plt.plot(gnss[:, 0], msf_imu_state_interp[:, 36] * 180 / np.pi, c="black")
# plt.plot(gnss[:, 0], -msf_imu_state_interp[:, 36] * 180 / np.pi, c="black")
# plt.ylim([-2.0, 2.0])

# plt.show()
