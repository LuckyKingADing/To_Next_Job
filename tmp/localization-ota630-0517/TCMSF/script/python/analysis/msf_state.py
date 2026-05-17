import pandas as pd
import numpy as np

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


def rot2d(x, y, theta):
    x_ = x * np.cos(theta) - y * np.sin(theta)
    y_ = x * np.sin(theta) + y * np.cos(theta)
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


msf_veh = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)
gnss = np.array(pd.read_csv("data/tmp/gps_debug_state.csv"), dtype=float)

gnss = gnss[gnss[:, 1] != 0.0, :]

msf_imu_state_interp = InterpState(msf_veh, msf_veh[:, 0], gnss[:, 0])

map_bias = msf_imu_state_interp[:, [0, 37, 38, 39, 40, 41, 42]]
map_bias_full = msf_veh[:, [0, 37, 38, 39, 40, 41, 42]]

[msf_veh_x, msf_veh_y] = InterpPos(
    msf_veh[:, 1],
    msf_veh[:, 2],
    msf_veh[0, 1],
    msf_veh[0, 2],
    msf_veh[:, 0],
    gnss[:, 0],
)

# ## ---------
veh_state = np.array(pd.read_csv("data/tmp/veh_debug_state.csv"))

# plt.figure("veh error")
# plt.subplot(3, 1, 1)
# pitch_mean = veh_state[-1000:-1:100, 1].mean()
# plt.text(veh_state[1000, 0], 0.5, "pitch steady state: " + str(pitch_mean))
# plt.plot(veh_state[:, 0], veh_state[:, 1] - pitch_mean)
# plt.plot(veh_state[:, 0], veh_state[:, 4], c="black")
# plt.plot(veh_state[:, 0], -veh_state[:, 4], c="black")
# plt.ylim([-1.0, 1.0])
# plt.legend(["pitch - ss mean"])
# plt.subplot(3, 1, 2)
# spd_scale = veh_state[-1000:-1:100, 2].mean()
# plt.text(veh_state[1000, 0], 0.0025, "spd_scale steady state: " + str(spd_scale))
# plt.plot(veh_state[:, 0], veh_state[:, 2] - spd_scale)
# plt.plot(veh_state[:, 0], veh_state[:, 5], c="black")
# plt.plot(veh_state[:, 0], -veh_state[:, 5], c="black")
# plt.ylim([-0.005, 0.005])
# plt.legend(["spd_scale - ss mean"])
# plt.subplot(3, 1, 3)
# yaw_mean = veh_state[-1000:-1:100, 3].mean()
# plt.text(veh_state[1000, 0], 0.5, "yaw steady state: " + str(yaw_mean))
# plt.plot(veh_state[:, 0], veh_state[:, 3] - yaw_mean)
# plt.plot(veh_state[:, 0], veh_state[:, 6], c="black")
# plt.plot(veh_state[:, 0], -veh_state[:, 6], c="black")
# plt.ylim([-1.0, 1.0])
# plt.legend(["yaw - ss mean"])

# plt.figure("slip")
# plt.subplot(3, 1, 1)
# plt.plot(veh_state[:, 0], veh_state[:, 7])
# plt.subplot(3, 1, 2)
# plt.plot(veh_state[:, 0], veh_state[:, 8])
# plt.subplot(3, 1, 3)
# plt.plot(veh_state[:, 0], veh_state[:, 9])
# plt.show()

# ## _______________


[gnss_y, gnss_x] = GPStoXY(gnss[:, 1], gnss[:, 2], msf_veh[0, 1], msf_veh[0, 2])
[msf_veh_y_full, msf_veh_x_full] = GPStoXY(
    msf_veh[:, 1],
    msf_veh[:, 2],
    msf_veh[0, 1],
    msf_veh[0, 2],
)

delta_pos = np.sqrt(np.power(msf_veh_x - gnss_x, 2) + np.power(msf_veh_y - gnss_y, 2))


msf_veh_hdg_interp = np.interp(gnss[:, 0], msf_veh[:, 0], msf_veh[:, 6])


delta_vel_x = msf_imu_state_interp[:, 7] - gnss[:, 4]
delta_vel_y = msf_imu_state_interp[:, 8] - gnss[:, 5]

[ego_vel_x, ego_vel_y] = rot2d(
    gnss[:, 4], gnss[:, 5], -msf_veh_hdg_interp / 180 * np.pi
)

# plt.figure("dt")
# plt.plot(gnss[1:-1, 0] - gnss[0:-2, 0])
# plt.show()

# plt.figure("ego vel")
# plt.plot(msf_imu_state_interp[:, 0], msf_imu_state_interp[:, 43])
# plt.show()

plt.figure("state")
plt.plot(msf_imu_state_interp[:, 0], msf_imu_state_interp[:, 48], c="red")
# plt.plot(msf_imu_state_interp[:, 0], np.abs(msf_imu_state_interp[:, 49]), c="blue")
# plt.plot(msf_imu_state_interp[:, 0], msf_imu_state_interp[:, 50], c="orange")
# plt.plot(msf_imu_state_interp[:, 0], np.abs(msf_imu_state_interp[:, 51]), c="green")
plt.axhline(0.0, c="black")
plt.axhline(0.3, c="green")
plt.axhline(-0.3, c="green")
plt.show()

plt.figure("rtk std")
plt.subplot(3, 1, 1)
plt.scatter(gnss[:, 0], gnss[:, 12], c=gnss[:, 9], s=4)
plt.scatter(gnss[:, 0], gnss[:, 13], c=gnss[:, 9], s=4)
plt.axhline(1, c="black")
plt.axhline(2, c="red")
plt.axhline(3, c="blue")
plt.axhline(4, c="green")
plt.axhline(5, c="orange")
plt.axhline(6, c="brown")
plt.ylim([0, 20])
plt.subplot(3, 1, 2)
plt.scatter(gnss[:, 0], gnss[:, 15], c=gnss[:, 9], s=4)
plt.scatter(gnss[:, 0], gnss[:, 16], c=gnss[:, 9], s=4)
plt.scatter(gnss[:, 0], gnss[:, 17], c=gnss[:, 9], s=4)
plt.axhline(1, c="black")
plt.axhline(2, c="red")
plt.axhline(3, c="blue")
plt.ylim([0, 10])
plt.subplot(3, 1, 3)
plt.plot(gnss[:, 0], gnss[:, 18] * 180 / np.pi)
plt.axhline(1, c="black")
plt.axhline(2, c="red")
plt.axhline(3, c="blue")
plt.ylim([0, 10])
plt.show()

plt.figure("dp")
plt.plot(gnss[:, 0], delta_pos)
plt.scatter(gnss[:, 0], gnss[:, 9], c="black", s=4)

# plt.scatter(gnss[:, 0], -gnss[:, 7], c="red", s=3)
# plt.plot(gnss[:, 0], msf_veh_hdg_interp / 180 * np.pi)

delta_hdg = gnss[:, 7] + msf_veh_hdg_interp / 180 * np.pi

plt.scatter(gnss[:, 0], delta_hdg * 180 / np.pi, s=3, c="red")
plt.axhline(0, c="black")


plt.plot(gnss[:, 0], gnss[:, 12], c="yellow")
plt.plot(gnss[:, 0], gnss[:, 15], c="yellow")

plt.plot(veh_state[:, 0], veh_state[:, 3])

plt.plot(msf_imu_state_interp[:, 0], msf_imu_state_interp[:, 48], c="blue")

plt.plot(gnss[:, 0], ego_vel_x, c="green")
# plt.plot(gnss[:, 0], ego_vel_y, c="green")

plt.plot(msf_imu_state_interp[:, 0], msf_imu_state_interp[:, 43], c="orange")


plt.plot(msf_veh[:, 0], msf_veh[:, 22], c="black")
plt.plot(msf_veh[:, 0], -msf_veh[:, 22], c="black")

plt.plot(msf_veh[:, 0], msf_veh[:, 25], c="red")
plt.plot(msf_veh[:, 0], -msf_veh[:, 25], c="red")

# plt.plot(map_bias[:, 0], map_bias[:, 1], c="black")
# plt.plot(map_bias[:, 0], map_bias[:, 4], c="brown")
# plt.plot(map_bias[:, 0], -map_bias[:, 1], c="black")

plt.ylim([-10, 20])

# plt.show()


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
