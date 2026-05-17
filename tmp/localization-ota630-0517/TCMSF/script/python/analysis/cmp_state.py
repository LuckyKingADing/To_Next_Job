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


cmp_state = np.array(pd.read_csv("data/tmp/kcp_debug_state.csv"))

msf_veh = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)
gnss = np.array(pd.read_csv("data/tmp/gps_debug_state.csv"), dtype=float)

[gnss_y, gnss_x] = GPStoXY(gnss[:, 10], gnss[:, 11], msf_veh[0, 61], msf_veh[0, 62])


plt.rcParams["legend.loc"] = "upper right"


SUBPLOTS_NUM = 9
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.scatter(cmp_state[:, 0], cmp_state[:, 1], s=3)
plt.legend(["dt"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.scatter(cmp_state[:, 0], cmp_state[:, 2], s=3)
plt.legend(["vx"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.scatter(cmp_state[:, 0], cmp_state[:, 3], s=3)
plt.legend(["vy"])
plt.subplot(SUBPLOTS_NUM, 1, 4)
plt.scatter(cmp_state[:, 0], cmp_state[:, 4], s=3)
plt.legend(["vz"])
plt.subplot(SUBPLOTS_NUM, 1, 5)
plt.scatter(cmp_state[:, 0], cmp_state[:, 5], s=3)
plt.legend(["px"])
plt.subplot(SUBPLOTS_NUM, 1, 6)
plt.scatter(cmp_state[:, 0], cmp_state[:, 6], s=3)
plt.legend(["py"])
plt.subplot(SUBPLOTS_NUM, 1, 7)
plt.scatter(cmp_state[:, 0], cmp_state[:, 7], s=3)
plt.legend(["pz"])
plt.subplot(SUBPLOTS_NUM, 1, 8)
plt.scatter(cmp_state[:, 0], cmp_state[:, 8], s=3)
plt.legend(["v_ego_lon"])
plt.subplot(SUBPLOTS_NUM, 1, 9)
plt.scatter(cmp_state[:, 0], cmp_state[:, 9], s=3)
plt.legend(["hdg"])
plt.show()

dx = gnss_x[2:-1] - gnss_x[1:-2]
dy = gnss_y[2:-1] - gnss_y[1:-2]
dp = np.sqrt(np.power(dx, 2) + np.power(dy, 2))

SUBPLOTS_NUM = 2
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.scatter(gnss[2:-1, 0], gnss[2:-1, 0] - gnss[1:-2, 0], s=3)
plt.ylim([0, 0.3])
plt.legend(["dt"])
# plt.subplot(SUBPLOTS_NUM, 1, 2)
# plt.scatter(gnss[2:-1, 0], dx, s=3)
# plt.scatter(gnss[2:-1, 0], dy, s=3)
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.scatter(gnss[2:-1, 0], dp, s=3)
plt.ylim([0, 8])
plt.legend(["dp"])
plt.show()
