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


gnss_std = np.array(pd.read_csv("data/tmp/gps_std.csv"))
gnss_update = np.array(pd.read_csv("data/tmp/gps_update.csv"))
gnss_state = np.array(pd.read_csv("data/tmp/gps_debug_state.csv"), dtype=float)


SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.scatter(gnss_state[:, 0], gnss_state[:, 9], s=0.1)
plt.legend(["rtk status"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.scatter(gnss_state[:, 0], gnss_state[:, 20], s=0.1)
plt.legend(["align type"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.scatter(gnss_update[:, 0], gnss_update[:, 22], s=0.1)
plt.axhline(y=0.01)
plt.axhline(y=0.001)
plt.legend(["pos K norm"])
plt.ylim([-0.01, 0.012])
plt.show()

# SUBPLOTS_NUM = 3
# plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
# plt.subplot(SUBPLOTS_NUM, 1, 1)
# plt.plot(gnss_std[:, 7])
# plt.axhline(y=0, c="black")
# plt.subplot(SUBPLOTS_NUM, 1, 2)
# plt.plot(gnss_std[:, 8])
# plt.axhline(y=0, c="black")
# plt.subplot(SUBPLOTS_NUM, 1, 3)
# plt.plot(gnss_std[:, 9])
# plt.axhline(y=0, c="black")
# plt.show()


SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(gnss_update[:, 0], gnss_update[:, 1] * 180.0 / np.pi)
plt.plot(gnss_update[:, 0], gnss_update[:, 2] * 180.0 / np.pi)
plt.plot(gnss_update[:, 0], gnss_update[:, 3] * 180.0 / np.pi)
plt.axhline(y=0, c="black")
plt.ylim([-0.1, 0.1])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(gnss_update[:, 0], gnss_update[:, 4])
plt.plot(gnss_update[:, 0], gnss_update[:, 5])
plt.plot(gnss_update[:, 0], gnss_update[:, 6])
plt.axhline(y=0, c="black")
plt.ylim([-0.03, 0.03])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(gnss_update[:, 0], gnss_update[:, 8])
plt.plot(gnss_update[:, 0], gnss_update[:, 7])
plt.plot(gnss_update[:, 0], gnss_update[:, 9])
plt.axhline(y=0, c="black")
plt.ylim([-0.03, 0.03])
# plt.show()

SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True, sharey=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.scatter(gnss_std[:, 0], gnss_std[:, 7], s=3)
plt.plot(gnss_update[:, 0], gnss_update[:, 7], c="black")
plt.plot(gnss_update[:, 0], gnss_update[:, 5], c="green")
plt.plot(gnss_update[:, 0], gnss_update[:, 1] * 180.0 / np.pi, c="red")
plt.axhline(y=0, c="black")
plt.ylim([-0.5, 0.5])
plt.legend(["pos inno", "pos update", "vel update", "pitch update"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.scatter(gnss_std[:, 0], gnss_std[:, 8], s=3)
plt.plot(gnss_update[:, 0], gnss_update[:, 8], c="black")
plt.plot(gnss_update[:, 0], gnss_update[:, 4], c="green")
plt.plot(gnss_update[:, 0], gnss_update[:, 2] * 180.0 / np.pi, c="red")
plt.axhline(y=0, c="black")
plt.legend(["pos inno", "pos update", "vel update", "roll update"])
plt.ylim([-0.5, 0.5])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.scatter(gnss_std[:, 0], gnss_std[:, 9], s=3)
plt.plot(gnss_update[:, 0], gnss_update[:, 9], c="black")
plt.plot(gnss_update[:, 0], gnss_update[:, 6], c="green")
plt.plot(gnss_update[:, 0], gnss_update[:, 3] * 180.0 / np.pi, c="red")
plt.axhline(y=0, c="black")
plt.legend(["pos inno", "pos update", "vel update", "yaw update"])
plt.ylim([-0.5, 0.5])
plt.show()
