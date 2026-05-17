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


vis = np.array(pd.read_csv("data/tmp/vis_debug_state.csv"))

veh_state = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"))

# gnss = np.array(pd.read_csv("data/tmp/gps_debug_state.csv"), dtype=float)

VIS_SUBPLOTS = 2
plt.subplots(VIS_SUBPLOTS, 1, sharex=True)
plt.subplot(VIS_SUBPLOTS, 1, 1)
plt.scatter(vis[:, 0], vis[:, 12] * 180 / np.pi, s=3)
plt.subplot(VIS_SUBPLOTS, 1, 2)
plt.scatter(vis[:, 0], vis[:, 13] * 180 / np.pi, s=3)
plt.show()


VIS_SUBPLOTS = 6
plt.subplots(VIS_SUBPLOTS, 1, sharex=True)
plt.subplot(VIS_SUBPLOTS, 1, 1)
plt.scatter(vis[:, 0], np.abs(vis[:, 1]), s=3)
plt.ylabel("dpos x mean")
plt.subplot(VIS_SUBPLOTS, 1, 2)
plt.scatter(vis[:, 0], vis[:, 4], s=3)
plt.ylabel("dpos x std")
plt.subplot(VIS_SUBPLOTS, 1, 3)
plt.scatter(vis[:, 0], np.abs(vis[:, 3]) * 180 / np.pi, s=3)
plt.ylabel("dhdg mean")
plt.subplot(VIS_SUBPLOTS, 1, 4)
plt.scatter(vis[:, 0], vis[:, 6] * 180 / np.pi, s=3)
plt.ylabel("dhdg std")
plt.subplot(VIS_SUBPLOTS, 1, 5)
plt.scatter(vis[:, 0], vis[:, 7], s=3)
# plt.scatter(vis[:, 0], vis[:, 10], c="red", s=2)
plt.axhline(0, c="black")
plt.plot(veh_state[:, 0], veh_state[:, 37], c="black")
plt.plot(veh_state[:, 0], -veh_state[:, 37], c="black")
plt.ylabel("dpos x")
plt.subplot(VIS_SUBPLOTS, 1, 6)
plt.scatter(vis[:, 0], vis[:, 9] * 180 / np.pi, s=3)
# plt.scatter(vis[:, 0], vis[:, 12], c="red", s=2)
plt.plot(veh_state[:, 0], veh_state[:, 39] * 180 / np.pi, c="black")
plt.plot(veh_state[:, 0], -veh_state[:, 39] * 180 / np.pi, c="black")
plt.ylabel("dhdg")
plt.show()
