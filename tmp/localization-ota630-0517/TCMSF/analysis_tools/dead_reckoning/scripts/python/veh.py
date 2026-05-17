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


veh = np.array(pd.read_csv("data/parsed/vehicle.csv"))
veh_state = np.array(pd.read_csv("data/tmp/veh_debug_state.csv"))

plt.rcParams["legend.loc"] = "upper right"

SUBPLOTS_NUM = 4
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(veh[:, 1], veh[:, 2])
plt.plot(veh[:, 1], veh[:, 6])
plt.legend(["rl wheel spd"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(veh[:, 1], veh[:, 3])
plt.plot(veh[:, 1], veh[:, 7])
plt.legend(["rr wheel spd"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(veh[:, 1], (veh[:, 2] + veh[:, 3]) / 2.0)
plt.plot(veh[:, 1], (veh[:, 6] + veh[:, 7]) / 2.0)
plt.plot(veh[:, 1] - 0.16, (veh[:, 6] + veh[:, 7]) / 2.0)
plt.legend(["wheel spd mean"])
plt.subplot(SUBPLOTS_NUM, 1, 4)
plt.plot(veh[:, 1], veh[:, 4])
plt.legend(["yaw rate"])
plt.show()

SUBPLOTS_NUM = 3
plt.subplots(SUBPLOTS_NUM, 1, sharex=True, sharey=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(veh[:, 1], (veh[:, 2] + veh[:, 3]) / 2.0)
plt.legend(["veh raw"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.plot(veh[:, 1], (veh[:, 6] + veh[:, 7]) / 2.0)
plt.legend(["veh lpf"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.plot(veh_state[:, 0], veh_state[:, 14])
plt.legend(["veh lpf with cmp"])
plt.show()

plt.figure("vel")
plt.plot(veh[:, 1], (veh[:, 2] + veh[:, 3]) / 2.0)
plt.plot(veh[:, 1], (veh[:, 6] + veh[:, 7]) / 2.0)
plt.plot(veh_state[:, 0], veh_state[:, 14])
plt.legend(["veh", "veh lpf", "veh lpf comp"])
plt.show()
