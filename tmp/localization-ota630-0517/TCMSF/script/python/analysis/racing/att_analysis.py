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


msf_veh = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)

veh_state = np.array(pd.read_csv("data/tmp/veh_debug_state.csv"))


plt.rcParams["legend.loc"] = "upper right"

acc_lgt = veh_state[:, 20]
acc_lgt[acc_lgt > 0.0] = acc_lgt[acc_lgt > 0.0] * 0.34
acc_lgt[acc_lgt <= 0.0] = acc_lgt[acc_lgt <= 0.0] * 0.2

acc_lat = veh_state[:, 19]
acc_lat[acc_lat > 0.0] = acc_lat[acc_lat > 0.0] * 0.4
acc_lat[acc_lat <= 0.0] = acc_lat[acc_lat <= 0.0] * 0.4

plt.subplots(2, 1, sharex=True)
plt.subplot(2, 1, 1)
plt.plot(msf_veh[:, 0], msf_veh[:, 4])
# plt.plot(veh_state[:, 0], acc_lgt)
plt.plot(veh_state[:, 0], veh_state[:, 21])
plt.axhline(0, c="k")
plt.legend(["pitch", "pitch_comp"])
plt.subplot(2, 1, 2)
plt.plot(msf_veh[:, 0], msf_veh[:, 5])
# plt.plot(veh_state[:, 0], acc_lat)
plt.plot(veh_state[:, 0], veh_state[:, 22])
plt.axhline(0, c="k")
plt.legend(["roll", "roll_comp"])
plt.show()
