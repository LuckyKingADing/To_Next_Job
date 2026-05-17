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


spp_vali = np.array(pd.read_csv("data/tmp/spp_vali_state.csv"), dtype=float)

plt.rcParams["legend.loc"] = "upper right"

SUB_NUM = 5
plt.subplots(SUB_NUM, 1, sharex=True)
plt.subplot(SUB_NUM, 1, 1)
# plt.plot(spp_vali[:, 0], spp_vali[:, 4])
plt.scatter(spp_vali[:, 0], spp_vali[:, 9], s=0.3)
plt.plot(spp_vali[:, 0], spp_vali[:, 5], c="red")
plt.subplot(SUB_NUM, 1, 2)
plt.plot(spp_vali[:, 0], spp_vali[:, 6])
plt.ylim([0, 10])
plt.legend(["pos_inno"])
plt.subplot(SUB_NUM, 1, 3)
plt.plot(spp_vali[:, 0], spp_vali[:, 7])
plt.ylim([0, 1.5])
plt.legend(["dr_gnss_crx"])
plt.subplot(SUB_NUM, 1, 4)
plt.scatter(spp_vali[:, 0], spp_vali[:, 8], s=0.3)
plt.legend(["bad gnss"])
plt.subplot(SUB_NUM, 1, 5)
plt.plot(spp_vali[:, 0], spp_vali[:, 5] - spp_vali[:, 9])
plt.legend(["sat num diff"])
plt.show()
