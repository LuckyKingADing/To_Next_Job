import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.gridspec as gridspec


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


rts = np.array(pd.read_csv("data/tmp/rts_result.csv"))
gt = np.array(pd.read_csv("data/shared_ubuntu/20250428_span_stdref_02.csv"))
lc = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"))

gt[:, 1] = gt[:, 1] / 1000.0
gt_interp = InterpState(gt, gt[:, 1], rts[:, 10])
lc_interp = InterpState(lc, lc[:, 0], rts[:, 0])

[dpos_E, dpos_N] = GPStoXY(rts[:, 7], rts[:, 8], gt_interp[:, 5], gt_interp[:, 6])


plt.rcParams["legend.loc"] = "upper left"

SUBNUM = 3
plt.subplots(SUBNUM, 1, sharex=True)
plt.subplot(SUBNUM, 1, 1)
plt.scatter(rts[:, 0], dpos_E, s=0.3)
plt.axhline(y=0, c="black")
plt.axhline(
    y=-0.2,
    c="black",
    linestyle="--",
)
plt.axhline(
    y=0.2,
    c="black",
    linestyle="--",
)
plt.ylim([-0.5, 0.5])
plt.legend(["dpos E rts-100c"])
plt.subplot(SUBNUM, 1, 2)
plt.scatter(rts[:, 0], dpos_N, s=0.3)
plt.axhline(y=0, c="black")
plt.axhline(
    y=-0.2,
    c="black",
    linestyle="--",
)
plt.axhline(
    y=0.2,
    c="black",
    linestyle="--",
)
plt.ylim([-0.5, 0.5])
plt.legend(["dpos N rts-100c"])
plt.subplot(SUBNUM, 1, 3)
plt.scatter(rts[:, 0], rts[:, 3] + gt_interp[:, 17], s=0.3)
plt.scatter(lc_interp[:, 0], lc_interp[:, 6] + gt_interp[:, 17], s=0.3)
plt.scatter(lc_interp[:, 0], rts[:, 3] - lc_interp[:, 6], s=0.3)
plt.axhline(y=0, c="black")
plt.axhline(
    y=-0.2,
    c="black",
    linestyle="--",
)
plt.axhline(
    y=0.2,
    c="black",
    linestyle="--",
)
plt.ylim([-0.5, 0.5])
plt.legend(["dhdg rts-100c", "dhdg lc-100c", "dhdg rts-lc"])
plt.show()
