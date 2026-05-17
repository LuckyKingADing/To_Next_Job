import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt

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


dr = np.array(pd.read_csv("data/tcmsf/dr.csv"))


# ======================================

plt.rcParams["legend.loc"] = "upper left"

# ----------------------------------------------
fig = plt.figure("Sensor Info", figsize=(16, 8))
gs = gridspec.GridSpec(3, 3, width_ratios=[1, 1, 1], height_ratios=[1, 1, 1])


ax_left1 = fig.add_subplot(gs[:, 0])

ax_left1.plot(dr[:, 2], dr[:, 3])
ax_left1.legend(["dr traj"])
plt.gca().set_aspect("equal")


ax_middle1 = fig.add_subplot(gs[0, 1])

ax_middle1.plot(dr[:, 1], dr[:, 2])
ax_middle1.legend(["dr_F (m)"])


ax_middle2 = fig.add_subplot(gs[1, 1])

ax_middle2.plot(dr[:, 1], dr[:, 3])
ax_middle2.legend(["dr_L (m)"])

ax_middle3 = fig.add_subplot(gs[2, 1])

ax_middle3.plot(dr[:, 1], dr[:, 4])

ax_middle3.set_ylim([-5 + dr[0, 4], 5 + dr[0, 4]])
ax_middle3.legend(["dr_U (m)"])


ax_right1 = fig.add_subplot(gs[0, 2])

ax_right1.plot(dr[:, 1], dr[:, 12])
ax_right1.set_ylim([-5, 5])
ax_right1.legend(["eulr_x (deg)"])

ax_right2 = fig.add_subplot(gs[1, 2])

ax_right2.plot(dr[:, 1], dr[:, 13])
ax_right2.set_ylim([-5, 5])
ax_right2.legend(["eulr_y (deg)"])

ax_right3 = fig.add_subplot(gs[2, 2])

ax_right3.plot(dr[:, 1], dr[:, 14])
ax_right3.legend(["eulr_z (deg)"])

plt.show()
