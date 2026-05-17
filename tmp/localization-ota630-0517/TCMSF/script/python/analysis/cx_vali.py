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


alg_imu = np.array(pd.read_csv("data/tmp/alg_debug_state.csv"))
msf_veh = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)
gnss = np.array(pd.read_csv("data/tmp/gps_debug_state.csv"), dtype=float)
veh_state = np.array(pd.read_csv("data/tmp/veh_debug_state.csv"))
imu_state = np.array(pd.read_csv("data/tmp/imu_debug_state.csv"))
kcp_state = np.array(pd.read_csv("data/tmp/kcp_debug_state.csv"))

cx_vali = np.array(pd.read_csv("data/tmp/cx_vali_state.csv"))

# plt.figure("time")
# plt.scatter(msf_veh[2:-1, 0], msf_veh[2:-1, 0] - msf_veh[1:-2, 0], s=3)
# plt.legend(["msf dt"])
# plt.show()


gnss_std = np.array(pd.read_csv("data/tmp/gps_std.csv"), dtype=float)

# vis_state = np.array(pd.read_csv("data/tmp/vis_debug_state.csv"))

msf_veh_interp = InterpState(msf_veh, msf_veh[:, 0], veh_state[:, 0])

[msf_veh_x, msf_veh_y] = InterpPos(
    msf_veh[:, 61],
    msf_veh[:, 62],
    msf_veh[0, 61],
    msf_veh[0, 62],
    msf_veh[:, 0],
    gnss[:, 0],
)

[gnss_y, gnss_x] = GPStoXY(gnss[:, 10], gnss[:, 11], msf_veh[0, 61], msf_veh[0, 62])


plt.rcParams["legend.loc"] = "upper right"

CX_SUBPLOTS = 5
plt.subplots(CX_SUBPLOTS, 1, sharex=True)
plt.subplot(CX_SUBPLOTS, 1, 1)
plt.plot(cx_vali[:, 0], cx_vali[:, 1])
plt.legend(["SSI"])
plt.subplot(CX_SUBPLOTS, 1, 2)
plt.plot(cx_vali[:, 0], cx_vali[:, 2])
plt.ylim([-1, 2])
plt.legend(["pos_inno_norm"])
plt.subplot(CX_SUBPLOTS, 1, 3)
plt.plot(cx_vali[:, 0], cx_vali[:, 4])
plt.ylim([-1, 2])
plt.legend(["gnss_dr_diff"])
plt.subplot(CX_SUBPLOTS, 1, 4)
plt.plot(cx_vali[:, 0], cx_vali[:, 5])
plt.ylim([-1, 2])
plt.legend(["msf_gnss_diff"])
plt.subplot(CX_SUBPLOTS, 1, 5)
plt.plot(cx_vali[:, 0], cx_vali[:, 6])
plt.ylim([-1, 2])
plt.legend(["cross_diff"])
plt.show()
