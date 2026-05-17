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


def YawChangeIdx(yaw):
    dyaw = np.abs(yaw[1:] - yaw[0:-1])
    return np.where(dyaw > 180.0)[0] + 1


def MakeYawContinous(yaw):
    MsfYawChange = YawChangeIdx(yaw)
    print(MsfYawChange)
    for idx in MsfYawChange:
        yaw[idx:] = yaw[idx:] - (yaw[idx] - yaw[idx - 1]) + (yaw[idx + 1] - yaw[idx])


msf_veh = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)
gnss = np.array(pd.read_csv("data/tmp/gps_debug_state.csv"), dtype=float)
gnss_std = np.array(pd.read_csv("data/tmp/gps_std.csv"), dtype=float)

gnss[:, 7] = gnss[:, 7] * 180.0 / 3.141592653

MakeYawContinous(gnss[:, 7])
MakeYawContinous(msf_veh[:, 6])
msf_veh_interp = InterpState(msf_veh, msf_veh[:, 0], gnss[:, 0])

MakeYawContinous(msf_veh_interp[:, 6])

plt.rcParams["legend.loc"] = "upper right"

SUBPLOTS = 4
plt.subplots(SUBPLOTS, 1, sharex=True)
plt.subplot(SUBPLOTS, 1, 1)
plt.plot(msf_veh[:, 0], -msf_veh[:, 6])
plt.plot(gnss[:, 0], gnss[:, 7])
plt.subplot(SUBPLOTS, 1, 2)
plt.plot(gnss[:, 0], gnss[:, 7] + msf_veh_interp[:, 6])
plt.plot(msf_veh_interp[:, 0], msf_veh_interp[:, 21] * 180.0 / 3.141592653, c="black")

plt.plot(gnss_std[:, 0], gnss_std[:, 19] * 180.0 / np.pi, c="gray")

plt.plot(msf_veh_interp[:, 0], -msf_veh_interp[:, 21] * 180.0 / 3.141592653, c="black")

plt.plot(gnss_std[:, 0], -gnss_std[:, 19] * 180.0 / np.pi, c="gray")
# plt.plot(gnss_std[:, 0], gnss_std[:, 20] * 180.0 / np.pi, c="orange")
# plt.plot(gnss_std[:, 0], -gnss_std[:, 20] * 180.0 / np.pi, c="orange")
plt.legend(["hdg diff", "msf hdg std", "gnss hdg std"])
plt.subplot(SUBPLOTS, 1, 3)
plt.plot(msf_veh[:, 0], msf_veh[:, 53])
plt.legend(["speed"])
plt.subplot(SUBPLOTS, 1, 4)
plt.plot(gnss[:, 0], gnss[:, 35] * 180 / np.pi)
plt.plot(gnss[:, 0], gnss[:, 36] * 180 / np.pi)
plt.plot(gnss[:, 0], gnss[:, 37] * 180 / np.pi)
plt.legend(["hdg inno compensated", "hdg inno imu", "hdg inno veh"])
plt.show()
