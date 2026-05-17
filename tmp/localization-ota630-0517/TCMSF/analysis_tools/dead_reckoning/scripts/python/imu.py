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


imu = np.array(pd.read_csv("data/parsed/imu.csv"))

acc = imu[0:1000, 2:5]
gyro = imu[0:1000, 5:8]

acc_norm = np.linalg.norm(acc, ord=2, axis=1)
gyro_mean = np.mean(gyro, axis=0)


plt.rcParams["legend.loc"] = "upper right"

plt.figure("imu")
plt.subplot(4, 1, 1)
plt.plot(acc_norm)
acc_mean = np.mean(acc_norm)
plt.axhline(acc_mean)
plt.legend(["acc mean: " + "%5.6f" % acc_mean])
plt.subplot(4, 1, 2)
plt.plot(gyro[:, 0])
plt.axhline(gyro_mean[0])
plt.legend(["gyro mean x: " + "%5.6f" % gyro_mean[0]])
plt.subplot(4, 1, 3)
plt.plot(gyro[:, 1])
plt.axhline(gyro_mean[1])
plt.legend(["gyro mean y: " + "%5.6f" % gyro_mean[1]])
plt.subplot(4, 1, 4)
plt.plot(gyro[:, 2])
plt.axhline(gyro_mean[2])
plt.legend(["gyro mean z: " + "%5.6f" % gyro_mean[2]])
plt.show()
