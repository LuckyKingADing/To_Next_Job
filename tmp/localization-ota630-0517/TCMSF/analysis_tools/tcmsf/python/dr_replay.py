import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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


def vector_angle_2d_simple(v1, v2, degrees=False):
    """
    专门针对二维向量的简化版本
    使用atan2函数直接计算带符号的角度[5,8](@ref)
    """
    if len(v1) != 2 or len(v2) != 2:
        raise ValueError("此函数仅适用于二维向量")

    v1 = np.array(v1)
    v2 = np.array(v2)

    # 计算每个向量与x轴的夹角[5](@ref)
    angle1 = math.atan2(v1[1], v1[0])
    angle2 = math.atan2(v2[1], v2[0])

    # 计算角度差[8](@ref)
    angle_diff = angle2 - angle1

    # 将角度规范化到[-π, π]范围内
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi

    if degrees:
        return math.degrees(angle_diff)
    else:
        return angle_diff


dr = np.array(pd.read_csv("data/tcmsf/dr_replay.csv"))

plt.rcParams["legend.loc"] = "upper left"

plt.plot(dr[:, 2], dr[:, 3])

plt.gca().set_aspect("equal")
plt.show()

SUBS = 2
plt.subplots(SUBS, 1, sharex=True)
plt.subplot(SUBS, 1, 1)
plt.plot(dr[:, 1], dr[:, 9])
plt.subplot(SUBS, 1, 2)
plt.plot(dr[:, 1], dr[:, 15])
plt.show()
