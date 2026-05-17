import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


import argparse


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


def convert_csv(csv_file, N):
    ds_ins = np.array(pd.read_csv(csv_file))
    ds_ins = ds_ins[0 : -1 : (int)(N), :]  # 降频索引
    ds_ins = ds_ins[ds_ins[:, 2] != 0, :]  # 提取有效
    ds_ins[:, 3:5] = ds_ins[:, 3:5] / 1e7
    return ds_ins


def main():

    parser = argparse.ArgumentParser(
        description="DS INS数据格式转换", formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--input", "-i", help="输入文件", required=True)
    parser.add_argument("--output", "-o", help="输出文件", required=True)
    parser.add_argument("--downsample", "-N", help="等距降采样", default=1)
    args = parser.parse_args()
    try:
        ds_ins_out = convert_csv(args.input, args.downsample)
        np.savetxt(args.output, ds_ins_out, delimiter=",")
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
