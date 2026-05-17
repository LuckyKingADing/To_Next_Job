import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


gnss_std = np.array(pd.read_csv("data/tmp/gps_std.csv"))
gnss_update = np.array(pd.read_csv("data/tmp/gps_update.csv"))

bias_state = np.array(pd.read_csv("data/tmp/bias_esti_state.csv"))
imu_state = np.array(pd.read_csv("data/tmp/imu_debug_state.csv"))
# gt = np.array(pd.read_csv("data/post_results/post_mix_result_.txt", delimiter=" "))

# gt[:, 1:4] = gt[:, 1:4] / 3600
# gt[:, 4:7] = gt[:, 4:7] / 1e5

plt.rcParams["legend.loc"] = "upper left"

SUBPLOTS_NUM = 6
plt.subplots(SUBPLOTS_NUM, 1, sharex=True)
plt.subplot(SUBPLOTS_NUM, 1, 1)
plt.plot(imu_state[:, 0], imu_state[:, 3])
# plt.plot(gt[:, 0], -gt[:, 3])
plt.legend(["gyro bias z", "gt"])
plt.subplot(SUBPLOTS_NUM, 1, 2)
plt.scatter(bias_state[:, 0], bias_state[:, 12], s=3)
plt.axhline(0,c='black')
plt.legend(["gyro bias kf dz"])
plt.subplot(SUBPLOTS_NUM, 1, 3)
plt.scatter(bias_state[:, 0], -bias_state[:, 3], s=3)
plt.axhline(0,c='black')
plt.legend(["hdg kf dz"])

plt.subplot(SUBPLOTS_NUM, 1, 4)
plt.scatter(gnss_std[:, 0], gnss_std[:, 7], s=3)
plt.plot(gnss_update[:, 0], gnss_update[:, 7], c="black")
plt.plot(gnss_update[:, 0], gnss_update[:, 5], c="green")
plt.axhline(y=0, c="black")
plt.ylim([-0.5, 0.5])
plt.legend(["pos inno E", "pos update", "vel update"])
plt.subplot(SUBPLOTS_NUM, 1, 5)
plt.scatter(gnss_std[:, 0], gnss_std[:, 8], s=3)
plt.plot(gnss_update[:, 0], gnss_update[:, 8], c="black")
plt.plot(gnss_update[:, 0], gnss_update[:, 4], c="green")
plt.axhline(y=0, c="black")
plt.legend(["pos inno N", "pos update", "vel update"])
plt.ylim([-0.5, 0.5])
plt.subplot(SUBPLOTS_NUM, 1, 6)
plt.plot(gnss_update[:, 0], gnss_update[:, 3] * 180.0 / np.pi, c="red")
plt.axhline(y=0, c="black")
plt.legend(["yaw update"])
plt.ylim([-0.5, 0.5])
plt.show()


# VEH_SUBPLOTS = 9
# plt.subplots(VEH_SUBPLOTS, 1, sharex=True)
# plt.subplot(VEH_SUBPLOTS, 1, 1)
# plt.plot(bias_state[:, 0], bias_state[:, 1])
# plt.subplot(VEH_SUBPLOTS, 1, 2)
# plt.plot(bias_state[:, 0], bias_state[:, 2])
# plt.subplot(VEH_SUBPLOTS, 1, 3)
# plt.plot(bias_state[:, 0], bias_state[:, 3])
# plt.subplot(VEH_SUBPLOTS, 1, 4)
# plt.plot(bias_state[:, 0], bias_state[:, 4])
# plt.subplot(VEH_SUBPLOTS, 1, 5)
# plt.plot(bias_state[:, 0], bias_state[:, 5])
# plt.subplot(VEH_SUBPLOTS, 1, 6)
# plt.plot(bias_state[:, 0], bias_state[:, 6])
# plt.subplot(VEH_SUBPLOTS, 1, 7)
# plt.plot(bias_state[:, 0], bias_state[:, 7])
# plt.subplot(VEH_SUBPLOTS, 1, 8)
# plt.plot(bias_state[:, 0], bias_state[:, 8])
# plt.subplot(VEH_SUBPLOTS, 1, 9)
# plt.plot(bias_state[:, 0], bias_state[:, 9])

# plt.show()

STATE_INFO_PLOT_NUM = 6
plt.subplots(STATE_INFO_PLOT_NUM, 1, sharex=True)
plt.subplot(STATE_INFO_PLOT_NUM, 1, 1)
plt.plot(imu_state[:, 0], imu_state[:, 1])
plt.plot(imu_state[:, 0], imu_state[:, 2])
plt.plot(imu_state[:, 0], imu_state[:, 3])
gyro_bias_x = imu_state[-100:-1:10, 1].mean()
gyro_bias_y = imu_state[-100:-1:10, 2].mean()
gyro_bias_z = imu_state[-100:-1:10, 3].mean()
plt.legend(
    [
        "gyro bias x " + "%.3f" % gyro_bias_x,
        "gyro bias y " + "%.3f" % gyro_bias_y,
        "gyro bias z " + "%.3f" % gyro_bias_z,
    ]
)

plt.subplot(STATE_INFO_PLOT_NUM, 1, 2)
plt.plot(gt[:, 0], gt[:, 1])
plt.plot(gt[:, 0], gt[:, 2])
plt.plot(gt[:, 0], gt[:, 3])
gyro_bias_x = gt[-100:-1:10, 1].mean()
gyro_bias_y = gt[-100:-1:10, 2].mean()
gyro_bias_z = gt[-100:-1:10, 3].mean()
plt.legend(
    [
        "gyro bias x gt " + "%.3f" % gyro_bias_x,
        "gyro bias y gt " + "%.3f" % gyro_bias_y,
        "gyro bias z gt " + "%.3f" % gyro_bias_z,
    ]
)

plt.subplot(STATE_INFO_PLOT_NUM, 1, 3)
plt.plot(bias_state[:, 0], bias_state[:, 10])
plt.plot(bias_state[:, 0], bias_state[:, 11])
plt.plot(bias_state[:, 0], bias_state[:, 12])
plt.legend(
    [
        "gyro bias dx",
        "gyro bias dy",
        "gyro bias dz",
    ]
)
plt.subplot(STATE_INFO_PLOT_NUM, 1, 4)
plt.plot(imu_state[:, 0], imu_state[:, 4])
plt.plot(imu_state[:, 0], imu_state[:, 5])
plt.plot(imu_state[:, 0], imu_state[:, 6])
acc_bias_x = imu_state[-100:-1:10, 4].mean()
acc_bias_y = imu_state[-100:-1:10, 5].mean()
acc_bias_z = imu_state[-100:-1:10, 6].mean()
plt.legend(
    [
        "acc bias x " + "%.3f" % acc_bias_x,
        "acc bias y " + "%.3f" % acc_bias_y,
        "acc bias z " + "%.3f" % acc_bias_z,
    ]
)
plt.subplot(STATE_INFO_PLOT_NUM, 1, 5)
plt.plot(bias_state[:, 0], bias_state[:, 13])
plt.plot(bias_state[:, 0], bias_state[:, 14])
plt.plot(bias_state[:, 0], bias_state[:, 15])
plt.legend(
    [
        "acc bias dx",
        "acc bias dy",
        "acc bias dz",
    ]
)
plt.subplot(STATE_INFO_PLOT_NUM, 1, 6)
plt.plot(gt[:, 0], gt[:, 4])
plt.plot(gt[:, 0], gt[:, 5])
plt.plot(gt[:, 0], gt[:, 6])
acc_bias_x = gt[-100:-1:10, 4].mean()
acc_bias_y = gt[-100:-1:10, 5].mean()
acc_bias_z = gt[-100:-1:10, 6].mean()
plt.legend(
    [
        "acc bias x gt " + "%.3f" % acc_bias_x,
        "acc bias y gt " + "%.3f" % acc_bias_y,
        "acc bias z gt " + "%.3f" % acc_bias_z,
    ]
)
plt.show()
