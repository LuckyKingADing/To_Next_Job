import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


bias_state = np.array(pd.read_csv("data/tmp/bias_esti_state.csv"))
# bias_state = bias_state[0:-1:10, :]

plt.rcParams["legend.loc"] = "upper left"


def scope_by_time(left_t, right_t, data):
    timestamp = data[:, 0]
    condition_ = (timestamp > left_t) & (timestamp < right_t)
    return data[condition_, :]


# #
# LEFT_T_1 = 1762324697.7169
# RIGHT_T_1 = 1762324727.017

# LEFT_T_2 = 1762325224.6179
# RIGHT_T_2 = 1762325300.0181

# LEFT_T_3 = 1762324143.9156
# RIGHT_T_3 = 1762324205.1158

# 2025-11-04_09-25-45
LEFT_T_1 = 1762221913.3176
RIGHT_T_1 = 1762221949.4178

LEFT_T_2 = 1762221360.8132
RIGHT_T_2 = 1762221422.0139

LEFT_T_3 = 1762222456.12
RIGHT_T_3 = 1762222530.9202

# bias_state = np.concatenate(
#     (
#         scope_by_time(LEFT_T_1, RIGHT_T_1, bias_state),
#         # scope_by_time(LEFT_T_2, RIGHT_T_2, bias_state),
#         # scope_by_time(LEFT_T_3, RIGHT_T_3, bias_state),
#     ),
#     axis=0,
# )

# NUM = 6
# plt.subplots(NUM, 1, sharex=True, sharey=True)
# plt.subplot(NUM, 1, 1)
# plt.scatter(bias_state[:, 0], bias_state[:, 16], c=bias_state[:, 22], s=3)
# plt.ylim([-0.1, 0.1])
# plt.subplot(NUM, 1, 2)
# plt.scatter(bias_state[:, 0], bias_state[:, 17], c=bias_state[:, 22], s=3)
# plt.subplot(NUM, 1, 3)
# plt.scatter(bias_state[:, 0], bias_state[:, 18], c=bias_state[:, 22], s=3)
# plt.subplot(NUM, 1, 4)
# plt.scatter(bias_state[:, 0], bias_state[:, 19], c=bias_state[:, 22], s=3)
# plt.subplot(NUM, 1, 5)
# plt.scatter(bias_state[:, 0], bias_state[:, 20], c=bias_state[:, 22], s=3)
# plt.subplot(NUM, 1, 6)
# plt.scatter(bias_state[:, 0], bias_state[:, 21], c=bias_state[:, 22], s=3)
# # plt.show()

condition = (
    (bias_state[:, 22] == 6)
    & (np.abs(bias_state[:, 16]) < 0.3)
    & (np.abs(bias_state[:, 17]) < 0.3)
)

pos_mean = np.mean(
    bias_state[
        condition,
        16:18,
    ],
    axis=0,
)

print(pos_mean)

RANGE = 0.3
plt.figure("inno")
plt.scatter(bias_state[:, 16], bias_state[:, 17], c=bias_state[:, 22], s=3)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.10, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.09, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.08, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.07, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.06, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.05, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.04, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.03, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.02, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle((0, 0), 0.01, fill=False, edgecolor="black", linewidth=1)
)
plt.gca().add_artist(
    plt.Circle(
        (pos_mean[0], pos_mean[1]),
        0.002,
        fill=True,
        facecolor="red",
        edgecolor="red",
        linewidth=1,
    )
)
plt.gca().add_artist(
    plt.Circle(
        (0, 0),
        0.002,
        fill=True,
        facecolor="black",
        edgecolor="black",
        linewidth=1,
    )
)
plt.xlim([-RANGE, RANGE])
plt.ylim([-RANGE, RANGE])

plt.gca().set_aspect("equal")
plt.show()
