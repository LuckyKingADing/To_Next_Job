import matplotlib.pyplot as plt
import numpy as np


def scale(pos_sd):
    factor = 1.0
    factor = 0.2 / np.exp(-0.15 * pos_sd)
    factor = 0.15 if factor < 0.15 else factor
    factor = 1.0 if factor > 1.0 else factor
    scaled_pos_sd = factor * pos_sd
    scaled_pos_sd = 0.2 if scaled_pos_sd < 0.2 else scaled_pos_sd
    return scaled_pos_sd


px0 = 0.0
px1 = 2.0
px2 = 4.0
px3 = 6.0
px4 = 8.0
px5 = 10.0

py0 = 0.0
py1 = 1.0
py2 = 2.5
py3 = 4.5
py4 = 7.0
py5 = 10.0


def line(x):
    if x < 0.1:
        x = 0.1

    if px1 >= x > px0:
        return py0 + (py1 - py0) / (px1 - px0) * (x - px0)

    if px2 >= x > px1:
        return py1 + (py2 - py1) / (px2 - px1) * (x - px1)

    if px3 >= x > px2:
        return py2 + (py3 - py2) / (px3 - px2) * (x - px2)

    if px4 >= x > px3:
        return py3 + (py4 - py3) / (px4 - px3) * (x - px3)

    if px5 >= x > px4:
        return py4 + (py5 - py4) / (px5 - px4) * (x - px4)

    if x > px5:
        return x


x = np.arange(0.01, 12, 0.01)
scaled = np.arange(0.01, 12, 0.01)
scaled_1 = np.arange(0.01, 12, 0.01)

for i in range(scaled.size):
    scaled[i] = scale(x[i])
    scaled_1[i] = line(x[i])

plt.figure("scale fig")
plt.plot(x, scaled)
plt.plot(x, scaled_1)
plt.plot(x, x)

plt.gca().set_aspect("equal")
plt.show()
