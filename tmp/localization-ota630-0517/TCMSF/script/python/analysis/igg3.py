import numpy as np
import matplotlib.pyplot as plt

# if (err < k0) {
#     w = 1.0;
# } else if (err < k1) {
#     w = k0 / err * ((k1 - err) / (k1 - k0)) * ((k1 - err) / (k1 - k0));
# } else {
#     w = std::numeric_limits<double>::epsilon();
# }


def igg3(err, k0, k1):
    err = abs(err)
    if err < k0:
        w = 1.0
    elif err < k1:
        w = k0 / err * ((k1 - err) / (k1 - k0)) * ((k1 - err) / (k1 - k0))
    else:
        w = 1e-10

    return w


x = np.linspace(-5.0, 5.0, 100)
w = np.zeros_like(x)

for i, x_ in enumerate(x):
    w[i] = igg3(x_, 1, 3)

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(x, w)
plt.subplot(2, 1, 2)
plt.plot(x, np.sqrt(w))
plt.show()
