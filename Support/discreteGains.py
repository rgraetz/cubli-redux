import numpy as np

# user input control frequency
wc = 2 * np.pi * 15
wi = 2 * np.pi * 0.0001
wlp = 2 * np.pi * 100
P = 125
a = 10.0

Ts = 0.01  # sampling time [sec]

# calculations
s = wc * 1j

if a > 0:
    K = P * abs(1 / ((1 + s / wi) / s * (1 + s * a / wc) / (1 + s / (a * wc))))
else:
    K = P * abs(1 / ((1 + s / wi) / s))

if a > 0:
    if wlp > 1 / Ts * np.pi:

        a0 = (2 * Ts * a * wc * wi + 4 * wi)
        a1 = - 8 * wi
        a2 = -2 * Ts * a * wc * wi + 4 * wi
        a3 = 0

        b0 = K * Ts ** 2 * a * wc * wi + 2 * K * Ts * a ** 2 * wi + 2 * K * Ts * a * wc + 4 * K * a ** 2
        b1 = (2 * K * Ts ** 2 * a * wc * wi - 8 * K * a ** 2)
        b2 = K * Ts ** 2 * a * wc * wi - 2 * K * Ts * a ** 2 * wi - 2 * K * Ts * a * wc + 4 * K * a ** 2
        b3 = 0


    else:
        a0 = (2 * Ts ** 2 * a * wc * wi * wlp + 4 * Ts * a * wc * wi + 4 * Ts * wi * wlp + 8 * wi)
        a1 = (2 * Ts ** 2 * a * wc * wi * wlp - 4 * Ts * a * wc * wi - 4 * Ts * wi * wlp - 24 * wi)
        a2 = (-2 * Ts ** 2 * a * wc * wi * wlp - 4 * Ts * a * wc * wi - 4 * Ts * wi * wlp + 24 * wi)
        a3 = (-2 * Ts ** 2 * a * wc * wi * wlp + 4 * Ts * a * wc * wi + 4 * Ts * wi * wlp - 8 * wi)

        b0 = (K * Ts ** 3 * a * wc * wi * wlp + 2 * K * Ts ** 2 * a ** 2 * wi * wlp + 2 * K * Ts ** 2 * a * wc * wlp + 4 * K * Ts * a ** 2 * wlp)
        b1 = (3 * K * Ts ** 3 * a * wc * wi * wlp + 2 * K * Ts ** 2 * a ** 2 * wi * wlp + 2 * K * Ts ** 2 * a * wc * wlp - 4 * K * Ts * a ** 2 * wlp)
        b2 = (3 * K * Ts ** 3 * a * wc * wi * wlp - 2 * K * Ts ** 2 * a ** 2 * wi * wlp - 2 * K * Ts ** 2 * a * wc * wlp - 4 * K * Ts * a ** 2 * wlp)
        b3 = (K * Ts ** 3 * a * wc * wi * wlp - 2 * K * Ts ** 2 * a ** 2 * wi * wlp - 2 * K * Ts ** 2 * a * wc * wlp + 4 * K * Ts * a ** 2 * wlp)
else:
    b0 = (K * Ts ** 2 * wi * wlp + 2 * K * Ts * wlp)
    b1 = 2 * K * Ts ** 2 * wi * wlp
    b2 = K * Ts ** 2 * wi * wlp - 2 * K * Ts * wlp
    b3 = 0

    a0 = (2 * Ts * wi * wlp + 4 * wi)
    a1 = - 8 * wi
    a2 = -2 * Ts * wi * wlp + 4 * wi
    a3 = 0

# normalize
a1 = a1 / a0
a2 = a2 / a0
a3 = a3 / a0

b0 = b0 / a0
b1 = b1 / a0
b2 = b3 / a0
b3 = b3 / a0

a0 = 1

print("{%f, %f, %f, %f, %f, %f, %f, %f}" % (a0, a1, a2, a3, b0, b1, b2, b3))

a = 1
