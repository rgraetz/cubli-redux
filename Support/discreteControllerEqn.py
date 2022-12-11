import numpy as np
import sympy as sym

## difference equation solver
z = sym.Symbol('z')
Ts = sym.Symbol('Ts')
wi = sym.Symbol('wi')
wc = sym.Symbol('wc')
a = sym.Symbol('a')
K = sym.Symbol('K')
wlp = sym.Symbol('wlp')
whp = sym.Symbol('whp')
w0 = sym.Symbol('w0')
s = (z - 1) / (Ts * z)
s = 2 * (z - 1) / (Ts * (z + 1))
# s = wc*1j

# gyroscope controller - lead lag, high pass
# temp = K * (1 + s * a / wc) / (1 + s / (a * wc)) * s / (s + whp)

# gyroscope controller - lead lag, band pass
# temp = K * (1 + s * a / wc) / (1 + s / (a * wc)) * s / (s + whp) * wlp / (s + wlp)

# lead lag, integral, low pass filter
# temp = K * (1 + s / wi) / s * (1 + s * a / wc) / (1 + s / (a * wc)) * wlp / (s + wlp)

# leaky controller
# temp = K * s * s / ((1 + s / wc) * (1 + s / wc)) * wlp / (s + wlp)

# lead lag, low pass filter
# temp = K * (1 + s * a / wc) / (1 + s / (a * wc)) * wlp / (s + wlp)

# lead lag, integral
# temp = K * (1 + s / wi) / s * (1 + s * a / wc) / (1 + s / (a * wc))

# lead lag
# temp = K * (1 + s * a / wc) / (1 + s / (a * wc))

# low pass filter
# temp = K * wlp / (s + wlp)

# integral
# temp = K * (1 + s / wi) / s

# proportional
# temp = K

# 2nd order butter worth low-pass filter
# temp = 1.0 / ((s/wc) ** 2 + np.sqrt(2) * (s/wc) + 1)

# 3rd order butter worth low-pass filter
# temp = 1.0 / (((s/wc)+1)*((s/wc)**2+(s/wc)+1))

# 2nd order butter worth high-pass filter
s1 = wc**2/s
temp = 1.0 / ((s1/wc) ** 2 + np.sqrt(2) * (s1/wc) + 1)

temp = 1.0 / ((wc/s) ** 2 + np.sqrt(2) * (wc/s) + 1)

num, den = sym.fraction(sym.together(temp))
print("B... z^3 => b0, z^2 => b1, z^1 => b2, z^0 => b3 = ")
print(sym.collect(sym.expand(num), z))
print("A... z^3 => a0, z^2 => a1, z^1 => a2, z^0 => a3 = ")
print(sym.collect(sym.expand(den), z))
