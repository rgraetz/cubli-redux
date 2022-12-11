import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# sampling time
Ts = 0.01

# complimentary filter coefficient
C = 0.9

# 2nd order butter worth
wc = 1.0 * 2 * np.pi

b0 = 1.0 * Ts ** 2 * wc ** 2
b1 = 2.0 * Ts ** 2 * wc ** 2
b2 = 1.0 * Ts ** 2 * wc ** 2
b3 = 0.0

a0 = (Ts ** 2 * wc ** 2 + 2.82842712474619 * Ts * wc + 4)
a1 = (2 * Ts ** 2 * wc ** 2 - 8)
a2 = Ts ** 2 * wc ** 2 - 2.82842712474619 * Ts * wc + 4
a3 = 0.0
#
# # 3rd order butter worth
# b0 = 1.0 * Ts ** 3 * wc ** 3
# b1 = 3.0 * Ts ** 3 * wc ** 3
# b2 = 3.0 * Ts ** 3 * wc ** 3
# b3 = 1.0 * Ts ** 3 * wc ** 3
#
# a0 = Ts ** 3 * wc ** 3 + 4 * Ts ** 2 * wc ** 2 + 8 * Ts * wc + 8
# a1 = 3 * Ts ** 3 * wc ** 3 + 4 * Ts ** 2 * wc ** 2 - 8 * Ts * wc - 24
# a2 = 3 * Ts ** 3 * wc ** 3 - 4 * Ts ** 2 * wc ** 2 - 8 * Ts * wc + 24
# a3 = Ts ** 3 * wc ** 3 - 4 * Ts ** 2 * wc ** 2 + 8 * Ts * wc - 8
#
# # controller testing
Ts2 = Ts * Ts
Ts3 = Ts * Ts * Ts
gain = 1000.0
wi = 2 * np.pi * 0.01
wc = 2 * np.pi * 10.0
wlp = 2 * np.pi * 200
a = 3.0
# # lead lag + integral + low pass
# K = gain * wi / a
# a0 = (2 * Ts2 * a * wc * wi * wlp + 4 * Ts * a * wc * wi + 4 * Ts * wi * wlp + 8 * wi)
# a1 = (2 * Ts2 * a * wc * wi * wlp - 4 * Ts * a * wc * wi - 4 * Ts * wi * wlp - 24 * wi)
# a2 = (-2 * Ts2 * a * wc * wi * wlp - 4 * Ts * a * wc * wi - 4 * Ts * wi * wlp + 24 * wi)
# a3 = (-2 * Ts2 * a * wc * wi * wlp + 4 * Ts * a * wc * wi + 4 * Ts * wi * wlp - 8 * wi)
#
# b0 = (K * Ts3 * a * wc * wi * wlp + 2 * K * Ts2 * a * a * wi * wlp + 2 * K * Ts2 * a * wc * wlp + 4 * K * Ts * a * a * wlp)
# b1 = (3 * K * Ts3 * a * wc * wi * wlp + 2 * K * Ts2 * a * a * wi * wlp + 2 * K * Ts2 * a * wc * wlp - 4 * K * Ts * a * a * wlp)
# b2 = (3 * K * Ts3 * a * wc * wi * wlp - 2 * K * Ts2 * a * a * wi * wlp - 2 * K * Ts2 * a * wc * wlp - 4 * K * Ts * a * a * wlp)
# b3 = (K * Ts3 * a * wc * wi * wlp - 2 * K * Ts2 * a * a * wi * wlp - 2 * K * Ts2 * a * wc * wlp + 4 * K * Ts * a * a * wlp)
#
# # lead lag + integral
# K = gain * wi / a
# a0 = 2 * Ts * a * wc * wi + 4 * wi
# a1 = -8 * wi
# a2 = -2 * Ts * a * wc * wi + 4 * wi
# a3 = 0.0
#
# b0 = K * Ts * Ts * a * wc * wi + 2 * K * Ts * a * a * wi + 2 * K * Ts * a * wc + 4 * K * a * a
# b1 = 2 * K * Ts * Ts * a * wc * wi - 8 * K * a * a
# b2 = K * Ts * Ts * a * wc * wi - 2 * K * Ts * a * a * wi - 2 * K * Ts * a * wc + 4 * K * a * a
# b3 = 0.0
#
# lead lag
# K = gain / a
# a0 = Ts * a * wc + 2
# a1 = Ts * a * wc - 2
# a2 = 0.0
# a3 = 0.0
#
# b0 = K * Ts * a * wc + 2 * K * a * a
# b1 = K * Ts * a * wc - 2 * K * a * a
# b2 = 0.0
# b3 = 0.0
#
# # lead lag + low pass
# K = gain / a
# b0 = K * Ts * Ts * a * wc * wlp + 2 * K * Ts * a * a * wlp
# b1 = 2 * K * Ts * Ts * a * wc * wlp
# b2 = K * Ts * Ts * a * wc * wlp - 2 * K * Ts * a * a * wlp
# b3 = 0.0
#
# a0 = Ts * Ts * a * wc * wlp + 2 * Ts * a * wc + 2 * Ts * wlp + 4
# a1 = 2 * Ts * Ts * a * wc * wlp - 8
# a2 = Ts * Ts * a * wc * wlp - 2 * Ts * a * wc - 2 * Ts * wlp + 4
# a3 = 0.0

# # integral + low pass
# K = gain * wi
# b0 = (K * Ts2 * wi * wlp + 2 * K * Ts * wlp)
# b1 = 2 * K * Ts2 * wi * wlp
# b2 = K * Ts2 * wi * wlp - 2 * K * Ts * wlp
# b3 = 0
#
# a0 = (2 * Ts * wi * wlp + 4 * wi)
# a1 = -8 * wi
# a2 = -2 * Ts * wi * wlp + 4 * wi
# a3 = 0
#
# # low pass
# K = gain
# a0 = Ts * wlp + 2
# a1 = Ts * wlp - 2
# a2 = 0.0
# a3 = 0.0
#
# b0 = K * Ts * wlp
# b1 = K * Ts * wlp
# b2 = 0.0
# b3 = 0.0
#
# integral
# wi = 0.1176*10
# gain = 19.24
# K = gain * wi
# a0 = 2 * wi
# a1 = -2 * wi
# a2 = 0.0
# a3 = 0.0
#
# b0 = K * Ts * wi + 2 * K
# b1 = K * Ts * wi - 2 * K
# b2 = 0.0
# b3 = 0.0

# manual gain entry
# test = [0.00030370,-0.00023676,-0.00006694,0.00000000,0.09718814,-0.19069078,0.09350264,0.00000000]
# a0 = test[0]
# a1 = test[1]
# a2 = test[2]
# a3 = test[3]
# b0 = test[4]
# b1 = test[5]
# b2 = test[6]
# b3 = test[7]

# leaky discrete controller test
# wc = 1 * 2 * np.pi
# wlp = 2 * 2 * np.pi
# K = 2.0 / wc ** 2
# b0 = 4 * K * wc * wc
# b1 = - 8 * K * wc * wc
# b2 = 4 * K * wc * wc
# b3 = 0.0
#
# a0 = Ts * Ts * wc * wc + 4 * Ts * wc + 4
# a1 = 2 * Ts * Ts * wc * wc - 8
# a2 = Ts * Ts * wc * wc - 4 * Ts * wc + 4
# a3 = 0.0
#
# b0 = 4 * K * Ts * wc * wc * wlp
# b1 = - 4 * K * Ts * wc * wc * wlp
# b2 = - 4 * K * Ts * wc * wc * wlp
# b3 = + 4 * K * Ts * wc * wc * wlp
#
# a0 = Ts * Ts * Ts * wc * wc * wlp + 2 * Ts * Ts * wc * wc + 4 * Ts * Ts * wc * wlp + 8 * Ts * wc + 4 * Ts * wlp + 8
# a1 = 3 * Ts * Ts * Ts * wc * wc * wlp + 2 * Ts * Ts * wc * wc + 4 * Ts * Ts * wc * wlp - 8 * Ts * wc - 4 * Ts * wlp - 24
# a2 = 3 * Ts * Ts * Ts * wc * wc * wlp - 2 * Ts * Ts * wc * wc - 4 * Ts * Ts * wc * wlp - 8 * Ts * wc - 4 * Ts * wlp + 24
# a3 = Ts * Ts * Ts * wc * wc * wlp - 2 * Ts * Ts * wc * wc - 4 * Ts * Ts * wc * wlp + 8 * Ts * wc + 4 * Ts * wlp - 8

# gyroscope controller - lead lag with high pass
# gain = 1.0
# wc = 2 * np.pi * 7.0
# wlp = 2 * np.pi * 500
# a = 2.0
# whp = 2 * np.pi * 1.0
#
# K = gain / a
# b0 = 2 * K * Ts * a * wc + 4 * K * a * a
# b1 = - 8 * K * a * a
# b2 = -2 * K * Ts * a * wc + 4 * K * a * a
# b3 = 0
#
# a0 = Ts * Ts * a * wc * whp + 2 * Ts * a * wc + 2 * Ts * whp + 4
# a1 = 2 * Ts * Ts * a * wc * whp - 8
# a2 = Ts * Ts * a * wc * whp - 2 * Ts * a * wc - 2 * Ts * whp + 4
# a3 = 0

# butter worth high pass
import scipy
fc = 0.5
wc = fc*2*np.pi
b,a = scipy.signal.butter(2, fc,btype='high', fs=1/Ts)

b0 = b[0]
b1 = b[1]
b2 = b[2]
b3 = 0

a0 = a[0]
a1 = a[1]
a2 = a[2]
a3 = 0

# b0 = 4.0
# b1 = - 8.0
# b2 = + 4.0
# b3 = 0
#
# a0 = (Ts**2*wc**2 + 2.82842712474619*Ts*wc + 4)
# a1 = (2*Ts**2*wc**2 - 8)
# a2 = Ts**2*wc**2 - 2.82842712474619*Ts*wc + 4
# a3 = 0

# simulate time domain response
cycles = 20

frequencies = np.power(10, np.arange(-1, np.log10(1 / Ts / 2.0), 0.01))
mags1 = np.zeros(np.size(frequencies))
phases1 = np.zeros(np.size(frequencies))
mags2 = np.zeros(np.size(frequencies))
phases2 = np.zeros(np.size(frequencies))
for fidx, frequency in enumerate(frequencies):
    try:
        time = np.arange(0, 1.0 / frequency * cycles, Ts)
        inputs = np.sin(2 * np.pi * time * frequency)
        outputs1 = np.zeros(np.size(inputs))
        outputs2 = np.zeros(np.size(inputs))
        _out = np.zeros(np.size(inputs))
        for idx, input in enumerate(inputs):
            if idx > 0:
                outputs1[idx] = C * outputs1[idx - 1] + (1 - C) * inputs[idx]

            if idx > 3:
                outputs2[idx] = (-a1 * outputs2[idx - 1] - a2 * outputs2[idx - 2] - a3 * outputs2[idx - 3] + b0 * inputs[idx] + b1 * inputs[idx - 1] + b2 * inputs[idx - 2] + b3 * inputs[idx - 3]) / a0
                _out[idx] = _out[idx - 1] + outputs2[idx]
                # if abs(input) < 1.0:
                #     _out[idx] = _out[idx] * 0.99
                # outputs2[idx] = _out[idx] - _out[idx-1]

        fftData = sp.fft.fft(outputs1) / len(time) * 2.0
        mags1[fidx] = np.abs(fftData[cycles])
        phases1[fidx] = np.degrees(np.angle(fftData[cycles])) + 90

        fftData = sp.fft.fft(outputs2) / len(time) * 2.0
        mags2[fidx] = np.abs(fftData[cycles])
        phases2[fidx] = np.degrees(np.angle(fftData[cycles])) + 90

        # plt.figure()
        # plt.plot(time, inputs, label='input')
        # plt.plot(time, outputs, label='output')
        # plt.title('Frequency = %0.2f' % frequency)
        # plt.legend()
        # plt.show(block=True)

        # if phases[fidx] > 0:
        #     plt.figure()
        #     plt.plot(time, inputs, label='input')
        #     plt.plot(time, outputs, label='output')
        #     plt.title('Frequency = %0.2f' % frequency)
        #     plt.legend()
        #     plt.show(block=True)
        #
        # plt.figure()
        # plt.semilogx(np.abs(fftData))
        # plt.show(block=True)
    except:
        a = 1

plt.figure()
plt.subplot(2, 1, 1)
plt.loglog(frequencies, mags1)
plt.loglog(frequencies, mags2)
plt.plot(frequencies, 0.5 * np.ones(np.size(frequencies)), 'k--')
plt.plot(frequencies, np.sqrt(2.0) * np.ones(np.size(frequencies)), 'b--')
plt.plot(frequencies, np.ones(np.size(frequencies)), 'c--')

plt.subplot(2, 1, 2)
plt.semilogx(frequencies, phases1)
plt.semilogx(frequencies, phases2)
plt.show(block=True)
