import numpy as np
import sympy as sym

angle = 34 * np.pi / 180.0
u_vector = [-0.5, 1.0, -0.5]

angle = -23 * np.pi / 180.0
u_vector = [-1.0, 0.0, 1.0]
u_vector = np.array(u_vector)/np.sqrt(np.sum(np.power(u_vector,2.0)))
qw = np.cos(angle / 2.0)
qx = np.sin(angle / 2.0) * u_vector[0]
qy = np.sin(angle / 2.0) * u_vector[1]
qz = np.sin(angle / 2.0) * u_vector[2]

# to euler
sinr_cosp = 2 * (qw * qx + qy * qz)
cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
roll = np.arctan2(sinr_cosp, cosr_cosp)

sinp = 2 * (qw * qy - qz * qx)
if (abs(sinp) >= 1):
    pitch = np.sign(sinp) * np.pi / 2
else:
    pitch = np.arcsin(sinp)

siny_cosp = 2 * (qw * qz + qx * qy)
cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
yaw = np.arctan2(siny_cosp, cosy_cosp)

# to quaterion
cr = np.cos(roll * 0.5)
sr = np.sin(roll * 0.5)
cp = np.cos(pitch * 0.5)
sp = np.sin(pitch * 0.5)
cy = np.cos(yaw * 0.5)
sy = np.sin(yaw * 0.5)

qw_ = cr * cp * cy + sr * sp * sy
qx_ = sr * cp * cy - cr * sp * sy
qy_ = cr * sp * cy + sr * cp * sy
qz_ = cr * cp * sy - sr * sp * cy

cr = sym.Symbol('cr')
sr = sym.Symbol('sr')
cp = sym.Symbol('cp')
sp = sym.Symbol('sp')
cy = sym.Symbol('cy')
sy = sym.Symbol('sy')

# Quaternion()
qw = cr * cp * cy + sr * sp * sy
qx = sr * cp * cy - cr * sp * sy
qy = cr * sp * cy + sr * cp * sy
qz = cr * cp * sy - sr * sp * cy

2 * (qw * qx + qy * qz)

# cr = np.cos(angles.roll * 0.5)
# sr = np.sin(angles.roll * 0.5)
# cp = np.cos(angles.pitch * 0.5)
# sp = np.sin(angles.pitch * 0.5)
# cy = np.cos(angles.yaw * 0.5)
# sy = np.sin(angles.yaw * 0.5)

# q.update()
