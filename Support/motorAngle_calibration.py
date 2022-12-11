import os
import matplotlib.pyplot as plt
import numpy as np

motors = ["black", "purple", "silver"]
delim = ","


class Quaternion:
    def __init__(self, angle=0, u_vector=[]):
        if len(u_vector) > 0:
            self.angle = angle
            self.u_vector = u_vector
            self.w = np.cos(angle / 2.0)
            self.x = np.sin(angle / 2.0) * u_vector[0]
            self.y = np.sin(angle / 2.0) * u_vector[1]
            self.z = np.sin(angle / 2.0) * u_vector[2]
        else:
            self.w = 0.0
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    def update(self):
        self.angle = np.arccos(self.w) * 2.0
        u1 = self.x / np.sin(self.angle / 2.0)
        u2 = self.y / np.sin(self.angle / 2.0)
        u3 = self.z / np.sin(self.angle / 2.0)
        self.u_vector = [u1, u2, u3]


class Euler:
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


def EulerToQuaternion(angles):
    cr = np.cos(np.radians(angles.roll) * 0.5)
    sr = np.sin(np.radians(angles.roll) * 0.5)
    cp = np.cos(np.radians(angles.pitch) * 0.5)
    sp = np.sin(np.radians(angles.pitch) * 0.5)
    cy = np.cos(np.radians(angles.yaw) * 0.5)
    sy = np.sin(np.radians(angles.yaw) * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.update()

    return q


def QuaternionToEuler(q):
    angles = Euler()

    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    angles.roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    if (abs(sinp) >= 1):
        angles.pitch = np.sign(sinp) * np.pi / 2
    else:
        angles.pitch = np.arcsin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angles.yaw = np.arctan2(siny_cosp, cosy_cosp)

    angles.roll = angles.roll * 180 / np.pi
    angles.pitch = angles.pitch * 180 / np.pi
    angles.yaw = angles.yaw * 180 / np.pi

    return angles


def Q2E(q):
    angles = Euler()
    angles.yaw = np.arctan2(2 * (q.y * q.w - q.x * q.z), 1 - 2 * (q.y * q.y - q.z * q.z))
    angles.pitch = np.arcsin(2 * (q.x * q.y + q.z * q.w))
    angles.roll = np.arctan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * q.x * q.x - 2 * q.z * q.z)

    angles.roll = angles.roll * 180 / np.pi
    angles.pitch = angles.pitch * 180 / np.pi
    angles.yaw = angles.yaw * 180 / np.pi

    return angles


# build system of linear equations for solution
motor_angles = []
input_angles = []
for mIdx, motor in enumerate(motors):
    fid = open(os.path.join(os.path.dirname(__file__), "data_%s.txt" % motor))
    lines = fid.readlines()
    fid.close()

    rpy_idx = [0, 1, 2]
    roll_idx = 0
    pitch_idx = 1
    yaw_idx = 2
    signs = [-1, 1, 1]
    m1 = [-0.5, 1.0, -0.5]
    m2 = [-1.0, 0.0, 1.0]
    m3 = [1.0, 1.0, 1.0]

    data = []
    for line in lines:
        splitStr = line.split(delim)
        if len(splitStr) > 5:
            data.append([float(s) for s in splitStr[0:6]])
        elif len(splitStr) > 2:
            data.append([float(s) for s in splitStr[0:3]])
    data = np.array(data)

    roll = []
    pitch = []
    yaw = []
    qangle = []
    qvector = []

    data[:, yaw_idx] -= data[0, yaw_idx] + 15 - 90
    for dIdx in range(len(data[:, 0])):
        # euler self test
        rpy = []
        for i in rpy_idx:
            rpy.append(signs[i] * data[dIdx, i] * np.pi / 180.0)

        e = Euler(rpy[0], rpy[1], rpy[2])
        q = EulerToQuaternion(e)
        qangle.append(q.angle * 180.0 / np.pi)
        qvector.append(q.u_vector)

        # define quaternion for roll
        angle = signs[0] * data[dIdx, roll_idx] * np.pi / 180.0
        u_vector = m1.copy()
        u_vector /= np.sqrt(np.sum(np.power(u_vector, 2.0)))
        q_roll = Quaternion(angle, u_vector)
        # define quaternion for pitch
        angle = signs[1] * data[dIdx, pitch_idx] * np.pi / 180.0
        u_vector = m2.copy()
        u_vector /= np.sqrt(np.sum(np.power(u_vector, 2.0)))
        q_pitch = Quaternion(angle, u_vector)
        # define quaternion for yaw
        angle = signs[2] * data[dIdx, yaw_idx] * np.pi / 180.0
        u_vector = m3.copy()
        u_vector /= np.sqrt(np.sum(np.power(u_vector, 2.0)))
        q_yaw = Quaternion(angle, u_vector)
        # quaternion to euler
        e1 = QuaternionToEuler(q_roll)
        e2 = QuaternionToEuler(q_pitch)
        e3 = QuaternionToEuler(q_yaw)

        e1 = Q2E(q_roll)
        e2 = Q2E(q_pitch)
        e3 = Q2E(q_yaw)

        q1 = EulerToQuaternion(e1)
        q2 = EulerToQuaternion(e2)
        q3 = EulerToQuaternion(e3)

        roll.append((e1.roll + e2.roll + e3.roll))
        pitch.append((e1.pitch + e2.pitch + e3.pitch))
        yaw.append((e1.yaw + e2.yaw + e3.yaw))

        a = 1

    # debug plotting
    plt.figure()
    plt.clf()
    plt.plot(data[:, roll_idx], label='roll')
    plt.plot(data[:, pitch_idx], label='pitch')
    plt.plot(data[:, yaw_idx], label='yaw')
    plt.legend()

    plt.show(block=False)

    plt.figure()
    plt.clf()
    plt.plot(roll - roll[0], '--', label='motor roll')
    plt.plot(pitch - pitch[0], '--', label='motor pitch')
    plt.plot(yaw - yaw[0], '-.', label='motor yaw')
    plt.legend()

    plt.figure()
    plt.clf()
    plt.plot((np.array(qangle) - qangle[0]) * 180.0 / np.pi * 2.0)

    # plt.figure()
    # plt.clf()
    # plt.plot(signs[0] * data[:, roll_idx] - 0.75 * signs[1] * data[:, pitch_idx], label='roll')
    # plt.plot(signs[0] * data[:, roll_idx] + signs[1] * data[:, pitch_idx], label='pitch')
    # plt.plot(signs[0] * data[:, roll_idx] + signs[1] * data[:, pitch_idx], label='yaw')
    # plt.legend()

    plt.show(block=True)
    plt.close()

    a = 1
