import numpy as np

class RPY:
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class RotationMatrix:
    def __init__(self, data_00=1, data_01=0, data_02=0, 
                       data_10=0, data_11=1, data_12=0, 
                       data_20=0, data_21=0, data_22=1):
        self.matrix = np.matrix([[data_00, data_01, data_02],
                                 [data_10, data_11, data_12],
                                 [data_20, data_21, data_22]])

class Rotation:
    def __init__(self) -> None:
        self._init_variables()

    def _init_variables(self):
        self._euler_angles = RPY()
        self._quaternion = Quaternion()
        self._rotation_matrix = RotationMatrix()

    def from_euler_angles(self, roll, pitch, yaw):
        # Euler Angles
        self._euler_angles = RPY(roll=roll, pitch=pitch, yaw=yaw)

        # Quaternion
        self._quaternion = self._euler_to_quaternion(self._euler_angles)

        # Rotation Matrix

    def _euler_to_quaternion(self, euler_angles=RPY()):
        roll = euler_angles.roll
        pitch = euler_angles.pitch
        yaw = euler_angles.yaw

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        quaternion = Quaternion(x=qx, y=qy, z=qz, w=qw)

        return quaternion