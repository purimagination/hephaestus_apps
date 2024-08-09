import math
import sys
import time
import yaml
import numpy as np
import rospy

class Test:
    def __init__(self) -> None:
        self._read_params()

    def _read_params(self):
        cfg_path = sys.path[0]+'/../config/corl.yaml'
        with open(cfg_path, 'r') as config:
            self.cfg = yaml.safe_load(config)

    def calc_pose(self):
        # 机械臂坐标系下的相机位姿
        camera_2_arm = np.mat([[0.0000000,  0.4787004, -0.8779783, 1.000], 
                               [1.0000000,  0.0000000,  0.0000000, -0.250], 
                               [0.0000000, -0.8779783, -0.4787004, 0.600], 
                               [0, 0, 0, 1]])
        # 机械臂坐标系下的抓取位姿
        grasping_2_arm = self.cfg["T2"]
        # 相机坐标系下的抓取位姿
        grasping_2_camera = np.linalg.inv(camera_2_arm).dot(grasping_2_arm)
        # 相机观测到的相机坐标系下的gripper位姿
        cam_observed_gripper_2_camera = np.mat()
        # tf观测到的相机坐标系下的gripper位姿
        tf_observed_gripper_2_camera = self._tf_listener.lookupTransform("/d435_base", "/gripper", rospy.Time(0))
        # 相机观测到的gripper坐标系下的抓取位姿
        cam_observed_grasping_2_gripper = np.linalg.inv(cam_observed_gripper_2_camera).dot(grasping_2_camera)
        # 在当前机械臂状态下，根据相机观测计算得到的相机坐标系下的抓取位姿
        combined_grasping_2_camera = tf_observed_gripper_2_camera.dot(cam_observed_grasping_2_gripper)
        # 在当前机械臂状态下，根据相机观测计算得到的机械臂坐标系下的抓取位姿
        combined_grasping_2_arm = np.linalg.inv(camera_2_arm).dot(combined_grasping_2_camera)
        
        # 最终把这个位姿态给moveit用来抓取
        return combined_grasping_2_arm

t = Test()