#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import os
from scipy.spatial.transform import Rotation as R
import numpy as np
import yaml

class DATALoader:
    def __init__(self, dataset_root_path=None) -> None:
        self._dataset_root_path = dataset_root_path
        self._init_variables()
        self._init_ros_assets()

    def _init_variables(self):
        self.rgb_image = None
        self.depth_image = None
        self.gripper_pose = None
        self.rgb_init = False
        self.depth_init = False
        self.counter = 0

    def _init_ros_assets(self):
        self._rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self._rgb_callback)
        self._depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self._depth_callback)
        self._timer = rospy.Timer(rospy.Duration(1.0/5.0), self._main_loop) # 100 hz
        self._tf_listener = tf.TransformListener()
        self._cv_bridge = CvBridge()

    def _rgb_callback(self, msg):
        # std_msgs/Header header
        # uint32 seq
        # time stamp
        # string frame_id
        # uint32 height
        # uint32 width
        # string encoding
        # uint8 is_bigendian
        # uint32 step
        # uint8[] data"/home/hzx/sample_data_4"
        # print(len(msg.data))
        self.rgb_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if(self.rgb_init == False):
            self.rgb_init = True

    def _depth_callback(self, msg):
        # std_msgs/Header header
        # uint32 seq
        # time stamp
        # string frame_id
        # uint32 height
        # uint32 width
        # string encoding
        # uint8 is_bigendian
        # uint32 step
        # uint8[] data
        self.depth_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if(self.depth_init == False):
            self.depth_init = True

    def _main_loop(self, e):
        if(self.rgb_init * self.depth_init):
            saving_path = self._dataset_root_path + "/data_" + str(self.counter)
            try:
                print("saving frame: "+str(self.counter))
                (trans,rot) = self._tf_listener.lookupTransform('/link1', '/gripper', rospy.Time(0))
                r = R.from_quat(rot).as_matrix()
                pose = np.asarray([[r[0,0], r[0,1], r[0,2], trans[0]],
                                [r[1,0], r[1,1], r[1,2], trans[1]],
                                [r[2,0], r[2,1], r[2,2], trans[2]],
                                [0, 0, 0, 1]])
                import os
                os.makedirs(saving_path)
                np.save(saving_path+"/pose.npy", pose)
                cv2.imwrite(saving_path+"/rgb.png", self.rgb_image)
                cv2.imwrite(saving_path+"/depth.png", self.depth_image)
                self.counter += 1
                rospy.loginfo("saved frame: "+str(self.counter))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("tf error")
                pass

rospy.init_node('corl', anonymous=True)
save_dir = "/home/hzx/sample_data_5"
os.makedirs(save_dir, exist_ok=True)
dl = DATALoader(save_dir)
rospy.spin()