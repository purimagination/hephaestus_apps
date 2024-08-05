#!/usr/bin/env python3

import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import shape_msgs.msg
import geometry_msgs.msg
import hephaestus_msgs.srv
from tf.transformations import quaternion_from_euler, quaternion_from_matrix

import math
import sys
import time
import yaml

NUM_JOINTS = 6

def call_gripper(command):
    rospy.wait_for_service('set_mode')
    try:
        set_gripper_mode = rospy.ServiceProxy('set_mode', hephaestus_msgs.srv.SwitchMode)
        result = set_gripper_mode(command)
        return result.message

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    pass

def dist(p, q):
    return math.sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = math.fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= math.cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self) -> None:
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("grasp_and_place", anonymous=True)
        group_name = "hephaestus_arm_group"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

        self._read_params()
        self._add_scene()

    def _read_params(self):
        cfg_path = sys.path[0]+'/../config/corl.yaml'
        with open(cfg_path, 'r') as config:
            self.cfg = yaml.safe_load(config)

    def create_collision_object(self, id, dimensions, pose):
        object = moveit_msgs.msg.CollisionObject()
        object.id = id
        object.header.frame_id = "link1"

        solid = shape_msgs.msg.SolidPrimitive()
        solid.type = solid.BOX
        solid.dimensions = dimensions
        object.primitives = [solid]

        object_pose = geometry_msgs.msg.Pose()
        object_pose.position.x = pose[0]
        object_pose.position.y = pose[1]
        object_pose.position.z = pose[2]

        object.primitive_poses = [object_pose]
        object.operation = object.ADD
        return object

    def _add_scene(self):
        floor_limit = self.create_collision_object(id='floor_limit',
                                            dimensions=[10, 10, 0.05],
                                            pose=[0, 0, -0.5])
        camera_holder_1 = self.create_collision_object(id='camera_holder_1',
                                            dimensions=[0.1, 0.1, 1],
                                            pose=[0.6, 0, 0.5])
        camera_holder_2 = self.create_collision_object(id='camera_holder_2',
                                            dimensions=[0.4, 0.1, 0.1],
                                            pose=[0.35, 0, 0.55])
        table = self.create_collision_object(id='table',
                                            dimensions=[0.15, 0.3, 0.05],
                                            pose=[0.0, -0.25, 0.0])

        # self.scene.add_object(floor_limit)
        # self.scene.add_object(camera_holder_1)
        # self.scene.add_object(camera_holder_2)
        self.scene.add_object(table)

    def go_to_joint_state(self, goals):
        if type(goals) is list:
            move_group = self.move_group
            joint_goal = move_group.get_current_joint_values()
            for i in range(NUM_JOINTS):
                joint_goal[i] = goals[i]
            move_group.go(joint_goal, wait=True)
            move_group.stop()
            current_joints = move_group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 0.01)
        else:
            rospy.logwarn("Invalid Input for Function go_To_joint_state. Type must be List.")
    
    def reach_pose(self, arm, pose, tolerance=0.001):
        arm.set_pose_target(pose)
        arm.set_goal_position_tolerance(tolerance)
        return arm.go(wait=True)

    def go_to_pose_1(self, tolerance=0.02):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id="link1"
        pose.pose.position.x = self.cfg["T1"][0][3]
        pose.pose.position.y = self.cfg["T1"][1][3]
        pose.pose.position.z = self.cfg["T1"][2][3]
        print(pose.pose.position)
        orientation = quaternion_from_matrix(self.cfg["T1"])
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        print(pose.pose.orientation)

        self.move_group.set_pose_target(pose)
        self.move_group.set_goal_position_tolerance(tolerance)
        self.move_group.go(wait=True)

    def go_to_pose_2(self, tolerance=0.02):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id="link1"
        pose.pose.position.x = self.cfg["T2"][0][3]
        pose.pose.position.y = self.cfg["T2"][1][3]
        pose.pose.position.z = self.cfg["T2"][2][3]
        print(pose.pose.position)
        orientation = quaternion_from_matrix(self.cfg["T2"])
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        print(pose.pose.orientation)

        self.move_group.set_pose_target(pose)
        self.move_group.set_goal_position_tolerance(tolerance)
        self.move_group.go(wait=True)

    # def go_to_pose(self, tolerance=0.01):
    #     pose = geometry_msgs.msg.PoseStamped()
    #     pose.header.frame_id="link1"
    #     pose.pose.position.x = 0.16778352936345772
    #     pose.pose.position.y = -0.5291778354911999
    #     pose.pose.position.z = 0.019502880475603557

    #     pose.pose.orientation.x = -0.5
    #     pose.pose.orientation.y = -0.5
    #     pose.pose.orientation.z = -0.5
    #     pose.pose.orientation.w = 0.5

    #     self.move_group.set_pose_target(pose)
    #     self.move_group.set_goal_position_tolerance(tolerance)
    #     self.move_group.go(wait=True)
        
def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()

        rospy.loginfo("Press ENTER to go to pose 1.")
        input("")
        tutorial.go_to_pose_1()

        rospy.loginfo("Press ENTER to go to pose 2.")
        input("")
        tutorial.go_to_pose_2()

        rospy.loginfo("Press ENTER to close gripper")
        input("")
        call_gripper('close')

        rospy.loginfo("Press ENTER to go to pose 3.")
        input("")
        tutorial.go_to_joint_state([0,0,0,0,0,0])

        rospy.loginfo("Press ENTER to open gripper")
        input("")
        call_gripper('open')

        rospy.loginfo("Press ENTER to stop gripper")
        input("")
        call_gripper('stop')

    except rospy.ROSInterruptException:
        return
    
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()