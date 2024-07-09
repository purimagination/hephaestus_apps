#!/usr/bin/env python3

import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import geometry_msgs.msg
import hephaestus_msgs.srv

import math
import sys
import time

NUM_JOINTS = 6

goals = []
goals.append([0.00000, 0.69994, 1.39994, 0.69987, 0.00000, 0.00000])
goals.append([0.29803, -0.72531, 2.17089, 2.89615, -0.29816, -0.00019])
goals.append([0.22573, -0.84446, 1.81459, 2.65896, -0.22578, 0.00000])
goals.append([0.22325, -0.50386, 1.83074, 2.33462, -0.22329, 0.00000])
goals.append([-0.04108, -0.53584, 1.77700, 2.31295, -0.04112, 0.00000])
goals.append([-0.03793, -0.85098, 1.76583, 2.61668, 0.03787, 0.00000])
goals.append([-0.10869, -0.69929, 2.26987, 2.96844, 0.13987, 0.00000])
goals.append([0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000])

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
        move_group = moveit_commander.MoveGroupCommander(group_name)

        self.move_group = move_group

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
    
def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()

        # 抓水瓶demo
        rospy.loginfo("Press ENTER to move the arm.")
        input("")
        tutorial.go_to_joint_state(goals[0])

        rospy.loginfo("Press ENTER to move the arm.")
        input("")
        tutorial.go_to_joint_state(goals[1])

        rospy.loginfo("Press ENTER to move the arm.")
        input("")
        tutorial.go_to_joint_state(goals[2])

        call_gripper('close')
        time.sleep(2.0)

        rospy.loginfo("Press ENTER to move the arm.")
        input("")
        tutorial.go_to_joint_state(goals[3])

        rospy.loginfo("Press ENTER to move the arm.")
        input("")
        tutorial.go_to_joint_state(goals[4])

        rospy.loginfo("Press ENTER to move the arm.")
        input("")
        tutorial.go_to_joint_state(goals[5])

        call_gripper('open')
        time.sleep(2.0)

        rospy.loginfo("Press ENTER to move the arm.")
        input("")
        tutorial.go_to_joint_state(goals[6])

        rospy.loginfo("Press ENTER to move the arm.")
        input("")
        tutorial.go_to_joint_state(goals[7])

    except rospy.ROSInterruptException:
        return
    
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()