#!/usr/bin/env python

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

# 重复定位精度测试1
group = []
group.append([5.639635128318332e-05, 0.45136258006095886, 2.4546475410461426, 2.0033788681030273, -9.58738019107841e-05, 0.0])
group.append([-0.02293451689183712, -0.23876336216926575, 1.8140150308609009, 1.826395869255066, 0.29998910427093506, 0.10296846181154251])
group.append([0.04216567426919937, -0.7553990483283997, 1.0126830339431763, 1.7791301012039185, 0.4460049271583557, -0.44610080122947693])
group.append([0.40238046646118164, -0.7424128651618958, 1.2770277261734009, 1.8116313219070435, -0.13709953427314758, 0.2521480917930603])
group.append([0.46038973331451416, -0.5211060643196106, 1.7887080907821655, 2.928560972213745, 0.3940413296222687, 0.13940049707889557])
group.append([0.7847740650177002, -0.32041025161743164, 2.03021240234375, 1.7153739929199219, 0.35770514607429504, -0.015243934467434883])
group.append([0.9357132315635681, -0.9793903231620789, 1.0743693113327026, 1.9183388948440552, -0.845798671245575, 0.6613374948501587])
group.append([0.9947414398193359, -0.507465660572052, 1.3732023239135742, 2.2096993923187256, 0.4746711850166321, -0.45223671197891235])
group.append([1.146819829940796, 0.15490198135375977, 2.132319688796997, 1.8965754508972168, -0.5510826110839844, 0.23728765547275543])
group.append([-0.0, 0.9433417916297913, 2.4602231979370117, 1.516915202140808, -9.58738019107841e-05, 0.0])


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
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
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

        for goals in group:
            rospy.loginfo("Press ENTER to move the arm.")
            input("")
            tutorial.go_to_joint_state(goals)

    except rospy.ROSInterruptException:
        return
    
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()