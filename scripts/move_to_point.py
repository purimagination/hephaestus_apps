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
group_1 = []
group_1.append([0, 0, 0, 0, 0, 0])
group_1.append([0, 45/180*math.pi, 90/180*math.pi, -30/180*math.pi, 0, 0])
group_1.append([32/180*math.pi, 25/180*math.pi, 127/180*math.pi, 6/180*math.pi, 0, 0])
group_1.append([32/180*math.pi, 45/180*math.pi, 127/180*math.pi, 0/180*math.pi, 0, 0])
group_1.append([0, 0, 0, 0, 0, 0])

# 抓水瓶demo
group_2 = []
group_2.append([0.00000, 0.69994, 1.39994, 0.69987, 0.00000, 0.00000])
group_2.append([0.29803, -0.72531, 2.17089, 2.89615, -0.29816, -0.00019])
group_2.append([0.22573, -0.84446, 1.81459, 2.65896, -0.22578, 0.00000])
group_2.append([0.22325, -0.50386, 1.83074, 2.33462, -0.22329, 0.00000])
group_2.append([-0.04108, -0.53584, 1.77700, 2.31295, -0.04112, 0.00000])
group_2.append([-0.03793, -0.85098, 1.76583, 2.61668, 0.03787, 0.00000])
group_2.append([-0.10869, -0.69929, 2.26987, 2.96844, 0.13987, 0.00000])
group_2.append([0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000])

# 重复定位精度测试2
group_3 = []
group_3.append([0.00000, 0.69994, 1.39994, 0.69987, 0.00000, 0.00000])
group_3.append([-0.22246481478214264, -0.48573803901672363, 1.8941880464553833, 2.37977933883667, 0.22242721915245056, -9.58738019107841e-05])

group_3.append([-0.1963081806898117, -0.5920037627220154, 1.712009072303772, 2.303847312927246, 0.19634954631328583, 9.58738019107841e-05])
group_3.append([-0.22246481478214264, -0.48573803901672363, 1.8941880464553833, 2.37977933883667, 0.22242721915245056, -9.58738019107841e-05])

group_3.append([-0.1963081806898117, -0.5920037627220154, 1.712009072303772, 2.303847312927246, 0.19634954631328583, 9.58738019107841e-05])
group_3.append([-0.22246481478214264, -0.48573803901672363, 1.8941880464553833, 2.37977933883667, 0.22242721915245056, -9.58738019107841e-05])

group_3.append([-0.1963081806898117, -0.5920037627220154, 1.712009072303772, 2.303847312927246, 0.19634954631328583, 9.58738019107841e-05])
group_3.append([-0.22246481478214264, -0.48573803901672363, 1.8941880464553833, 2.37977933883667, 0.22242721915245056, -9.58738019107841e-05])

group_3.append([-0.1963081806898117, -0.5920037627220154, 1.712009072303772, 2.303847312927246, 0.19634954631328583, 9.58738019107841e-05])
group_3.append([-0.22246481478214264, -0.48573803901672363, 1.8941880464553833, 2.37977933883667, 0.22242721915245056, -9.58738019107841e-05])

group_3.append([-0.1963081806898117, -0.5920037627220154, 1.712009072303772, 2.303847312927246, 0.19634954631328583, 9.58738019107841e-05])
group_3.append([-0.22246481478214264, -0.48573803901672363, 1.8941880464553833, 2.37977933883667, 0.22242721915245056, -9.58738019107841e-05])

group_3.append([0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000])

# 重复定位精度测试3
group_4 = []
group_4.append([0.00000, 0.69994, 1.39994, 0.69987, 0.00000, 0.00000])
group_4.append([0.2141895890235901, -0.19861666858196259, 2.3588788509368896, 2.557337760925293, -0.21408618986606598, -9.58738019107841e-05])

group_4.append([0.0786353126168251, -0.21567845344543457, 2.335624933242798, 2.5510101318359375, -0.07861651480197906, 0.0])
group_4.append([0.2141895890235901, -0.19861666858196259, 2.3588788509368896, 2.557337760925293, -0.21408618986606598, -9.58738019107841e-05])

group_4.append([0.0786353126168251, -0.21567845344543457, 2.335624933242798, 2.5510101318359375, -0.07861651480197906, 0.0])
group_4.append([0.2141895890235901, -0.19861666858196259, 2.3588788509368896, 2.557337760925293, -0.21408618986606598, -9.58738019107841e-05])

group_4.append([0.0786353126168251, -0.21567845344543457, 2.335624933242798, 2.5510101318359375, -0.07861651480197906, 0.0])
group_4.append([0.2141895890235901, -0.19861666858196259, 2.3588788509368896, 2.557337760925293, -0.21408618986606598, -9.58738019107841e-05])

group_4.append([0.0786353126168251, -0.21567845344543457, 2.335624933242798, 2.5510101318359375, -0.07861651480197906, 0.0])
group_4.append([0.2141895890235901, -0.19861666858196259, 2.3588788509368896, 2.557337760925293, -0.21408618986606598, -9.58738019107841e-05])

group_4.append([0.0786353126168251, -0.21567845344543457, 2.335624933242798, 2.5510101318359375, -0.07861651480197906, 0.0])
group_4.append([0.2141895890235901, -0.19861666858196259, 2.3588788509368896, 2.557337760925293, -0.21408618986606598, -9.58738019107841e-05])

group_4.append([0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000])


# 重复定位精度测试4
group_5 = []
group_5.append([0.00000, 0.69994, 1.39994, 0.69987, 0.00000, 0.00000])
group_5.append([-0.06812303513288498, -0.4679242968559265, 2.3136603832244873, 2.7815866470336914, 0.06807039678096771, -9.58738019107841e-05])

group_5.append([-0.0681004747748375, -0.5748367309570312, 2.3042044639587402, 2.8789942264556885, 0.06807039678096771, -9.58738019107841e-05])
group_5.append([-0.06812303513288498, -0.4679242968559265, 2.3136603832244873, 2.7815866470336914, 0.06807039678096771, -9.58738019107841e-05])

group_5.append([-0.0681004747748375, -0.5748367309570312, 2.3042044639587402, 2.8789942264556885, 0.06807039678096771, -9.58738019107841e-05])
group_5.append([-0.06812303513288498, -0.4679242968559265, 2.3136603832244873, 2.7815866470336914, 0.06807039678096771, -9.58738019107841e-05])

group_5.append([-0.0681004747748375, -0.5748367309570312, 2.3042044639587402, 2.8789942264556885, 0.06807039678096771, -9.58738019107841e-05])
group_5.append([-0.06812303513288498, -0.4679242968559265, 2.3136603832244873, 2.7815866470336914, 0.06807039678096771, -9.58738019107841e-05])

group_5.append([-0.0681004747748375, -0.5748367309570312, 2.3042044639587402, 2.8789942264556885, 0.06807039678096771, -9.58738019107841e-05])
group_5.append([-0.06812303513288498, -0.4679242968559265, 2.3136603832244873, 2.7815866470336914, 0.06807039678096771, -9.58738019107841e-05])

group_5.append([-0.0681004747748375, -0.5748367309570312, 2.3042044639587402, 2.8789942264556885, 0.06807039678096771, -9.58738019107841e-05])
group_5.append([-0.06812303513288498, -0.4679242968559265, 2.3136603832244873, 2.7815866470336914, 0.06807039678096771, -9.58738019107841e-05])

group_5.append([0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000])


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

        # # 重读定位精度测试1
        # group = group_1
        # for goals in group:
        #     rospy.loginfo("Press ENTER to move the arm.")
        #     input("")
        #     tutorial.go_to_joint_state(goals)



        # # 抓水瓶demo
        # rospy.loginfo("Press ENTER to move the arm.")
        # input("")
        # tutorial.go_to_joint_state(group_2[0])

        # rospy.loginfo("Press ENTER to move the arm.")
        # input("")
        # tutorial.go_to_joint_state(group_2[1])

        # rospy.loginfo("Press ENTER to move the arm.")
        # input("")
        # tutorial.go_to_joint_state(group_2[2])

        # call_gripper('close')
        # time.sleep(2.0)

        # rospy.loginfo("Press ENTER to move the arm.")
        # input("")
        # tutorial.go_to_joint_state(group_2[3])

        # rospy.loginfo("Press ENTER to move the arm.")
        # input("")
        # tutorial.go_to_joint_state(group_2[4])

        # rospy.loginfo("Press ENTER to move the arm.")
        # input("")
        # tutorial.go_to_joint_state(group_2[5])

        # call_gripper('open')
        # time.sleep(2.0)

        # rospy.loginfo("Press ENTER to move the arm.")
        # input("")
        # tutorial.go_to_joint_state(group_2[6])

        # rospy.loginfo("Press ENTER to move the arm.")
        # input("")
        # tutorial.go_to_joint_state(group_2[7])



        # # 重读定位精度测试2
        # group = group_3
        # for goals in group:
        #     rospy.loginfo("Press ENTER to move the arm.")
        #     input("")
        #     tutorial.go_to_joint_state(goals)

        # # 重读定位精度测试3
        # group = group_4
        # for goals in group:
        #     rospy.loginfo("Press ENTER to move the arm.")
        #     input("")
        #     tutorial.go_to_joint_state(goals)

        # 重读定位精度测试4
        group = group_5
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