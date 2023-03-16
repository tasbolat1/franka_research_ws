#! /usr/bin/env python3

# from __future__ import division, print_function

import rospy

# import os
# import time
# import datetime
# import numpy as np


# from std_msgs.msg import Int16
# from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState, Errors as FrankaErrors
# import tf.transformations as tft

from franka_control_wrappers.panda_commander import PandaCommander
# from mvp_grasping.utils import correct_grasp

# import dougsm_helpers.tf_helpers as tfh
from dougsm_helpers.ros_control import ControlSwitcher

# from ggrasp.msg import Grasp

HOME_POSE = [-0.010087330820306798, -0.7763083389326214, 0.0073893375214589855, -2.354554671363115, -0.013121012074334023, 1.570351987068393, 0.78471674188274]
EE_FRAME = "panda_hand_tcp"

class MoveRobot(object):
    """
    Generic Move Group Class to control the robot.
    """

    def __init__(self):
        self.gripper = rospy.get_param("~gripper", "panda")

        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher(
            "/cartesian_velocity_node_controller/cartesian_velocity",
            Twist,
            queue_size=1,
        )
        self.max_velo = 0.10
        self.curr_velo = Twist()

        self.cs = ControlSwitcher(
            {
                "moveit": "effort_joint_trajectory_controller",
                "velocity": "cartesian_velocity_node_controller",
            }
        )
        self.cs.switch_controller("moveit")
        self.pc = PandaCommander(group_name="panda_arm", gripper=self.gripper, ee=EE_FRAME)
        self.robot_state = None
        self.ROBOT_ERROR_DETECTED = False
        self.BAD_UPDATE = False
        rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self.__robot_state_callback,
            queue_size=1,
        )

    def __recover_robot_from_error(self):
        rospy.logerr("Recovering")
        self.pc.recover()
        self.cs.switch_controller("moveit")
        self.pc.goto_saved_pose("start", velocity=0.1)
        rospy.logerr("Done")
        self.ROBOT_ERROR_DETECTED = False

    def __robot_state_callback(self, msg):
        self.robot_state = msg
        if any(self.robot_state.cartesian_collision):
            if not self.ROBOT_ERROR_DETECTED:
                rospy.logerr("Detected Cartesian Collision")
            self.ROBOT_ERROR_DETECTED = True
        for s in FrankaErrors.__slots__:
            if getattr(msg.current_errors, s):
                self.stop()
                if not self.ROBOT_ERROR_DETECTED:
                    rospy.logerr("Robot Error Detected")
                self.ROBOT_ERROR_DETECTED = True


    def stop(self):
        self.pc.stop()
        self.curr_velo = Twist()
        self.curr_velo_pub.publish(self.curr_velo)


    def goto_home(self, velocity=1.0):
        return self.pc.goto_joints(HOME_POSE, velocity=velocity)

    def test(self):

        # print move group info
        self.pc.print_debug_info()
        self.cs.switch_controller("moveit")

        # go home pose
        print('Going home')
        self.goto_home(velocity=0.1)
        
        
        # move using joint states
        print('Going joint pose')
        joints =[-0.01, -0.77, 0.10, -2.25, -0.11, 1.77, 0.88]
        self.pc.goto_joints(joints, velocity=0.1)

        # move using cartesian 
        print('Going cartesian pose')
        pose = [0.35, 0.3, 0.45, 1.0, 0.0, 0.0, 0.0]
        self.pc.goto_pose(pose, velocity=0.1)

        # move using cartesian velocity control
        print('Moving linearly downwards')
        self.cs.switch_controller("velocity")
        v = Twist()
        v.linear.z = -0.05

        rospy.sleep(1)

        # Monitor robot state and descend
        while (
            self.robot_state.O_T_EE[-2] > 0.25
            and not any(self.robot_state.cartesian_contact)
            and not self.ROBOT_ERROR_DETECTED
        ):
            self.curr_velo_pub.publish(v)
            rospy.sleep(0.01)

        
        # Check for collisions
        if self.ROBOT_ERROR_DETECTED:
            self.__recover_robot_from_error()

        rospy.sleep(1)
        self.cs.switch_controller("moveit")

        # close the fingers.
        print('Closing fingers')
        rospy.sleep(0.2)
        self.pc.gripper.grasp(0, force=1)
        
        # open
        print('Opening fingers')
        rospy.sleep(0.2)
        self.pc.gripper.set_gripper(0.1)

        # go home 
        print('Going home pose')
        self.goto_home(velocity=0.1)

        return True
        


if __name__ == "__main__":
    rospy.init_node("move_robot")
    move_robot = MoveRobot()
    if move_robot.test():
        print('Successfully executed all actions!')
        print('Done.')
