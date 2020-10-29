#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import sys
import copy
import moveit_commander
from moveit_msgs.msg import Grasp
import geometry_msgs.msg
from copy import deepcopy
from math import pi
success=False
scale=1

rospy.init_node('scene_test', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


group = moveit_commander.MoveGroupCommander("arm")
#arm_group.set_named_target("home")
#plan1 = arm_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.515
pose_target.position.y = -0.12
pose_target.position.z = 1.09

print pose_target

group.set_pose_target(pose_target)
plan2 = group.go()

joint_goal = group.get_current_joint_values()
joint_goal_1 = group.get_current_joint_values()

joint_goal[0] = joint_goal_1[0]
joint_goal[1] = joint_goal_1[1]
joint_goal[2] = joint_goal_1[2]
joint_goal[3] = joint_goal_1[3]
joint_goal[4] = pi/4
joint_goal[5] = -pi/12


# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group.stop()

pub = rospy.Publisher('/svh_controller/channel_targets', JointState, queue_size = 10)
rate = rospy.Rate(10) # 10hz
add = 0.0005
temp_position = np.zeros(20)

while not rospy.is_shutdown():
    state = JointState()

    state.name = ['left_hand_Thumb_Flexion', 'left_hand_Thumb_Opposition', 'left_hand_Index_Finger_Distal', 'left_hand_Index_Finger_Proximal', 'left_hand_Middle_Finger_Proximal', 'left_hand_Middle_Finger_Distal', 'left_hand_Ring_Finger', 'left_hand_Pinky', 'left_hand_Finger_Spread']

    state.position = [0.38, 0.66, 0.55, 0.66, 0.50, 0.70, 0.42, 0.55, 0.21]
    pub.publish(state)
    rate.sleep()


rospy.sleep(5)
moveit_commander.roscpp_shutdown()
