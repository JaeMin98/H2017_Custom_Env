#!/usr/bin/env python3

import sys
import rospy
import math
import moveit_commander
import geometry_msgs.msg
import tf.transformations as transformations
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import time

def reset_cube_position():
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = 'cube'
        model_state.pose.position.x = -0.8
        model_state.pose.position.y = -0.5
        model_state.pose.position.z = 0.826
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0

        resp = set_model_state(model_state)
        if resp.success:
            rospy.loginfo("Cube position reset successfully.")
        else:
            rospy.logerr("Failed to reset cube position.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def move_group_to_pose_goal():
    moveit_commander.roscpp_initialize(sys.argv)
    reset_cube_position()
    rospy.init_node('demo_attach_links', anonymous=True)
    arm_group = moveit_commander.MoveGroupCommander("arm")
    hand_group = moveit_commander.MoveGroupCommander("hand")

    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    req1 = AttachRequest()
    req1.model_name_1 = 'robot'
    req1.link_name_1 = 'link7'
    req1.model_name_2 = 'cube'
    req1.link_name_2 = 'cube'

    req2 = AttachRequest()
    req2.model_name_1 = 'robot'
    req2.link_name_1 = 'link8'
    req2.model_name_2 = 'cube'
    req2.link_name_2 = 'cube'

    hand_group.go([0, 0], wait=True)
    arm_group.go([0, 0, 0, 0, 0, 0], wait=True)

    arm_group.go([0.5585938521567169, -0.5585345140036804, -1.8500669821229945, -0.00017259988586282304, -0.7329569994915106, 0.5585019767871504], wait=True)

    arm_group.go([0.558579991333513, -0.5585649924317533, -1.8499887792720724, -0.00016740685505745034, -0.7329500177326009, 0.5585250198174698], wait=True)

    arm_group.go([0.5692976058145103, -0.5571501636261553, -1.8415661005945125, 0.00012667305526115058, -0.7435365911724343, -1.0017043592415176], wait=True)

    arm_group.go([0.5692168989114927, -0.7565584985244413, -1.867553945695339, -0.0007817741851718907, -0.5168469374962044, -1.001389190876739], wait=True)

    hand_group.go([-0.0399, 0.0399], wait=True)
    attach_srv.call(req1)
    attach_srv.call(req2)

    hand_group.go([-0.0399, 0.0399], wait=False)
    arm_group.go([0.5692708287252648, -0.5571687281831608, -1.8415445948140263, -0.00013987072257748423, -0.743963066023003, -1.002759374912391], wait=True)
    hand_group.go([-0.0399, 0.0399], wait=False)
    arm_group.go([3.132, -0.275, -2.343, 0, -0.523, 1.56], wait=True)

    arm_group.go([3.132, -0.510, -2.379, 0, -0.253, 1.56], wait=True)

    
    detach_srv.call(req1)
    detach_srv.call(req2)
    hand_group.go([0, 0], wait=True)
    # time.sleep(1)

    arm_group.go([3.132, -0.275, -2.343, 0, -0.523, 1.56], wait=True)

    arm_group.go([0, 0, 0, 0, 0, 0], wait=True)
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_group_to_pose_goal()
    except rospy.ROSInterruptException:
        pass
