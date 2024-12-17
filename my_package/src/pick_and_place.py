#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import time
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as transformations
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from typing import Dict, List, Tuple, Optional


class Pick_and_Place_controller_module:
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        self.object_list: List[str] = ['cup', 'apple', 'bottle']
        self.object_pose_map: Dict[str, List[float]] = {
            'bottle': [-0.8, -0.5, 0.8255, 0, 0, -0.785, 1],
            'apple':  [-0.8, -0.0, 0.8255, 0, 0,  0.0,   1],
            'cup':    [-0.8,  0.51, 0.8255, 0, 0,  0.3025, 1]
        }
        [self._set_object_pose(obj) for obj in self.object_list]
        self._init_move_groups()
        self._init_link_attachment_services()

    def _init_move_groups(self) -> None:
        rospy.init_node('demo_attach_links', anonymous=True)
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")

    def _init_link_attachment_services(self) -> None:
        self._attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self._detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self._attach_srv.wait_for_service()
        self._detach_srv.wait_for_service()
        [self._register_object_for_attachment(obj) for obj in self.object_list]

    def _register_object_for_attachment(self, obj_name: str) -> None:
        candidate_links = ['mobile_base', 'link7', 'link8']
        for link_name in candidate_links:
            attr_name = f'{link_name}_to_{obj_name}'
            req = AttachRequest()
            req.model_name_1 = 'robot'
            req.link_name_1 = link_name
            req.model_name_2 = obj_name
            req.link_name_2 = obj_name
            setattr(self, attr_name, req)

    def _set_object_pose(self, obj_name: str) -> None:
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = ModelState(model_name=obj_name)
            position = self.object_pose_map[obj_name][:3]
            orientation = self.object_pose_map[obj_name][3:]
            model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z = position
            model_state.pose.orientation.x, model_state.pose.orientation.y, model_state.pose.orientation.z, model_state.pose.orientation.w = orientation
            response = set_model_state(model_state)
            log_func = rospy.loginfo if response.success else rospy.logerr
            log_func(f"{obj_name} position reset {'successfully' if response.success else 'failed'}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def reset(self) -> None:
        self.unprocessed_objects = self.object_list.copy()
        for obj in self.object_list:
            req_base = f'mobile_base_to_{obj}'
            self._detach_srv.call(getattr(self, req_base))
            self._detach_srv.call(getattr(self, req_base))
            self._set_object_pose(obj)

    def move_robot_arm(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        qx, qy, qz, qw = transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = qx
        pose_target.orientation.y = qy
        pose_target.orientation.z = qz
        pose_target.orientation.w = qw
        self.arm_group.set_pose_target(pose_target)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def get_joint_state(self) -> List[float]:
        joint_angles = self.arm_group.get_current_joint_values()
        joint_angles_formatted = [round(angle, 3) for angle in joint_angles]
        print("Current Joint Angles:", joint_angles_formatted)
        return joint_angles_formatted

    def get_end_effector_pose(self) -> Tuple[List[float], List[float]]:
        current_pose = self.arm_group.get_current_pose().pose
        position = current_pose.position
        orientation = current_pose.orientation
        print(f"End-Effector Position (x, y, z): ({position.x:.3f}, {position.y:.3f}, {position.z:.3f})")
        print(f"End-Effector Orientation (w, x, y, z): ({orientation.w:.3f}, {orientation.x:.3f}, {orientation.y:.3f}, {orientation.z:.3f})")
        return [position.x, position.y, position.z], [orientation.w, orientation.x, orientation.y, orientation.z]

    def move_along_z_axis(self, delta_z: float) -> None:
        current_pose = self.arm_group.get_current_pose().pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = current_pose.position.x
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z + delta_z
        target_pose.orientation = current_pose.orientation
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def get_object_values(self, obj: Optional[str]) -> Optional[Tuple[List[float], float]]:
        obj_dict = {
            "cup":    ([-0.018, 0.018], -0.575),
            "apple":  ([-0.035, 0.035], -0.59),
            "bottle": ([-0.039, 0.039], -0.31),
            None:     (None, None)
        }
        return obj_dict.get(obj, None)

    def performing_command(self, command: str, object: Optional[str] = None) -> None:
        link7_req = f'link7_to_{object}'
        link8_req = f'link8_to_{object}'
        base_req = f'mobile_base_to_{object}'
        grip_effort, release_leveling = self.get_object_values(object)
        if command == "home":
            self.arm_group.go([0, 0, 0, 0, 0, 0], wait=True)
        elif command == "pick_ready":
            self.arm_group.go([0.0, -0.0, -1.572, -0.0, -1.572, -1.572], wait=True)
        elif command == "pick_cup":
            self.move_robot_arm(-0.8, 0.51, 1.16, math.pi, 0, math.pi/2)
        elif command == "pick_apple":
            self.move_robot_arm(-0.8, 0.0, 1.142, -math.pi, 0, -math.pi)
        elif command == "pick_bottle":
            self.move_robot_arm(-0.8, -0.5, 1.42, -math.pi, 0, -math.pi)
        elif command == "place_ready":
            if object == "cup":
                self.arm_group.go([-3.144, -0.014, -1.572, 0, -1.572, -1.572], wait=True)
            else:
                self.arm_group.go([3.144, -0.014, -1.572, 0, -1.572, -1.572], wait=True)
        elif command == "place_1":
            self.move_robot_arm(0.67, 0.35, 1.6, math.pi, 0, math.pi/2)
        elif command == "place_2":
            self.move_robot_arm(0.67, 0.0, 1.6, math.pi, 0, math.pi/2)
        elif command == "place_3":
            self.move_robot_arm(0.67, -0.35, 1.6, math.pi, 0, math.pi/2)
        elif command == "hand_open":
            self.hand_group.go([0.0, 0.0], wait=True)
        elif command == "grip":
            if grip_effort is None:
                return
            self.hand_group.go([0.0, 0.0], wait=True)
            self.move_along_z_axis(-0.15)
            time.sleep(1)
            self.hand_group.go(grip_effort, wait=True)
            time.sleep(1)
            self._attach_srv.call(getattr(self, link7_req))
            self._attach_srv.call(getattr(self, link8_req))
            self.move_along_z_axis(0.65)
            time.sleep(1)
        elif command == "release":
            if release_leveling is None:
                return
            self.move_along_z_axis(release_leveling)
            time.sleep(0.5)
            self._detach_srv.call(getattr(self, link7_req))
            self._detach_srv.call(getattr(self, link8_req))
            self.hand_group.go([0.0, 0.0], wait=False)
            time.sleep(0.1)
            self._attach_srv.call(getattr(self, base_req))
            self._attach_srv.call(getattr(self, base_req))
            self.move_along_z_axis(0.5)

    def run(self) -> None:
        self.reset()
        self.performing_command("hand_open")
        for i in range(3):
            self.performing_command("pick_ready")
            self.performing_command("hand_open")
            while True:
                target_obj = input("Enter the target object (cup, apple, bottle) (enter 'exit' to exit) : ")
                if target_obj in self.unprocessed_objects:
                    break
                elif target_obj == "reset":
                    self.reset()
                elif target_obj == "exit":
                    sys.exit(0)
                else:
                    print("It's an object that's already been processed or invalid.")
            self.performing_command(f"pick_{target_obj}")
            self.performing_command("grip", object=target_obj)
            self.performing_command("place_ready", object=target_obj)
            self.performing_command(f"place_{i+1}")
            self.performing_command("release", object=target_obj)
            self.unprocessed_objects.remove(target_obj)
        self.performing_command("pick_ready")

if __name__ == '__main__':
    manipulator = Pick_and_Place_controller_module()
    manipulator.run()
