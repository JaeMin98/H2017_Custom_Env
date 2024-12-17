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


class Pick_and_Place():
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        self.object_list = ['cup', 'apple', 'bottle'] #bottle 0.000
        self.obj_6DoF_dict = {'bottle': [-0.8, -0.5, 0.8255, 0, 0, -0.785, 1], 'apple': [-0.8, -0.0, 0.8255, 0, 0, 0, 1], 'cup': [-0.8, 0.51, 0.8255, 0, 0, 0.3025, 1]}
        [self.set_object_6DoF(obj) for obj in self.object_list]
        self.set_move_groups()
        self.set_link_attacher()

    def set_move_groups(self):
        rospy.init_node('demo_attach_links', anonymous=True)
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")

    def set_link_attacher(self):
        self.set_link_attacher_serviece()
        [self.set_link_attacher_request(obj) for obj in self.object_list]


    def set_link_attacher_serviece(self):
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.attach_srv.wait_for_service()
        self.detach_srv.wait_for_service()

    def set_link_attacher_request(self, object):
        for link_name in ['mobile_base','link7', 'link8']:
            attr_name = f'{link_name}_to_{object}'
            setattr(self, attr_name,  AttachRequest())
            setattr(getattr(self, attr_name), 'model_name_1', 'robot')
            setattr(getattr(self, attr_name), 'link_name_1', link_name)
            setattr(getattr(self, attr_name), 'model_name_2', object)
            setattr(getattr(self, attr_name), 'link_name_2', object)
            

    def set_object_6DoF(self, object):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = ModelState(model_name=object)
            
            position, orientation = self.obj_6DoF_dict[object][:3], self.obj_6DoF_dict[object][3:]
            model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z = position
            model_state.pose.orientation.x, model_state.pose.orientation.y, model_state.pose.orientation.z, model_state.pose.orientation.w = orientation
            
            resp = set_model_state(model_state)
            log_func = rospy.loginfo if resp.success else rospy.logerr
            log_func(f"{object} position reset {'successfully' if resp.success else 'failed'}.")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def reset(self):
        self.unprocessed_objects = self.object_list
        for obj in self.object_list:
            req3 = f'mobile_base_to_{obj}'
            self.detach_srv.call(getattr(self, req3))
            self.detach_srv.call(getattr(self, req3))
            self.set_object_6DoF(obj)

    def move_robot_arm(self, x, y, z, roll, pitch, yaw):
        # RPY 값을 쿼터니언으로 변환
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
        print(pose_target)

        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def get_joint_state(self):
        joint_angles = self.arm_group.get_current_joint_values()
        joint_angles_formatted = [round(angle, 3) for angle in joint_angles]
        print("Current Joint Angles:", joint_angles_formatted)
        return joint_angles_formatted
    
    def get_endeffector_position(self):
        current_pose = self.arm_group.get_current_pose().pose
        
        position = current_pose.position
        orientation = current_pose.orientation

        print(f"End-Effector Position (x, y, z): ({position.x:.3f}, {position.y:.3f}, {position.z:.3f})")
        print(f"End-Effector Orientation (w, x, y, z): ({orientation.w:.3f}, {orientation.x:.3f}, {orientation.y:.3f}, {orientation.z:.3f})")

        return [position.x, position.y, position.z], [orientation.w, orientation.x, orientation.y, orientation.z]

    def move_along_the_z_axis(self, value):
        current_pose = self.arm_group.get_current_pose().pose
        target_pose = current_pose
        target_pose.position.z += value

        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True)

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def get_object_values(self, obj):
        obj_dict = {
            "cup": ([-0.018, 0.018], -0.575),
            "apple": ([-0.035, 0.035], -0.59),
            "bottle": ([-0.039, 0.039], -0.31),
            None : (None, None)
        }
        return obj_dict.get(obj, None)

    def performing_command(self, command, object = None):
        req1 = f'link7_to_{object}'
        req2 = f'link8_to_{object}'
        req3 = f'mobile_base_to_{object}'
        grip_effort,release_leveling = self.get_object_values(object)

        if(command == "home"): self.arm_group.go([0, 0, 0, 0, 0, 0], wait=True)

        elif(command == "pick_ready") : self.arm_group.go([0.0, -0.0, -1.572, -0.0, -1.572, -1.572], wait=True)

        elif(command == "pick_cup") :    PP.move_robot_arm(-0.8, 0.51,  1.16,  math.pi, 0,  math.pi/2)
        elif(command == "pick_apple") :  PP.move_robot_arm(-0.8,  0.0, 1.142, -math.pi, 0, -math.pi)
        elif(command == "pick_bottle") : PP.move_robot_arm(-0.8, -0.5,  1.42, -math.pi, 0, -math.pi)

        elif(command == "place_ready"):
            if(object == "cup") : self.arm_group.go([-3.144, -0.014, -1.572, 0, -1.572, -1.572], wait=True)
            else : self.arm_group.go([3.144, -0.014, -1.572, 0, -1.572, -1.572], wait=True)
        elif(command == "place_1") : PP.move_robot_arm(0.67,  0.35, 1.6, math.pi ,0 ,math.pi/2)
        elif(command == "place_2") : PP.move_robot_arm(0.67,   0.0, 1.6, math.pi ,0 ,math.pi/2)
        elif(command == "place_3") : PP.move_robot_arm(0.67, -0.35, 1.6, math.pi ,0 ,math.pi/2)

        elif(command == "hand_open"): self.hand_group.go([0.0, 0.0], wait=True)

        elif(command == "grip") :
            self.hand_group.go([0.0, 0.0], wait=True)
            self.move_along_the_z_axis(-0.15)

            time.sleep(1)
            self.hand_group.go(grip_effort, wait=True)
            time.sleep(1)
            self.attach_srv.call(getattr(self, req1))
            self.attach_srv.call(getattr(self, req2))
            self.move_along_the_z_axis(0.65)
            time.sleep(1)
        
        elif(command == "release"):
            self.move_along_the_z_axis(release_leveling)
            time.sleep(0.5)
            self.detach_srv.call(getattr(self, req1))
            self.detach_srv.call(getattr(self, req2))
            self.hand_group.go([0.0, 0.0], wait=False)
            time.sleep(0.1)
            self.attach_srv.call(getattr(self, req3))
            self.attach_srv.call(getattr(self, req3))
            self.move_along_the_z_axis(0.5)


    def run(self):
        self.reset()

        # self.performing_command("home")
        self.performing_command("hand_open")

        for i in range(3):
            self.performing_command("pick_ready")
            self.performing_command("hand_open")

            while(1):
                target_obj = input("Enter the target object (cup, apple, bottle) (enter 'exit' to exit) : ")

                if(target_obj in self.unprocessed_objects) : break
                elif (target_obj == "reset") : self.reset()
                elif (target_obj == "exit") : sys.exit(0)
                else : print("It's an object that's already been processed")
                

            self.performing_command(f"pick_{target_obj}")
            # sys.exit(0)
            self.performing_command("grip", object=target_obj)
            PP.performing_command("place_ready", object=target_obj)
            self.performing_command(f"place_{i+1}")
            self.performing_command("release", object=target_obj)

            self.unprocessed_objects.remove(target_obj)
        self.performing_command("pick_ready")




if __name__ == '__main__':
    
    PP = Pick_and_Place()
    PP.run()

