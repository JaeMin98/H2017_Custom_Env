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
        self.object_list = ['cube', 'triangular_pillar', 'cylinder']
        self.obj_6DoF_dict = {'cube': [-0.8, -0.5, 0.8255, 0, 0, 0, 1], 'triangular_pillar': [-0.8, -0.0, 0.8255, 0, 0, 0, 1], 'cylinder': [-0.8, 0.51, 0.8255, 0, 0, 0, 1]}
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

    def get_joint_state(self):
        joint_angles = self.arm_group.get_current_joint_values()
        joint_angles_formatted = [round(angle, 3) for angle in joint_angles]
        print("Current Joint Angles:", joint_angles_formatted)
        return joint_angles_formatted
    
    def get_endeffector_position(self):
        current_pose = self.arm_group.get_current_pose().pose
        current_position = current_pose.position
        print("Current End-Effector Position (x, y, z):")
        print(f"x: {current_position.x:.3f}, y: {current_position.y:.3f}, z: {current_position.z:.3f}")
        return [current_position.x, current_position.y, current_position.z]
    
    def move_along_the_z_axis(self, value):
        current_pose = self.arm_group.get_current_pose().pose
        target_pose = current_pose
        target_pose.position.z += value

        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True)

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def performing_command(self, command, object = None):
        req1 = f'link7_to_{object}'
        req2 = f'link8_to_{object}'
        req3 = f'mobile_base_to_{object}'

        if(command == "home"): self.arm_group.go([0, 0, 0, 0, 0, 0], wait=True)

        elif(command == "pick_ready") : self.arm_group.go([0.0, -0.0, -1.572, -0.0, -1.572, -1.572], wait=True)
        elif(command == "pick_cube") : self.arm_group.go([0.567, -0.560, -1.837, 0.001, -0.746, -1.000], wait=True)
        elif(command == "pick_triangular_pillar") : self.arm_group.go([0.011, -0.445, -2.058, -0.003, -0.643, -1.549], wait=True)
        elif(command == "pick_cylinder") : self.arm_group.go([-0.557, -0.571, -1.848, -0.005, -0.741, -2.109], wait=True)

        elif(command == "place_ready") : self.arm_group.go([3.132, -0.275, -2.343, 0, -0.523, 1.56], wait=True)
        elif(command == "place_1") : self.arm_group.go([-2.701, -0.490, -2.156, 0.002, -0.493, 2.031], wait=True)
        elif(command == "place_2") : self.arm_group.go([3.153, -0.448, -2.263, 0.003, -0.430, 1.585], wait=True)
        elif(command == "place_3") : self.arm_group.go([2.752, -0.486, -2.182, 0.003, -0.474, 1.183], wait=True)

        elif(command == "hand_open"): self.hand_group.go([0.0, 0.0], wait=True)

        elif(command == "grip") :
            self.hand_group.go([0.0, 0.0], wait=True)
            self.move_along_the_z_axis(-0.19)

            self.hand_group.go([-0.026, 0.026], wait=True)
            self.attach_srv.call(getattr(self, req1))
            self.attach_srv.call(getattr(self, req2))
            self.move_along_the_z_axis(0.19)
        
        elif(command == "release"):
            self.move_along_the_z_axis(-0.06)
            time.sleep(0.5)
            self.detach_srv.call(getattr(self, req1))
            self.detach_srv.call(getattr(self, req2))
            self.hand_group.go([0.0, 0.0], wait=False)
            time.sleep(0.1)
            self.attach_srv.call(getattr(self, req3))
            self.attach_srv.call(getattr(self, req3))
            self.move_along_the_z_axis(0.06)


    def run(self):
        self.reset()

        # self.performing_command("home")
        self.performing_command("hand_open")

        for i in range(3):
            self.performing_command("pick_ready")
            self.performing_command("hand_open")

            while(1):
                target_obj = input("Enter the target object (cube, triangular_pillar, cylinder) (enter 'exit' to exit) : ")

                if(target_obj in self.unprocessed_objects) : break
                elif (target_obj == "reset") : self.reset()
                elif (target_obj == "exit") : sys.exit(0)
                else : print("It's an object that's already been processed")
            
                

            self.performing_command(f"pick_{target_obj}")
            self.performing_command("grip", object=target_obj)
            # self.performing_command("place_ready")
            self.performing_command(f"place_{i+1}")
            self.performing_command("release", object=target_obj)

            self.unprocessed_objects.remove(target_obj)




if __name__ == '__main__':
    Pick_and_Place().run()

