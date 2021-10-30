#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String
# from gomoku_ik.msg import Pose4D
# from gomoku_ik.msg import Joint_msg
from gomoku_ik.msg import new_move
from std_msgs.msg import Float32MultiArray
from gomoku_ik_solver.gomoku_ik_solver import ik_gomoku
import numpy as np
from std_srvs.srv import *


class ik_calc():
    def __init__(self):
        # self.pub = rospy.Publisher('joint_goal', Joint_msg, queue_size=10)
        self.pub = rospy.Publisher('/joint_goal', Float32MultiArray, queue_size=10)
        rospy.Subscriber("/robot_move", Float32MultiArray, self.ik_callback)
        self.ik_object = ik_gomoku()
        self.board_z = 100 #100mm
        self.length_side = 33 # the side of each square is 33mm
        self.x_offset = 84.2
        self.y_offset = -148.5
        self.z_offset = -28.38
        # self.default_position = [55, 155, 110]
        # self.default_position_h = [100, -150, 100]
        # self.intermediate_position_h = [100,-300,100]
        # # self.feeder_position = [55, 155, 120]
        # self.feeder_position_h = [100, -310, 46]
        self.default_position = [20,-100,60]
        self.intermediate_position = [0, -250, 50]
        self.feeder_position = [0, -250, -22]
        print("Initialised")
        # print(self.ik_object.joint_limits)
        # print(self.ik_object.link_lengths)

    def board_pos_to_coord(self,msg):
        x_board = msg.data[0]*self.length_side + self.x_offset
        y_board = msg.data[1]*self.length_side + self.y_offset
        return (x_board,y_board)

    def call_serv(self):
        rospy.wait_for_service('/trigger_movement')
        try:
            trigger_obj = rospy.ServiceProxy('/trigger_movement', Trigger)
            resp1 = trigger_obj()
            return resp1.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def move_to(self, values, gripper_motion, orient = None):
        # coordinates to joint values
        joint_angles_dict = self.coord_to_angles(values, orient)
        joint_angles_list = [joint_angles_dict['joint_1'],joint_angles_dict['joint_2'],joint_angles_dict['joint_3'],joint_angles_dict['gripper_joint']]
        joint_angles_list.append(gripper_motion)

        pub_msg = Float32MultiArray()
        pub_msg.data = joint_angles_list
        # pub_msg.data[0] = calculated_joints['joint_1']
        # pub_msg.data[1] = calculated_joints['joint_2']
        # pub_msg.data[2] = calculated_joints['joint_3']
        # pub_msg.data[3] = calculated_joints['gripper_joint']
        # pub_msg.data[4] = 1 # 1 -- open(drop), 2 -- close(pick), 0 -- wait 
        self.pub.publish(pub_msg)
        return self.call_serv()
        
    def coord_to_angles(self, coord, orient = None):
        x = coord[0]
        y = coord[1]
        z = coord[2]

        self.ik_object.position_endeff['x'] = x
        self.ik_object.position_endeff['y'] = y
        self.ik_object.position_endeff['z'] = z

        self.ik_object.set_orient = orient
        

        return self.ik_object.calculate_joint_values()


    def ik_callback(self,msg):
        x,y = self.board_pos_to_coord(msg)
        z = self.z_offset
        #### Need to trasform global ####
        print("Given pose: x:{0} y:{1} z:{2} ".format(x,y,z))
        ############ TO ADD  ###############
        # Check if the gripper is in the gripping position  (Not needed as it will have to be subscribe to /joint states and then check)
        # go to feeder and grip ---> publish message 
 
        # pub_msg.data[0] = calculated_joints['joint_1']
        # pub_msg.data[1] = calculated_joints['joint_2']
        # pub_msg.data[2] = calculated_joints['joint_3']
        # pub_msg.data[3] = calculated_joints['gripper_joint']
        # pub_msg.data[4] = 1 # 1 -- open(drop), 2 -- close(pick), 0 -- wait 
        # pub_msg = Joint_msg()
        # pub_msg.joint_1 = calculated_joints['joint_1']
        # pub_msg.joint_2 = calculated_joints['joint_2']
        # pub_msg.joint_3 = calculated_joints['joint_3']
        # pub_msg.gripper_joint = calculated_joints['gripper_joint']
        # pub_msg.gripper_motion = 1 # 1 -- open(drop), 2 -- close(pick), 0 -- wait 
        

        # get gripper orient of board position from ik first
        self.ik_object.position_endeff['x'] = x
        self.ik_object.position_endeff['y'] = y
        self.ik_object.position_endeff['z'] = z
        self.ik_object.calculate_joint_values()
        print("X: {0} Y: {1} Z: {2}".format(x,y,z))
        orient = self.ik_object.get_orient()
    
        moved_to_intermediate_initial = self.move_to(self.intermediate_position, 1, orient = orient)
        moved_to_feeder = self.move_to(self.feeder_position, 2, orient = orient)
        if(moved_to_feeder):
            moved_to_intermediate = self.move_to(self.intermediate_position, 2, orient = orient)
            input(
            "============ Press `Enter` to go to the board position ..."
            )
        if moved_to_intermediate:
            # moving to board
            board_position = [x,y,z]
            moved_to_board = self.move_to(board_position, 1)
            # input(
            # "============ Press `Enter` to go to the default position ..."
            # )
        if moved_to_board:
            # moving to default
            moved_to_default = self.move_to(self.default_position, 1)
        if moved_to_default:
            print("Back to home")
            
        ############ TO ADD  ###############
        #Go to feeder and wait         

def main(args=None):
    rospy.init_node('gomoku_ik_node')
    ik_calc()
    rospy.spin()



if __name__ == '__main__':
    main()