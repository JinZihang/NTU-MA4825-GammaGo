#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String
# from gomoku_ik.msg import pose_4D
# from gomoku_ik.msg import joint_msg
from std_msgs.msg import Float32MultiArray
from gomoku_ik_solver.gomoku_ik_solver import ik_gomoku
import numpy as np
from std_srvs.srv import *

class ik_calc():
    def __init__(self):
        self.pub = rospy.Publisher('/joint_goal', Float32MultiArray, queue_size=10)
        rospy.Subscriber("/robot_move", Float32MultiArray, self.ik_callback)
        self.ik_object = ik_gomoku()
        self.board_z = 100 #100mm
        self.length_side = 33 # the side of each square is 33mm
        self.x_offset = 84.2
        self.y_offset = -148.5
        self.z_offset = -28.38

        self.default_position = [50,-100,170]
        self.intermediate_position = [50, -255, 160]
        self.intermediate_position_bef_grip = [30, -255, 160]
        self.feeder_position = [30, -255, 110]

        print("Initialised")

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
         # 1 -- open(drop), 2 -- close(pick), 0 -- wait 
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
        print("Given pose: x:{0} y:{1} z:{2} ".format(x,y,z))        

        # get gripper orient of board position from ik first
        self.ik_object.position_endeff['x'] = x
        self.ik_object.position_endeff['y'] = y
        self.ik_object.position_endeff['z'] = z
        self.ik_object.calculate_joint_values()
        print("X: {0} Y: {1} Z: {2}".format(x,y,z))
        orient = self.ik_object.get_orient()
        board_position = [x,y,z]

        intermediate_before_grip = self.intermediate_position_bef_grip
        feeder_position = self.feeder_position
        intermediate = self.intermediate_position
        
        if orient == "h":
            # z
            feeder_position[2] -= 13
            board_position[2] += 20
            # y
            intermediate_before_grip[1] -= 24
            feeder_position[1] -= 24
            intermediate[1] -= 24
            board_position[1] += 33
            # x
            board_position[0] += 36
        intermediate_above_board = [board_position[0],board_position[1],board_position[2]+20]
        moved_to_intermediate_initial = self.move_to(intermediate_before_grip, 1, orient = orient)
        moved_to_feeder = self.move_to(feeder_position, 2, orient = orient)
        if(moved_to_feeder):
            moved_to_intermediate = self.move_to(intermediate, 2, orient = orient)
            input(
            "============ Press `Enter` to go to the board position ..."
            )
        if moved_to_intermediate:
            # moving to board
            
            moved_to_board = self.move_to(board_position, 1,orient=orient)
            # input(
            # "============ Press `Enter` to go to the default position ..."
            # )
        if moved_to_board:
            # moving to default
            if(orient == "h"):
                moved_to_intermediate = self.move_to(intermediate_above_board, 2, orient = orient)
            moved_to_default = self.move_to(self.default_position, 1,orient=orient)
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