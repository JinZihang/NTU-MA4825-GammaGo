#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String
from gomoku_ik.msg import pose_4D
from gomoku_ik.msg import joint_msg
from gomoku_ik_solver.gomoku_ik_solver import ik_gomoku
import numpy as np

class game_class():
    def __init__(self):
        self.pub = rospy.Publisher('joint_goal', joint_msg, queue_size=10)
        rospy.Subscriber("pose_goal", pose_4D, self.ik_callback)
        self.ik_object = ik_gomoku()
        print(self.ik_object.joint_limits)
        print(self.ik_object.link_lengths)

    def ik_callback(self,msg):
        x = msg.x
        y = msg.y
        z = msg.z
        orient = msg.theta
        print("Given pose: x:{0} y:{1} z:{2} orientation:{3}".format(x,y,z,orient))
        self.ik_object.position_endeff['x'] = x
        self.ik_object.position_endeff['y'] = y
        self.ik_object.position_endeff['z'] = z
        self.ik_object.orientation = orient
        calculated_joints = self.ik_object.calculate_joint_values()
        if(calculated_joints!=None ):
            pub_msg = joint_msg()
            pub_msg.joint_1 = calculated_joints['joint_1']
            pub_msg.joint_2 = calculated_joints['joint_2']
            pub_msg.joint_3 = calculated_joints['joint_3']
            pub_msg.gripper_joint = calculated_joints['gripper_joint']
            self.pub.publish(pub_msg)
        else:
            print("Not published")
        
    # def game_algo():

def quaternion_from_euler(yaw,roll = 0, pitch = 0 ):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def main(args=None):
    rospy.init_node('game_node', anonymous=True)
    game_class()
    rospy.spin()

if __name__ == '__main__':
    main()