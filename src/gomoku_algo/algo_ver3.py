#!/usr/bin/env python3
import math
from re import X
import rospy
from std_msgs.msg import String
from gomoku_ik.msg import new_move
from std_msgs.msg import Float32MultiArray
import time




import numpy as np

from game.game import Game


# show_animation = True

# import rclpy
# from rclpy.node import Node
# game = np.zeros([2,2])

class algo_class():
    def __init__(self):
        # publisher and subscriber
        self.pub = rospy.Publisher('/robot_move', Float32MultiArray, queue_size=10)
        self.pub_dummy = rospy.Publisher('/dummy_move', String, queue_size=10)
        rospy.Subscriber("/human_move", Float32MultiArray, self.human_callback)
        rospy.init_node('game_node')
        # time.sleep(10)
        # initialize game
        input(
            "============ Press `Enter` to start the game. Human plays first  ..."
            )
        self.pos = None
        self.game = Game(2) # 1 - Ai playes first 2- human first
        # fist move by AI
        # self.play_game(0)


    def human_callback(self,msg):
        size = self.game._board.size
        # row = int(msg.data)%size
        # col = int(int(msg.data)/size)
        row = int(msg.data[0])
        col = int(msg.data[1])
        pos = (col,row)
        print("Your response: row - {0} col - {1}".format(row,col))
        # store human move
        self.pos = pos
        # human moves
        self.play_game(1)
        # AI moves
        self.play_game(0)
        if(self.game_ends()):
            print("Game ends")

        # game[row][col] = 1 
        # print(game)

    def game_ends(self):
        if not self.game._board.victory():
            return False
        elif self.game._board.victory() == 1:
            print("Human wins!!!")
        elif self.game._board.victory() == 2:
            print("AI wins!!!")
        return True
        
    def play_game(self, player):
        if not self.game_ends():
            print("========Turn {}========".format(self.game._turn))
            if player == 0:
                # AI moves
                self.game.play()
                # gives AI move and publish
                AI_move = self.game._board.last_play # (column in Alphabet, row), from top left
                print("robot placed on ({}, {})".format(*AI_move))
                self.publish_move(AI_move)
                
            if player == 1:
                # Human moves
                print("Human placed on ({}, {})".format(*self.pos))
                self.game.play(self.pos)
                self.pos = None
            print(self.game._board)

    
    def publish_move(self, AI_move):
        y = ord(AI_move[0]) - 65
        x = AI_move[1] - 1
        y = 10 - y
        x = 10 - x
        # move = new_move()
        # move.x = x
        # move.y = y
        move  = Float32MultiArray()
        move.data = [x,y]
        # move.data[0] = x
        # move.data[1] = y
        print("publish move ({}, {})".format(x,y))
        msg_dummy = String()
        msg_dummy.data = str(move)
        self.pub.publish(move)
        self.pub_dummy.publish(msg_dummy)

    


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
    
    # print(game)
    algo_class()
    rospy.spin()



if __name__ == '__main__':
    main()
