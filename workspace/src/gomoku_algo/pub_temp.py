#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String
from gomoku_ik.msg import new_move



import numpy as np

from game.game import Game


# show_animation = True

# import rclpy
# from rclpy.node import Node
# game = np.zeros([2,2])

class game_class():
    def __init__(self):
        # publisher and subscriber
        self.pub = rospy.Publisher('new_move', new_move, queue_size=10)
        rospy.Subscriber("human_move", new_move, self.human_callback)

        # initialize game
        self.pos = None
        self.game = Game(1) # 1 - Ai playes first
        # fist move by AI
        self.play_game(0)


    def human_callback(self,msg):
        size = self.game._board.size
        # row = int(msg.data)%size
        # col = int(int(msg.data)/size)
        row = msg.x
        col = msg.y
        pos = (row,col)
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
            print("AI wins!!!")
        elif self.game._board.victory() == 2:
            print("Human wins!!!")
        return True
        
    def play_game(self, player):
        if not self.game_ends():
            print("========Turn {}========".format(self.game._turn))
            if player == 0:
                # AI moves
                self.game.play()
                # gives AI move and publish
                AI_move = self.game._board.last_play
                print("robot placed on ({}, {})".format(*AI_move))
                self.publish_move(AI_move)
            if player == 1:
                # Human moves
                print("Human placed on ({}, {})".format(*self.pos))
                self.game.play(self.pos)
                self.pos = None
            print(self.game._board)

    
    def publish_move(self, AI_move):
        pub_msg = String()
        pub_msg.data = str(AI_move)
        self.pub.publish(pub_msg)

    


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
    # print(game)
    game_class()
    rospy.spin()



if __name__ == '__main__':
    main()
