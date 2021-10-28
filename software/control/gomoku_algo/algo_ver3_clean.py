#!/usr/bin/env python3
import math
from re import X
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import time




import numpy as np

from game.game import Game


class algo_class():
    def __init__(self):
        # publisher and subscriber
        self.pub = rospy.Publisher('/robot_move', Float32MultiArray, queue_size=10)
        rospy.Subscriber("/human_move", Float32MultiArray, self.human_callback)
        rospy.init_node('game_node')
        self.player_1 = "Human"
        self.player_2 = "Robot"

        player_input = input(
            "============ Press `h` if you want to play FIRST\n\n  OR  \n\n ============ Press 'r' if you want the robot to play FIRST... "
            )
        self.pos = None
        if(player_input == 'h'):
            self.game = Game(2) # 1 - AI playes first 2- human first
            self.player_1 = "Human"
            self.player_2 = "Robot"
        elif(player_input == 'r'):
            # fist move by AI
            self.game = Game(1)
            self.play_game(0)
            self.player_1 = "Robot"
            self.player_2 = "Human"
# 

    def human_callback(self,msg):
        # Float32Array msg  ---> [x,y]
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


    def game_ends(self):
        if not self.game._board.victory():
            return False
        elif self.game._board.victory() == 1:
            print("{} wins!!!".format(self.player_1))
        elif self.game._board.victory() == 2:
            print("{} wins!!!".format(self.player_2))
        return True
        
    def play_game(self, player):
        if not self.game_ends():
            print("========Turn {}========".format(self.game._turn))
            if player == 0:
                # AI moves
                self.game.play()
                # gives AI move and publishes /robot_move
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
        y = ord(AI_move[0]) - 65 # ord('A') is 65
        x = AI_move[1] - 1 # To start with 0 column instead of 1 column
        #Transforming the x,y with respect to the origin of the board which is rotated 180 deg about z and tranlated to (10,10)
        y = 10 - y 
        x = 10 - x
 
        move  = Float32MultiArray()
        move.data = [x,y]
        print("published move ({}, {})".format(x,y))


def main(args=None):
    
    # print(game)
    algo_class()
    rospy.spin()



if __name__ == '__main__':
    main()
