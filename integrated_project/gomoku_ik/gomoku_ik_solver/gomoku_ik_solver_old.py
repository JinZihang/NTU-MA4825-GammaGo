import numpy as np
import math

class ik_gomoku:

    def __init__(self):
        self.position_endeff = {'x': 0 ,'y': 0,'z':0 }
        self.joint_values={'joint_1': 0, "joint_2": 0,"joint_3": 0,"gripper_joint": 0} #Value angles in accordance to the motors
        self.joint_angles={'joint_1': 0, "joint_2": 0,"joint_3": 0,"gripper_joint": 0} #Calulating joint angles according the equations
        self.orientation = 0 # gripper joint is just a revolute joint
        self.link_lengths = {'link_1': 245, 'link_2': 225, 'link_3': 160}
        self.position_wrist = {'x': 0 ,'y': 0,'z':0 }
        self.gamma = 0
        # joint-1: 60-240 joint-2: 61-285 joint-3: 61-285 gripper_joint: 61-285
        self.joint_limits={'joint_1': (1.0472,4.18879), "joint_2": (1.06465,4.97419),"joint_3": (1.06465,4.97419),"gripper_joint": (1.06465,4.97419)}


    def calculate_joint_1(self):
        if(self.position_endeff['y']!=0 and self.position_endeff['x']!=0):
            self.joint_angles['joint_1'] = math.atan2(self.position_endeff['y'],self.position_endeff['x']) #returns in radians
            return self.joint_angles['joint_1']
        else:
            print("Can't calculate joint_1 angle. Set proper goal endeff position")
            return 
    
    def calculate_wrist_position(self):
        self.position_wrist['x'] = self.position_endeff['x'] - self.link_lengths['link_3']*np.cos(self.orientation)
        self.position_wrist['z'] = self.position_endeff['z'] - self.link_lengths['link_3']*np.sin(self.orientation)
        self.position_wrist['y'] = self.position_endeff['y']        
    
    def calculate_joint_angles(self):
        self.calculate_wrist_position()
        if(self.link_lengths['link_1']==0 or self.link_lengths['link_2'] ==0 or self.link_lengths['link_3']==0):
            print("Set proper link lengths")
            print("Current link lengths: ", self.link_lengths)
            return 
        l1 = self.link_lengths['link_1']
        l2 = self.link_lengths['link_2']
        # l3 = self.link_lengths['link_3']
        z_wrist = self.position_wrist['z']
        x_wrist = self.position_wrist['x']
        
        alpha = math.atan2(z_wrist,x_wrist)

        #Calculating joint angle of link-2
        beta_num = (l1**2)+ (l2**2)-(x_wrist**2)-(z_wrist**2)
        beta = math.acos(beta_num/(2*l1*l2))
        self.joint_angles['joint_3'] = np.pi - beta

        gamma_num = (x_wrist**2) + (z_wrist**2) + (l1**2) - (l2**2)
        gamma_denom = 2*l1*math.sqrt((x_wrist**2) + (z_wrist**2))

        self.gamma = math.acos(gamma_num/gamma_denom)
        self.joint_angles['joint_2'] = alpha - self.gamma

        self.joint_angles['gripper_joint'] = self.orientation - self.joint_angles['joint_2'] - self.joint_angles['joint_3']
        return
        # return self.joint_angles
    
    def elbow_down_joint_angles(self):
        self.calculate_joint_angles()
        self.joint_angles['joint_2'] = self.joint_angles['joint_2'] + 2*self.gamma
        self.joint_angles['joint_3'] = -self.joint_angles['joint_3']
        self.joint_angles['gripper_joint'] = self.joint_angles['gripper_joint'] + 2*self.joint_angles['joint_3'] - 2*self.gamma
        return 
        # return self.joint_angles

    def check_joint_values(self):
        print("Joint angles computed: ",self.joint_angles)
        print("Joint values to be sent to the motor: ",self.joint_values)
        print("Joint limits: ",self.joint_limits)
        if(self.joint_values['joint_1']>self.joint_limits['joint_1'][0] and self.joint_values['joint_1']<self.joint_limits['joint_1'][1]
        and self.joint_values['joint_2']>self.joint_limits['joint_2'][0] and self.joint_values['joint_2']<self.joint_limits['joint_2'][1]        
        and self.joint_values['joint_3']>self.joint_limits['joint_3'][0] and self.joint_values['joint_3']<self.joint_limits['joint_3'][1]
        and self.joint_values['gripper_joint']>self.joint_limits['gripper_joint'][0] and self.joint_values['gripper_joint']<self.joint_limits['gripper_joint'][1]
        ):
            print("Safe")
            return True
        print("Unsafe")
        return False


    def calculate_joint_values(self,alternate_config=False):
        joint_1 = self.calculate_joint_1()
        if(joint_1 != None):
            if(not alternate_config ):
                self.calculate_joint_angles()
            else:
                self.elbow_down_joint_angles()
            self.joint_values['joint_1'] = self.joint_angles['joint_1']+ (150*np.pi/180)
            self.joint_values['joint_2'] = self.joint_angles['joint_2']+ (150*np.pi/180)
            self.joint_values['joint_3'] = self.joint_angles['joint_3']+ (150*np.pi/180)
            self.joint_values['gripper_joint'] = self.joint_angles['gripper_joint']+ (150*np.pi/180)

            safe_flag = self.check_joint_values()
            if(safe_flag):
                return self.joint_values
            else:
                return 
        else:
            return 
            






    

    # def set_position(self,goal_position):
    #     self.position['x'] = goal_position['x']
    #     self.position['y'] = goal_position['y']
    #     self.position['z'] = goal_position['z']
    
    # def set_links(self,given_link_len):
    #     self.link_lengths['link_1'] = given_link_len['link_1']
    #     self.link_lengths['link_2'] = given_link_len['link_2']
    #     self.link_lengths['link_3'] = given_link_len['link_3']

    # def set_orientation(self,goal_orient):
    #     self.orientation = goal_orient

    # def get_position(self):
    #     return self.position

    # def get_links(self):
    #     return self.link_lengths

    # def get_orientation(self):
    #     return self.orientation

    

# gomoku_test = ik_gomoku()

# pos = {'x': math.sqrt(3) ,'y': 1.0, 'z': 0}
# # gomoku_test.set_position(pos)
# gomoku_test.position = pos
# print(gomoku_test.position)
# print(gomoku_test.calculate_joint_1())

