#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# BEGIN_SUB_TUTORIAL imports
##
# To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
# This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
# and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import math
import time
import trajectory_msgs.msg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_srvs.srv import Trigger,TriggerResponse



try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                print("1: ",abs(actual[index] - goal[index]))
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        print("Current distance to the goal: ",d)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
        
    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        # BEGIN_SUB_TUTORIAL setup
        ##
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).  In this tutorial the group is the primary
        # arm joints in the Panda robot, so we set the group's name to "panda_arm".
        # If you are using a different robot, change this value to the name of your robot
        # arm planning group.
        # This interface can be used to plan and execute motions:
        group_name = "gomoku_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        # rospy.Subscriber("joint_goal", Joint_msg, self.update_joint_goal)
        rospy.Subscriber("joint_goal", Float32MultiArray, self.update_joint_goal)
        rospy.Service('/trigger_movement', Trigger, self.trigger_call_back)
        self.gripper_motion_pub = rospy.Publisher(
            "/gripper_motion",
            Int32,
            queue_size=20,
        )
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        joint_trajectory_publisher = rospy.Publisher(
            "/dynamixel_workbench/joint_trajectory",
            trajectory_msgs.msg.JointTrajectory,
            queue_size=20,
        )

        # END_SUB_TUTORIAL

        # BEGIN_SUB_TUTORIAL basic_info
        ##
        # Getting Basic Information
        # ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        # END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.joint_trajectory_publisher = joint_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.joint_values_global = []
        rospy.spin()
        
    def trigger_call_back(self,req):
        exec_status = self.go_to_joint_state(self.joint_values_global)
        print("Trajectory execution status ",exec_status)
        # while loop to rerun in case
        if(exec_status):
            if(self.joint_values_global[4] == 1 or self.joint_values_global[4] == 2):
                gripper_motion_msg = Int32()
                gripper_motion_msg.data = int(self.joint_values_global[4])
                self.gripper_motion_pub.publish(gripper_motion_msg)
                print("Published gripper to {0}".format(gripper_motion_msg.data))
                time.sleep(1) # Give time for the gripper to grip
            return TriggerResponse(success=True)


    def update_joint_goal(self,msg):
 
        joint_1_angle = msg.data[0]
        joint_2_angle = msg.data[1]
        joint_3_angle = msg.data[2]
        gripper_joint_angle = msg.data[3]

        self.joint_values_global = [joint_1_angle,joint_2_angle,joint_3_angle,gripper_joint_angle, msg.data[4]]

    
    def joint_states_callback(self,data):
        with open('joint_states_test.txt', 'w') as f:
            f.write(str(data))
            f.write("\n\n")



    def go_to_joint_state(self, joint_values):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        # Planning to a Joint Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^^
        # The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        # thing we want to do is move it to a slightly better configuration.
        # We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = joint_values[0]
        joint_goal[1] = joint_values[1]
        joint_goal[2] = joint_values[2]
        joint_goal[3] = joint_values[3]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        if(joint_values[0]<0):
            delay_multiplier = 1.5 # Give more time for the robot to reach the end position
        else:
            delay_multiplier = 1
        param_ret = move_group.plan(joint_goal)
        move_group.go(joint_goal, wait=True)

        # print(type(param_ret))
        # print(type(param_ret.joint_trajectory))
        joint_traj_msg = param_ret.joint_trajectory
        self.joint_trajectory_publisher.publish(joint_traj_msg)
        # print(joint_traj_msg)





        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # END_SUB_TUTORIAL

        # For testing:
        time.sleep(2.5*delay_multiplier) #Replace with while true
        current_joints = move_group.get_current_joint_values()
        return True
        # return all_close(joint_goal, current_joints, 2.5)

    def go_to_pose_goal(self, pose_values):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        # Planning to a Pose Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # We can plan a motion for this group to a desired pose for the
        # end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0.4
        # pose_goal.position.y = 0.1
        # pose_goal.position.z = 0.4
        # print(pose_values)
        quaternion = quaternion_from_euler(
            pose_values[3], pose_values[4], pose_values[5])
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        # pose_goal.orientation.w = pose_values[0]
        pose_goal.position.x = pose_values[0]
        pose_goal.position.y = pose_values[1]
        pose_goal.position.z = pose_values[2]

        move_group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.

        plan = move_group.go(wait=True)
        print(plan)
        with open('joint_traj_test.txt', 'w') as u:
            u.write(str(plan))
            u.write("\n\n")
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        # END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        # Cartesian Paths
        # ^^^^^^^^^^^^^^^
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through. If executing  interactively in a
        # Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        # END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # BEGIN_SUB_TUTORIAL display_trajectory
        ##
        # Displaying a Trajectory
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        ##
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        # print(plan)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        # END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # BEGIN_SUB_TUTORIAL execute_plan
        ##
        # Executing a Plan
        # ^^^^^^^^^^^^^^^^
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        move_group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        # END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        # BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        # Ensuring Collision Updates Are Received
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        # END_SUB_TUTORIAL


def main():
    try:
        # print("")
        # print("----------------------------------------------------------")
        # print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        # print("----------------------------------------------------------")
        # print("Press Ctrl-D to exit at any time")
        # print("")
        # input(
        #     "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        # )
        tutorial = MoveGroupPythonInterfaceTutorial()

        # rospy.spin()

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )

        # tutorial.go_to_joint_state()
        start_pose = [0.2, 0, 0.2, 0, 0, 0]

        print("start pose: ", start_pose)
        pickup_pose = [0.2, 0, 0.3, 0, 0, 0]
        print("goal pose: ", pickup_pose)
        goal_pose = [0.2, 0.1, 0.3, 0, 0, 0]
        print("goal pose: ", goal_pose)
        # joint_angles = [0, 0, 2.075, 1.55]
        joint_angles_2 = [1.0549924946608136, -0.08861922074673423, 2.211011561095715, 1.0192003132408125]
# {'joint_1': 1.0549924946608136, 'joint_2': -0.08861922074673423, 'joint_3': 2.211011561095715, 'gripper_joint': 1.0192003132408125}        
        print("joints goal: ", joint_angles_2)
        # input("============ Press `Enter` to execute a movement using a pose goal ...")

        # from moveit_python_tools.get_ik import GetIK
        # from geometry_msgs.msg import PoseStamped

        # gik = GetIK("gomoku_arm")
        # ps = PoseStamped()
        # ps.header.frame_id = 'base_link'
        # ps.pose.position.x = 0.2
        # ps.pose.position.y = 0.0
        # ps.pose.position.z = 0.2
        # ps.pose.orientation.w = 1.0
        # print(gik.get_ik(ps))

        # input("============ Press `Enter` to execute a movement using a pose goal ...")

        # tutorial.go_to_joint_state(joint_angles_2)
        time.sleep(1)
        # tutorial.go_to_pose_goal(start_pose)
        print("Reached start pose")
        time.sleep(1)
        # tutorial.go_to_pose_goal(pickup_pose)
        # print("Reached pickup pose")
        # time.sleep(1)
        # tutorial.go_to_pose_goal(goal_pose)
        print("Reached goal pose")

        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()

        # input(
        #     "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # )
        # tutorial.display_trajectory(cartesian_plan)

        # input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)

        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

# BEGIN_TUTORIAL
# .. _moveit_commander:
# http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
# .. _MoveGroupCommander:
# http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
# .. _RobotCommander:
# http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
# .. _PlanningSceneInterface:
# http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
# .. _DisplayTrajectory:
# http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
# .. _RobotTrajectory:
# http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
# .. _rospy:
# http://docs.ros.org/noetic/api/rospy/html/
# CALL_SUB_TUTORIAL imports
# CALL_SUB_TUTORIAL setup
# CALL_SUB_TUTORIAL basic_info
# CALL_SUB_TUTORIAL plan_to_joint_state
# CALL_SUB_TUTORIAL plan_to_pose
# CALL_SUB_TUTORIAL plan_cartesian_path
# CALL_SUB_TUTORIAL display_trajectory
# CALL_SUB_TUTORIAL execute_plan
# CALL_SUB_TUTORIAL add_box
# CALL_SUB_TUTORIAL wait_for_scene_update
# CALL_SUB_TUTORIAL attach_object
# CALL_SUB_TUTORIAL detach_object
# CALL_SUB_TUTORIAL remove_object
# END_TUTORIAL
