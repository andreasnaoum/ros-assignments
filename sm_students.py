#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from enum import Enum

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


"""
THINGS TO ASK:

- SOMETIMES MISPLACEMENT OF ROBOT AND TABLES
- MANUAL POSITION NEED ARRAGEMENT

"""


"""
Implement a state machine which goes through the following main states:

    Complete picking task
    Carry cube to second table
    Complete placing task

"""
from enum import Enum
class State(Enum):
    START = 0
    PICK = 1
    CARRY = 2
    PLACE = 3
    FINISH = 4
    ERROR = 5
    

DEBUG = 1 # Set 1 to show debug messages


class StateMachine(object):


    class_name = "State Machine: "


    def __init__(self):
        
        self.node_name = "Student SM"

        rospy.loginfo(self.class_name + "Initialized!")

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        # self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')

        self.pick_srv = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv = rospy.get_param(rospy.get_name() + '/place_srv')

        self.pick_pose_tp = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.place_pose_tp = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        # self.aruco_pose_tp = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        
        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')


        self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')

        rospy.loginfo(self.class_name + "Accessed ROS parameters")

        # Wait for service providers
        
        # rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        
        # rospy.wait_for_service(self.place_srv, timeout=30)
        rospy.loginfo(self.class_name + "Wait for service providers finished")

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.pick_pos_pub = rospy.Publisher(self.pick_pose_tp, PoseStamped, queue_size=10)
        self.place_pos_pub = rospy.Publisher(self.place_pose_tp, PoseStamped, queue_size=10)

        # self.aruco_pos_pub = rospy.Publisher(self.aruco_pose_tp, PoseStamped, queue_size=10)

        self.cube_pos_pub = rospy.Publisher(self.cube_pose, PoseStamped, queue_size=10)
        rospy.loginfo(self.class_name + "Publishers Initialized")

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Init state machine
        self.state = State.START
        rospy.sleep(3)
        self.check_states()


    # 0.50306828716, 0.0245718046511, 0.915538062216, 0.0144467629456, 0.706141958739, 0.707257659069, -0.0306827123383
    def get_pick_pose(self):
        pick_pose = PoseStamped()
        pick_pose.header.stamp = rospy.Time.now()
        pick_pose.header.frame_id = self.robot_base_frame
        pick_pose.pose.position.x = 0.50306828716
        pick_pose.pose.position.y = 0.0245718046511
        pick_pose.pose.position.z = 0.915538062216 - 0.03
        pick_pose.pose.orientation.x = 0.0144467629456
        pick_pose.pose.orientation.y = 0.706141958739
        pick_pose.pose.orientation.z = 0.707257659069
        pick_pose.pose.orientation.w = -0.0306827123383
        return pick_pose

    def tuck_arm(self):
        rospy.loginfo(self.class_name + "Tucking the arm...")
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = True
        self.play_motion_ac.send_goal(goal)
        success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))
        if success_tucking:
            rospy.loginfo(self.class_name +"Arm tucked succesfully!")
            self.state = State.PICK
        else:
            self.play_motion_ac.cancel_goal()
            rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
            self.state = State.ERROR
        rospy.sleep(1)


    def pick(self):
        rospy.loginfo(self.class_name + "Picking cube...")
        # rate = rospy.Rate(10)
        try: 
            rospy.wait_for_service(self.pick_srv, timeout=30)
            pick_pose = self.get_pick_pose()
            rospy.loginfo(self.class_name + "Publishing pose:\n" + str(pick_pose))
            self.pick_pos_pub.publish(pick_pose)
            rospy.sleep(1)
            pick_client = rospy.ServiceProxy(self.pick_srv, SetBool)
            res = pick_client()
            rospy.loginfo("StateMachine: pick request %s", res)
            if res.success == True:
                self.state = State.CARRY
                rospy.loginfo(self.class_name + "Picked the cube successfully")
            else:
                self.state = State.ERROR
                rospy.logerr("Node %s did not pick the cube", self.node_name)

        except rospy.ServiceException as e:
            rospy.logerr("Service call to pick server failed: %s", e)

    def carry(self):
        rospy.loginfo(self.class_name + "Moving to Table 2...")
        move_msg = Twist()
        move_msg.angular.z = -0.5

        rate = rospy.Rate(10)
        converged = False
        cnt = 0
        # rospy.loginfo("StateMachine: %s: Moving towards table", self.node_name)
        rospy.loginfo(self.class_name + "Rotating...")
        while not rospy.is_shutdown() and cnt < 62:
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()
            cnt = cnt + 1

        move_msg.linear.x = 0.5
        move_msg.angular.z = 0
        cnt = 0
        rospy.loginfo(self.class_name + "Moving Forward...")
        while not rospy.is_shutdown() and cnt < 17:
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()
            cnt = cnt + 1

        self.state = State.PLACE
        rospy.sleep(1)

    def place(self):
        rospy.loginfo(self.class_name + "Placing the cube to Table 2...")
        rate = rospy.Rate(10)
        try: 
            rospy.wait_for_service(self.place_srv, timeout=30)
            place_pose = self.get_pick_pose()
            self.place_pos_pub.publish(place_pose)
            rospy.sleep(1)
            place_client = rospy.ServiceProxy(self.place_srv, SetBool)
                    
            res = place_client()

            rospy.loginfo("StateMachine: place request %s", res)

            if res.success == True:
                self.state = State.FINISH
                rospy.loginfo(self.class_name + "Placed the cube successfully")
                # rospy.loginfo("StateMachine: Node %s successfully place the cube", self.node_name)
            else:
                self.state = State.ERROR
                rospy.loginfo("StateMachine: Node %s did not place the cube", self.node_name)

        except rospy.ServiceException as e:
            rospy.logerr("Service call to pick server failed: %s", e)

    def check_states(self):

        

        while not rospy.is_shutdown() and self.state != State.ERROR and self.state != State.FINISH:

            # State 1:  Tuck arm 
            if self.state == State.START:
                self.tuck_arm()

            # State 2:  Pick the cube from Table 1
            elif self.state == State.PICK:
                self.pick()

            # State 3:  Move the robot "manually" to table 2
            elif self.state == State.CARRY:
                self.carry()

            # State 3:  Place the cube to Table 2
            elif self.state == State.PLACE:
                self.place()


        if self.state == State.ERROR:
            rospy.logerr("StateMachine: %s State machine failed. Check your code and try again!", self.node_name)
            return
        
        rospy.loginfo("%s: State machine finished!", self.node_name)
        return

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
