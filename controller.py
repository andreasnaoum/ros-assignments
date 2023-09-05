#!/usr/bin/env python3

"""

Author: Andreas Naoum 
Email: anaoum@kth.se

Pseudoce for the assignment:

while True:
    path, gain = get_path_from_action_server()
    if path is empty:
        exit() # Done
    while path is not empty:
      path, setpoint = get_updated_path_and_setpoint_from_service(path)
      setpoint_transformed = transform_setpoint_to_robot_frame(setpoint)
      publish(setpoint_transformed)
      sleep()
"""

import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot
import math

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0

"""
def move(path):
    global control_client, robot_frame_id, pub

    # Call service client with path
    res = control_client(path)

    # Transform Setpoint from service client
    transform = tf_buffer.lookup_transform(robot_frame_id, "map", rospy.Time())

    # Create Twist message from the transformed Setpoint

    # Publish Twist

    # Call service client again if the returned path is not empty and do stuff again

    # Send 0 control Twist to stop robot

    # Get new path from action server
"""

# ---------------- Supportive Functions ----------------

def transform_setpoints(robot_frame_id, setpoint):
    # try:
    #     transform = tf_buffer.lookup_transform(robot_frame_id,setpoint.header.frame_id,rospy.Time())
    #     transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)
    # except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rate.sleep()
    #     continue
    transform = tf_buffer.lookup_transform(robot_frame_id, "map", rospy.Time()) 
    transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)
    return transformed_setpoint

def create_twist(transformed_setpoint):
    msg = Twist()
    msg.angular.z = math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    if msg.angular.z > max_angular_velocity:
        msg.angular.z = max_angular_velocity
    if msg.linear.x > max_linear_velocity:
        msg.linear.x = max_linear_velocity
    # msg.linear.x = 0.5 * math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
    # msg.angular.z = 4 * math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    # msg.linear.x = 0.5 * math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
    # limit_velocities(msg)
    return msg

def limit_velocities(msg):
    if msg.angular.z > max_angular_velocity:
        msg.angular.z = max_angular_velocity
    if msg.linear.x > max_linear_velocity:
        msg.linear.x = max_linear_velocity


def move(path):
    global control_client, robot_frame_id, pub

    while path.poses:
        # # Call service client with path
        res = control_client(path)
        setpoint = res.setpoint
        new_path = res.new_path
        # rospy.loginfo("The frame is: %s", res.setpoint.header.frame_id)

        # Transform Setpoint from service client
        try:
            transform = tf_buffer.lookup_transform(robot_frame_id,setpoint.header.frame_id,rospy.Time())
            transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        # transformed_setpoint = transform_setpoints(robot_frame_id, res.setpoint)
        
        # Create Twist message from the transformed Setpoint
        msg = create_twist(transformed_setpoint)

        # Publish Twist
        # rospy.loginfo("Publishing Twist" + str(msg))
        pub.publish(msg)
        rate.sleep()
        # Call service client again if the returned path is not empty and do stuff again
        path = new_path

    # while new_path.poses:
    #     # rospy.loginfo("new_path.poses = " + str(new_path.poses))
    #     # Call service client with path
    #     res = control_client(new_path)
    #     rospy.loginfo("The frame is: %s", res.setpoint.header.frame_id)
    #     # Transform Setpoint from service client
    #     transformed_setpoint = transform_setpoints(robot_frame_id, res.setpoint)
    #     # Create Twist message from the transformed Setpoint
    #     msg = create_twist(transformed_setpoint)
    #     # Publish Twist
    #     rospy.loginfo("Publishing Twist")
    #     pub.publish(msg)
    #     # Call service client again if the returned path is not empty and do stuff again
    #     new_path = res.new_path
    #     rate.sleep()
    #     # Get new path from action server
    #     path = res.new_path


    # Send 0 control Twist to stop robot
    msg = Twist() 
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)

'''
Example:

from move_base.msg import *
rospy.init_node('foo')


from move_base.msg import *
from geometry_msgs.msg import *
g1 = MoveBaseGoal(PoseStamped(Header(frame_id = 'base_link'),
                              Pose(Point(2, 0, 0),
                                   Quaternion(0, 0, 0, 1))))
g2 = MoveBaseGoal(PoseStamped(Header(frame_id = 'base_link'),
                              Pose(Point(5, 0, 0),
                                   Quaternion(0, 0, 0, 1))))

client = ActionClient('move_base', MoveBaseAction)

h1 = client.send_goal(g1)
h2 = client.send_goal(g2)
client.cancel_all_goals()
'''

def get_path():
    global goal_client

    while True:
        rospy.loginfo("Controller: Get Path is called")
        # Get path from action server
        
        goal_client.wait_for_server()
        # while ready == False:
        #     ready = goal_client.wait_for_server()

        rospy.loginfo("Controller: Wait for server ends")

        # goal_client.send_goal_and_wait(0)

        # goal_client.send_goal(0)

        goal = irob_assignment_1.msg.GetNextGoalAction()

        goal_client.send_goal(goal)

        rospy.loginfo("Controller: Sending the goal "+str(goal))

        goal_client.wait_for_result()
        # while ready1 == False:
        #     ready1 = goal_client.wait_for_result()
        rospy.loginfo("Controller: Wait for result ends")

        result = goal_client.get_result()

        print("Controller: Goal is " + str(result))

        

        # goal_client.wait_for_result()

        # if result.gain == 0:
        #     break

        # Call move with path from action server
        move(result.path)


if __name__ == "__main__":
    # Init node
    rospy.init_node("controller")
    rospy.loginfo("Controller: Node is initialized!")

    # Create TF buffer
    # Time to buffer the transforms
    tf_buffer = tf2_ros.Buffer()    
    listener = tf2_ros.TransformListener(tf_buffer) 

    # Init publisher
    # Topic /cmd_vel with subscriber /gazebo
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Limit the frequency of publishing to /cmd_vel between 10-20 Hz
    rate = rospy.Rate(10)

    # Init simple action client
    # Provided by the explore node
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Init service client
    # Proviced by the collision_avoidance node
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Call get path
    get_path()

    # Spin
    rospy.spin()
