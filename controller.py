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

irob_assignment_1/GetNextGoalAction --- action server get_next_goal
      
Definitions:

# Goal definition
---
# Result definition
float64 gain
nav_msgs/Path path
---
# Feedback definition
float64 gain
nav_msgs/Path path


irob_assignment_1/GetSetpoint

# Define the request
nav_msgs/Path path
---
# Define the response
geometry_msgs/PointStamped setpoint
nav_msgs/Path new_path

You get a new path since the collision avoidance node is removing points along the path which is thinks you have already moved passed. 
The setpoint is where you should move the robot next to follow the path in a safe and efficient way.

geometry_msgs/PointStamped

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z


Topics:

Type: geometry_msgs/Twist

Publishers: None

Subscribers: 
 * /gazebo (http://X:YYYYY/)

 
Transform:
transform = tf_buffer.lookup_transform(...)
transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

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



# ---------------- Supportive Functions ----------------

def transform_setpoints(robot_frame_id, setpoint):
    rospy.loginfo("Controller: Transforming...")
    transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, rospy.Time(0)) 
    transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)
    rospy.loginfo("Controller: Transforming Done")
    return transformed_setpoint

def create_twist(transformed_setpoint):
    msg = Twist()
    msg.angular.z = math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    # msg.linear.x = transformed_setpoint.point.x
    msg.linear.x = math.hypot(transformed_setpoint.point.x, transformed_setpoint.point.y)
    limit_velocities(msg)
    return msg

def limit_velocities(msg):
    if msg.angular.z > max_angular_velocity:
        msg.angular.z = max_angular_velocity
    if msg.linear.x > max_linear_velocity:
        msg.linear.x = max_linear_velocity

# ---------------- Main Functions ----------------

def move(path):
    global control_client, robot_frame_id, pub
    # Limit the frequency of publishing to /cmd_vel between 10-20 Hz
    rate = rospy.Rate(10)
    # Call service client with path
    # Call service client again if the returned path is not empty and do stuff again
    while path.poses:
        rospy.loginfo("Controller Move in While")
        # control_client.wait_for_service()
        res = control_client(path)
        setpoint = res.setpoint
        path = res.new_path
        
        # Transform Setpoint from service client
        transformed_setpoint = transform_setpoints(robot_frame_id, setpoint)
        
        # Create Twist message from the transformed Setpoint
        msg = create_twist(transformed_setpoint)

        # Publish Twist
        rospy.loginfo("Controller: Publishing Twist")
        pub.publish(msg)
        rate.sleep()

    # Send 0 control Twist to stop robot
    msg = Twist() 
    msg.angular.z = 0
    msg.linear.x = 0
    rospy.loginfo("Controller: Publishing Twist z=0, x=0")
    pub.publish(msg)


def get_path():
    global goal_client

    while True:
        rospy.loginfo("Controller: Get Path is called")
        # Get path from action server
        goal_client.wait_for_server()

        rospy.loginfo("Controller: Wait for goal_client ends")

        goal = irob_assignment_1.msg.GetNextGoalAction()

        goal_client.send_goal(goal)

        rospy.loginfo("Controller: Sending the goal")

        goal_client.wait_for_result()

        rospy.loginfo("Controller: Wait for goal_client result ends")

        result = goal_client.get_result()

        print("Controller: Get Goal")

        if result.gain == 0: # not result.path
            rospy.loginfo("Controller: Path is empty")
            break

        # Call move with path from action server
        move(result.path)

# ------------------- Main -------------------

if __name__ == "__main__":
    # Init node
    rospy.init_node("controller")
    rospy.loginfo("Controller: Node is initialized!")

    # Create TF buffer
    # Time to buffer the transforms
    tf_buffer = tf2_ros.Buffer()    
    listener = tf2_ros.TransformListener(tf_buffer) 
    rospy.loginfo("Controller: TF2 Buffer initialized!")

    # Init publisher
    # Topic /cmd_vel with subscriber /gazebo
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Controller: Set as Publisher to topic /cmd_vel")

    # Init simple action client
    # Provided by the explore node
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    rospy.loginfo("Controller: Set as Service Client for get_next_goal of Explore Node")

    # Init service client
    # Proviced by the collision_avoidance node
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)
    rospy.loginfo("Controller: Set as Service Client for get_setpoint of Collision Avoidance Node")

    # Call get path
    get_path()

    # Spin
    rospy.spin()

