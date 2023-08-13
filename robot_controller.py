#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
import math 
import numpy
from geometry_msgs.msg import Twist

# robot pose
robot_x = 0
robot_y = 0
robot_theta = 0 #in degrees

# Controller gain
K = 0.01

# goal position
goal_x = 1
goal_y = 1
# Commanded velocity 
move = Twist() # defining the way we can allocate the values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0

#############################Edit only this function######
def controller(): 
  global move
  global robot_x, robot_y, robot_theta
  global goal_x, goal_y

  distance_to_goal = math.sqrt(math.pow((goal_x - robot_x),2) + math.pow((goal_y - robot_y),2)) # Distance to goal
  if  distance_to_goal < 0.05:
    move.linear.x = 0
    move.linear.y = 0
    move.linear.z = 0
    move.angular.x = 0
    move.angular.y = 0
    move.angular.z = 0
    print("goal reached")
    # Stop robot
    return
  desired_orientation = math.degrees(math.atan2(goal_y - robot_y , goal_x - robot_x))# Desired orientation
  orientation_error = desired_orientation - robot_theta
  orientation_error = orientation_error % 360 #restrict error to (-180,180)
  if orientation_error > 180:
    orientation_error = orientation_error - 360
  move.angular.z = K * orientation_error
  move.linear.x = 0.5 # Linear velocity
  print("Des angle",desired_orientation)
  print("Error", orientation_error)
  
##########################################################


def callback(data):
  global robot_x, robot_y, robot_theta
  robot_x = data.pose.pose.position.x 
  robot_y = data.pose.pose.position.y
  orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
  euler = tf.transformations.euler_from_quaternion(orientation_list)
  robot_theta = math.degrees(euler[2]) # in degrees
  print("Robot state",robot_theta)
 
  controller()


rospy.init_node('Go_to_goal')  # Defines a node with name of Go_to_goal
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('/odom', Odometry, callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()
