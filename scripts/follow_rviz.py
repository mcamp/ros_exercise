#!/usr/bin/env python

# We receive a pose from rviz using '2D Pose Estimate'
# (it is not best way because of what it means, but it works in our case)
# We add the pose to a list of poses
# 
# The robot moves in this list of poses until arriving at the end of the array
# Whenever there is a new pose, the robot will move again

import roslib
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction

waypoints = []
position = 0
client = None

def save_pose(pose):
  global waypoints
  print "Getting new pose"
  waypoints.append(pose)

def follow_waypoints():
  global position
  global waypoints

  while not rospy.is_shutdown():
    amount_waypoints = len(waypoints)
    if amount_waypoints > position:
      follow_next_waypoint()

def follow_next_waypoint():
  global position
  global waypoints

  pose = waypoints[position]

  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = "odom"
  goal.target_pose.header.stamp = rospy.Time.now()
  goal.target_pose.pose.position.x = pose.pose.pose.position.x  
  goal.target_pose.pose.position.y = pose.pose.pose.position.y
  goal.target_pose.pose.orientation.w = pose.pose.pose.orientation.w

  client.send_goal(goal)
  client.wait_for_result()

  position = position + 1


def main():
  global client 

  rospy.init_node('follow_rviz')

  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  client.wait_for_server()

  #subscribe to receive waypoints
  rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, save_pose)
  
  # function to follow waypoints
  follow_waypoints()


if __name__ == '__main__':
    main()