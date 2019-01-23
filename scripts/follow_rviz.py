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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class RvizFollow:

  def __init__(self):    
    self.waypoints = []
    self.position = 0

    self.mark_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.client.wait_for_server()

  def get_pose(self, pose):
    rospy.loginfo("Getting new pose")
    self.save_pose(pose)
    self.draw_marker(pose)

  def save_pose(self, pose):
    self.waypoints.append(pose)

  def follow_waypoints(self):
    while not rospy.is_shutdown():
      amount_waypoints = len(self.waypoints)
      if amount_waypoints > self.position:
        self.follow_next_waypoint()

  def follow_next_waypoint(self):
    pose = self.waypoints[self.position]

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose.pose.pose.position.x  
    goal.target_pose.pose.position.y = pose.pose.pose.position.y
    goal.target_pose.pose.orientation.z = pose.pose.pose.orientation.z
    goal.target_pose.pose.orientation.w = pose.pose.pose.orientation.w

    self.client.send_goal(goal)
    self.client.wait_for_result()

    self.position = self.position + 1

  def subscribe_to_initialpose(self):
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.get_pose)

  def draw_marker(self, pose):
    marker = self.create_marker(pose,len(self.waypoints))
    self.mark_publisher.publish(marker)

  def create_marker(self, pose, id):
    marker = Marker()
    marker.id = id
    marker.header.frame_id = "odom";
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.pose.position.x = pose.pose.pose.position.x
    marker.pose.position.y = pose.pose.pose.position.y
    marker.pose.orientation.z = pose.pose.pose.orientation.z
    marker.pose.orientation.w = pose.pose.pose.orientation.w
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; 
    marker.color.r = 255;
    
    return marker
  
  def draw_makers(self):

    marker_array = MarkerArray()
    marker_array.markers = self.markers
    self.mark_publisher.publish(marker_array)



  # Class method
  # Initialize everything that is need to be done to make the follow waypoints work
  @classmethod
  def follow_rviz(cls):
    follower = cls()
    follower.subscribe_to_initialpose()
    follower.follow_waypoints()





def main():

  rospy.init_node('follow_rviz')
  
  # function to follow waypoints
  RvizFollow.follow_rviz()

if __name__ == '__main__':
    main()