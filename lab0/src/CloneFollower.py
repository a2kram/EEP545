#!/usr/bin/env python

import rospy
import Utils
import numpy as np
from geometry_msgs.msg import PoseStamped

SUB_TOPIC = '/sim_car_pose/pose'
PUB_TOPIC = '/clone_follower_pose/pose'
MAP_TOPIC = 'static_map'

# Follows the simulated robot around
class CloneFollower:
  '''
  Initializes a CloneFollower object
  In:
    follow_offset: The required x offset between the robot and its clone follower
    force_in_bounds: Whether the clone should toggle between following in front
                     and behind when it goes out of bounds of the map
  '''
  def __init__(self, follow_offset, force_in_bounds):
    self.follow_offset = follow_offset
    self.force_in_bounds = force_in_bounds
    self.map_img, self.map_info = Utils.get_map (MAP_TOPIC)
    
    # Setup publisher that publishes to PUB_TOPIC
    self.pub = rospy.Publisher (PUB_TOPIC, PoseStamped, queue_size=10)
    
    # Setup subscriber that subscribes to SUB_TOPIC and uses the self.update_pose
    # callback
    self.sub = rospy.Subscriber (SUB_TOPIC, PoseStamped, self.update_pose)
    
  '''
  Given the translation and rotation between the robot and map, computes the pose
  of the clone
  In:
    trans: The translation between the robot and map
    rot: The rotation between the robot and map
  Out:
    The pose of the clone
  '''
  def compute_follow_pose(self, trans, rot):
    theta = Utils.quaternion_to_angle (rot)
    x = trans.x + self.follow_offset * np.cos (theta)
    y = trans.y + self.follow_offset * np.sin (theta)

    return [x, y, theta]

  '''
  Callback that runs each time a sim pose is received. Should publish an updated
  pose of the clone.
  In:
    msg: The pose of the simulated car. Should be a geometry_msgs/PoseStamped
  '''
  def update_pose(self, msg):    
    # Compute the pose of the clone
    follow_pose = self.compute_follow_pose(msg.pose.position, msg.pose.orientation)

    # Check bounds if required
    if self.force_in_bounds:
      location = Utils.world_to_map (follow_pose, self.map_info)

      if self.map_img[location[1]][location[0]] == 0:
        self.follow_offset *= -1
        follow_pose = self.compute_follow_pose(msg.pose.position, msg.pose.orientation)

    # Setup the outgoing PoseStamped message
    out_pose = PoseStamped ()
    out_pose.header.frame_id = "/map"
    out_pose.header.stamp = rospy.Time.now()

    out_pose.pose.position.x = follow_pose[0]
    out_pose.pose.position.y = follow_pose[1]
    out_pose.pose.orientation = Utils.angle_to_quaternion (follow_pose[2])

    # Publish the clone's pose
    self.pub.publish (out_pose)
    
if __name__ == '__main__':
  follow_offset = 1.5 # The offset between the robot and clone
  force_in_bounds = False # Whether or not map bounds should be enforced
  
  rospy.init_node ('clone_follower', anonymous=True) # Initialize the node
  
  # Populate params with values passed by launch file
  follow_offset = rospy.get_param ('follow_offset')
  force_in_bounds = rospy.get_param ('force_in_bounds')
  
  cf = CloneFollower (follow_offset, force_in_bounds) # Create a clone follower
  rospy.spin () # Spin
  
