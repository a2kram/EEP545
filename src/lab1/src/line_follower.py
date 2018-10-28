#!/usr/bin/env python
import collections
import sys
import rospy
import numpy as np
import utils

from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped


# The topic to publish control commands to
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0' 

# The topic that provides the current pose of the robot as a PoseStamped msg
POSE_TOPIC = '/sim_car_pose/pose'


'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:

  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
  '''
  def __init__(self, plan, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, 
    error_buff_length, speed):
    self.plan = plan
    self.plan_lookahead = plan_lookahead
    # Normalize translation and rotation weights
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = collections.deque (maxlen=error_buff_length)
    self.speed = speed

    self.cmd_pub = rospy.Publisher (PUB_TOPIC, AckermannDriveStamped, queue_size=10)
    self.pose_sub = rospy.Subscriber (POSE_TOPIC, PoseStamped, self.pose_cb)
  
  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (True, 0.0) if the end of the plan has been reached. Otherwise, returns
           (False, E) - where E is the computed error
  '''
  def compute_error (self, cur_pose):
    
    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if 
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop
    while len(self.plan) > 0:
      # YOUR CODE HERE
      pass        
      
    # Check if the plan is empty. If so, return (False, 0.0)
    # YOUR CODE HERE
    
    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in 
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index
    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)
   
    # Compute the translation error between the robot and the configuration at goal_idx in the plan
    # YOUR CODE HERE
    
    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    #   Be carefult about the sign of the rotation error
    # YOUR CODE HERE
    error = # self.translation_weight * translation_error + self.rotation_weight * rotation_error

    return False, error
    
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time
    
    # Compute the derivative error using the passed error, the current time,
    # the most recent error stored in self.error_buff, and the most recent time
    # stored in self.error_buff
    # YOUR CODE HERE
    
    # Add the current error to the buffer
    self.error_buff.append((error, now))
    
    # Compute the integral error by applying rectangular integration to the elements
    # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
    # YOUR CODE HERE
    
    # Compute the steering angle as the sum of the pid errors
    # YOUR CODE HERE
    return #self.kp*error + self.ki*integ_error + self.kd * deriv_error
    
  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
  '''  
  def pose_cb (self, msg):
    cur_pose = np.array ([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
    success, error = self.compute_error (cur_pose)
    
    if success:
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      
    delta = self.compute_steering_angle (error)
    
    # Setup the control message
    ads = AckermannDriveStamped ()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now ()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish (ads)

def main ():
  rospy.init_node ('line_follower', anonymous=True)
    
  plan_lookahead = rospy.get_param ('plan_lookahead')
  translation_weight = rospy.get_param ('translation_weight')
  rotation_weight = rospy.get_param ('rotation_weight')
  kp = rospy.get_param ('kp')
  ki = rospy.get_param ('ki')
  kd = rospy.get_param ('kd')
  error_buff_length = rospy.get_param ('error_buff_length')
  speed = rospy.get_param ('speed')

  rcved_plan = []
  msg = rospy.wait_for_message ('/planner_node/car_plan', PoseArray)
  for pose in msg.poses:
    rcved_plan.append ([pose.position.x, pose.position.y, utils.quaternion_to_angle (pose.orientation)])

  LineFollower (rcved_plan, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, 
    error_buff_length, speed)
  
  rospy.spin ()

if __name__ == '__main__':
  main ()