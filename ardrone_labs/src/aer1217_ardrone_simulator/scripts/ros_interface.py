#!/usr/bin/env python2

"""
ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from position_controller import PositionController
from desired_positions import ROSDesiredPositionGenerator
import std_msgs
#from std_msgs.msg._Empty import Empty
#from numpy import roll


class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
    def __init__(self):
        self.rate = rospy.Rate(100)
        #Subscribers
        self.vicon_topic = '/vicon/ARDroneCarre/ARDroneCarre'
        self._vicon_msg = TransformStamped()
        self.sub_vicon = rospy.Subscriber(self.vicon_topic, TransformStamped, self._vicon_callback)
        
        #Publishers
        self.pub_traj = rospy.Publisher('/cmd_vel_RHC', Twist, queue_size=32)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        
    def _vicon_callback(self, msg):
        _vicon_msg = msg
        
    def get_pos(self):
        return _vicon_msg.transform.translation
    
    def get_orient(self):
        return euler_from_quaternion(_vicon_msg.transform.rotation)
    
    def land(self):
        self.pub_land.publish()

    def set_vel(self, traj):
        self.pub_traj.publish(traj)

if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node("ros_interface", disable_signals=True)    
    ardrone = ROSControllerNode()
    positionCtrl = PositionController()
    positionGen = ROSDesiredPositionGenerator()
    try:
        while not rospy.is_shutdown():
            #get position nad orientation from vicon
            currentPosition = ardrone.get_pos()
            currentOrientation = ardrone.get_orient()
            #compute desired pose
            desired_pose = positionGen.generate_position(currentPosition, currentOrientation)
            #pass to position controller
            traj = positionCtrl.generate_cmd(desired_pose, currentPosition, currentOrientation)
            #publish actuation commands
            ardrone.set_vel(traj)
            #spin
            rospy.rate.sleep()
    except KeyboardInterrupt:
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.z = 0
        ardone.set_vel(msg)
        ardrone.land()
        rospy.spin()
        
    rospy.spin()
