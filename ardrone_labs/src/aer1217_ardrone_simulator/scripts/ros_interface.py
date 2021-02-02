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


class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
    pass

if __name__ == '__main__':
    # write code to create ROSControllerNode
