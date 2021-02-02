#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator


class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""
    # write code here for desired position trajectory generator
    pass

if __name__ == '__main__':
    rospy.init_node('desired_position')
    ROSDesiredPositionGenerator()
    rospy.spin()
