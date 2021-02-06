#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

from geometry_msgs.msg import TransformStamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ros_interface import ROSControllerNode

class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""
    # write code here for desired position trajectory generator
    def __init__(self):
        self.rospy.Rate(200)
        self.pub_des_pos = rospy.Publisher('/desired_position', TransformStamped, queue_size=10)

    def linear(self, n):
        x1 = np.linspace(-1, 1, n)
        x2 = np.linspace(1, -1, n)
        x = np.concatenate((x1,x2))

        y1 = np.linspace(-1, 1, n)
        y2 = np.linspace(1, -1, n)
        y = np.concatenate((y1, y2))

        z1 = np.linspace(1, 2, n)
        z2 = np.linspace(2, 1, n)
        z = np.concatenate((z1, z2))

        yaw = np.zeros(n) #?

        return x,y,z, yaw


    def circular(self, n):
        r = 1
        theta = np.linspace(0,2*np.pi,n)
        x = np.cos(theta) * r
        y = np.cos(theta) * r

        z1 = np.linspace(0.5, 1.5, n/2)
        z2 = np.linspace(1.5, 0.5, n/2)
        z = np.concatenate((z1,z2))

        yaw = np.linspace(-np.pi, np.pi, n)

        return x, y, z, yaw


    def pub_des_pos(self, i, x, y, z, yaw):
        msg = TransformStamped()
        msg.transform.translation.x = x[i]
        msg.transform.translation.y = y[i]
        msg.transform.translation.z = z[i]

        q = quaternion_from_euler(0, 0, yaw[i])
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        self.pub_des_pos.publish(msg)


if __name__ == '__main__':
    rospy.init_node('desired_positions')
    PosGen = ROSDesiredPositionGenerator()

    time = 10
    freq = 100

    traj = 1
    n = time * freq

    while True:
        if traj == 1:
            i = 0
            (x, y, z, yaw) = PosGen.linear(n)
            while True:
                PosGen.pub_des_pos(i, x, y, z, yaw)
                i += 1
                if i > n:
                    break
        elif traj == 2:
            i = 0
            (x, y, z, yaw) = PosGen.circular(n)
            while True:
                PosGen.pub_des_pos(i, x, y, z, yaw)
                i += 1
                if i > n:
                    break
        else:
            break
    rospy.spin()


#     pos = ROSControllerNode.get_pos()
#     ori = ROSControllerNode.get_orient()
#     x = pos.x
#     y = pos.y
#     z = pos.z