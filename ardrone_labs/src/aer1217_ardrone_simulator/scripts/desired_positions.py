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
        self.vicon_topic = '/vicon/ARDroneCarre/ARDroneCarre'
        self.sub_vicon = rospy.Subscriber(self.vicon_topic, TransformStamped, self.get_vicon_data)
        self.pub_traj = rospy.Publisher('/desired_position', Twist, queue_size=10)

        self.freq = 2000

        self.x_des = []
        self.y_des = []
        self.z_des = []
        self.yaw_des = []
        self.count = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0

        self.total_count = 100
        self.thresh = 0.1
        self.linear(self.total_count/2)
        #self.circular(self.total_count / 2)
        self.traj_timer = rospy.Timer(rospy.Duration(1. / self.freq), self.pub_des_pos)




    def linear(self, n):
        x1 = np.linspace(-1, 1, n)
        x2 = np.linspace(1, -1, n)
        self.x_des = np.concatenate((x1, x2))

        y1 = np.linspace(-1, 1, n)
        y2 = np.linspace(1, -1, n)
        self.y_des = np.concatenate((y1, y2))

        z1 = np.linspace(1, 2, n)
        z2 = np.linspace(2, 1, n)
        self.z_des = np.concatenate((z1, z2))

        self.yaw_des = np.zeros(2*n) #?

        # x1 = np.linspace(4, 4, n)
        # x2 = np.linspace(4, 4, n)
        # self.x_des = np.concatenate((x1, x2))
        #
        # y1 = np.linspace(4, 4, n)
        # y2 = np.linspace(4, 4, n)
        # self.y_des = np.concatenate((y1, y2))
        #
        # z1 = np.linspace(4, 4, n)
        # z2 = np.linspace(4, 4, n)
        # self.z_des = np.concatenate((z1, z2))
        #
        # self.yaw_des = np.ones(2*n)  # ?



    def circular(self, n):
        r = 1
        theta = np.linspace(0, 2*np.pi, n*2)
        self.x_des = np.cos(theta) * r
        self.y_des = np.sin(theta) * r

        z1 = np.linspace(0.5, 1.5, n)
        z2 = np.linspace(1.5, 0.5, n)
        self.z_des = np.concatenate((z1, z2))

        #self.yaw_des = np.zeros(2 * n)

        self.yaw_des = np.linspace(-np.pi, np.pi, n*2)

    # callback function for vicon
    def get_vicon_data(self, vicon_msg):
        self.x = vicon_msg.transform.translation.x
        self.y = vicon_msg.transform.translation.y
        self.z = vicon_msg.transform.translation.z
        quaternion = np.array([vicon_msg.transform.rotation.x,
                               vicon_msg.transform.rotation.y,
                               vicon_msg.transform.rotation.z,
                               vicon_msg.transform.rotation.w])
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def pub_des_pos(self, i):
        # msg = TransformStamped()
        # msg.transform.translation.x = self.x_des[self.count]
        # msg.transform.translation.y = self.y_des[self.count]
        # msg.transform.translation.z = self.z_des[self.count]
        #
        # q = quaternion_from_euler(0, 0, self.yaw_des[self.count])
        # msg.transform.rotation.x = q[0]
        # msg.transform.rotation.y = q[1]
        # msg.transform.rotation.z = q[2]
        # msg.transform.rotation.w = q[3]

        msg = Twist()
        msg.linear.x = self.x_des[self.count]
        msg.linear.y = self.y_des[self.count]
        msg.linear.z = self.z_des[self.count]
        msg.angular.z = self.yaw_des[self.count]

        self.pub_traj.publish(msg)

        update_count = self.check_dist(self.x_des[self.count], self.x) and \
                       self.check_dist(self.y_des[self.count], self.y) and \
                       self.check_dist(self.z_des[self.count], self.z) and \
                       self.yaw_check(self.yaw_des[self.count], self.yaw)
        #print('count = {}'.format(self.count))
        #print(len(self.x_des))
        if update_count:
            if self.count < self.total_count-1:
                self.count += 1
                print('x = '.format(self.x_des[self.count]))
                print('y = '.format(self.y_des[self.count]))
                print('z = '.format(self.z_des[self.count]))
            else:
                self.count = 0
    #
    # def full_traj (n):
    #     (x_linear, y_linear, z_linear, yaw) = PosGen.linear(n)
    #

    # check distance of desired and actual value to determine if proceed
    def check_dist(self, des, act):
        if abs(des - act) > self.thresh:
            return False
        else:
            return True

    def yaw_check(self,des,act):
        yaw_error = des - act
        if yaw_error > np.pi:
            yaw_error = yaw_error - 2 * np.pi
        if yaw_error > self.thresh:
            return False
        else:
            return True




if __name__ == '__main__':
    rospy.init_node('desired_positions')
    ROSDesiredPositionGenerator()
    rospy.spin()


#     pos = ROSControllerNode.get_pos()
#     ori = ROSControllerNode.get_orient()
#     x = pos.x
#     y = pos.y
#     z = pos.z