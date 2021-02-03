#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist


class PositionController(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    def __init__(self):
        ## Current State
        # Position
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Velocity

        # Acceleration
        
        ## Desired State
        # Position
        self.x_des = 0
        self.y_des = 0
        self.z_des = 0
        self.roll_des = 0
        self.pitch_des = 0
        self.yaw_des = 0

        # Velocity

        # Acceleration



        return

    def updateState(self, x, y, z, roll, pitch, yaw):
        self.old_x = self.x
        self.x = x
        self.old_y = self.y
        self.y = y
        self.old_z = self.z
        self.z = z
        self.old_roll = self.roll
        self.roll = roll
        self.old_pitch = self.pitch
        self.pitch = pitch
        self.old_yaw = self.yaw
        self.yaw = yaw

        self.updateAcceleration(thrust, roll, pitch)

        return

    def updateAcceleration(self, thrust, roll, pitch):
        self.x_double_dot = thrust*np.cos(pitch)*np.sin(roll)
        self.y_double_dot = -thrust*np.sin(pitch)
        self.z_double_dot = thrust*np.cos(roll)*cos(pitch)
        return

    def getDesiredState(self, x_des, y_des, z_des, yaw_des):
        # Express dynamics in the inertial frame
        
        # Simplify by assuming 0 yaw
        
        # Express acceleration in terms of roll pitch and thrust
        
        # Express the commanded roll, pitch, and thrust in terms of commanded acceleration

        # Calculate commanded accelerations by designing a second order system based on the error in positions and velocities (PD controller)

        # Transform the commanded roll and pitch from inertial to body frame to account for non zero yaw
        
        C_x = 1;
        C_y = 1;
        w_n_x = 1;
        w_n_y = 1;
        
        x_double_dot_des = 2*C_x*w_n_x*(x_dot_des - x_dot) + w_n_x**2*(x_des - self.x)
        y_double_dot_des = 2*C_y*w_n_y*(y_dot_des - y_dot) + w_n_y**2*(y_des - self.y)
        
        f = (self.z_double_dot - 9.8)/(np.cos(self.roll)*np.cos(self.pitch))
        
        roll_des = np.arcsin(self.y_double_dot/f)
        pitch_des = np.arcsin(self.x_double_dot/(f*np.cos(roll_des)))

        roll_des_base = roll_des*np.cos(self.yaw)+pitch_des*np.sin(self.yaw)
        pitch_des_base = -roll_des*np.sin(self.yaw) + pitch_des*cos(self.yaw)
        
        return roll_des_base, pitch_des_base, yaw_dot_des, z_dot_des







