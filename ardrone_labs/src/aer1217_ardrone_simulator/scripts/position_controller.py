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
        # Internal state
        self.internal_state = TransformStamped()

        # Subscribers

        # Publishers


        ## Current State
        
        # Position

        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Old position

        self.x_old = 0
        self.y_old = 0
        self.z_old = 0
        self.roll_old = 0
        self.pitch_old = 0
        self.yaw_old = 0

        # Velocity

        self.x_dot = 0
        self.y_dot = 0
        self.z_dot = 0
        self.roll_dot = 0
        self.pitch_dot = 0
        self.yaw_dot = 0
        
        # Old velocity

        self.x_dot_old = 0
        self.y_dot_old = 0
        self.z_dot_old = 0
        self.roll_dot_old = 0
        self.pitch_dot_old = 0
        self.yaw_dot_old = 0

        # Acceleration

        self.x_double_dot = 0
        self.y_double_dot = 0
        self.z_double_dot = 0
        self.roll_double_dot = 0
        self.pitch_double_dot = 0
        self.yaw_double_dot = 0

        ## Desired State
        
        # Position

        self.x_des = 0
        self.y_des = 0
        self.z_des = 0
        self.roll_des = 0
        self.pitch_des = 0
        self.yaw_des = 0

        # Velocity

        self.x_dot_des = 0
        self.y_dot_des = 0
        self.z_dot_des = 0
        self.roll_dot_des = 0
        self.pitch_dot_des = 0
        self.yaw_dot_des = 0
        
        # Acceleration

        self.x_double_dot = 0
        self.y_double_dot = 0
        self.z_double_dot = 0
        self.roll_double_dot = 0
        self.pitch_double_dot = 0
        self.yaw_double_dot = 0

        return

    def updateState(self, dt):
        self.updatePosition()
        self.updateVelocity(dt)
        self.updateAcceleration(dt)
        return

    def updatePosition(self):
        self.old_x = self.x
        self.x = self.internal_state.transform.translation.x
        self.old_y = self.y
        self.y = self.internal_state.transform.translation.y
        self.old_z = self.z
        self.z = self.internal_state.transform.translation.z
        self.old_roll = self.roll
        self.roll = self.internal_state.transform.angular.x
        self.old_pitch = self.pitch
        self.pitch = self.internal_state.transform.angular.y
        self.old_yaw = self.yaw
        self.yaw = self.internal_state.transform.angular.z

        return

    def updateVelocity(self, dt):
        self.old_x_dot = self.x_dot
        self.x_dot = (self.x - self.old_x)/dt

        self.old_y_dot = self.y_dot
        self.y_dot = (self.y - self.old_y)/dt

        self.old_z_dot = self.z_dot
        self.z_dot = (self.z - self.old_z)/dt

        self.old_roll_dot = self.roll_dot
        self.roll_dot = (self.roll-self.old_roll)/dt

        self.old_pitch_dot = self.pitch_dot
        self.pitch_dot = (self.pitch - self.old_pitch)/dt

        self.old_yaw_dot = self.yaw_dot
        self.yaw_dot = (self.yaw - self.old_yaw)/dt
        return

    def updateAcceleration(self, dt):
        #self.x_double_dot = thrust*np.cos(pitch)*np.sin(roll)
        #self.y_double_dot = -thrust*np.sin(pitch)
        #self.z_double_dot = thrust*np.cos(roll)*cos(pitch)

        self.x_double_dot = (self.x_dot - self.x_dot_old)/dt
        self.y_double_dot = (self.y_dot - self.y_dot_old)/dt
        self.z_double_dot = (self.z_dot - self.z_dot_old)/dt
        return

    def getDesiredState(self, x_des, y_des, z_des, yaw_des, dt):
        # Get time since last call
        self.old_time = self.current_time
        self.current_time = rospy.get_time()
        dt = self.current_time - self.old_time
        
        # Update state variables
        self.updateState(dt)
        
        rise_time = 10;
        settling_time = 20;
        w_n_x = rise_time/1.8;
        w_n_y = rise_time/1.8;
        C_x = 4.6/(w_n_x*settling_time);
        C_y = 4.6/(w_n_y*settling_time);
        yaw_dot_P_gain = 1;
        z_dot_P_gain = 1;
        
        self.x_double_dot_des = 2*C_x*w_n_x*(self.x_dot_des - self.x_dot) + w_n_x**2*(self.x_des - self.x)
        self.y_double_dot_des = 2*C_y*w_n_y*(self.y_dot_des - self.y_dot) + w_n_y**2*(self.y_des - self.y)
        
        f = (self.z_double_dot - 9.8)/(np.cos(self.roll)*np.cos(self.pitch))
        
        self.roll_des = np.arcsin(self.y_double_dot/f)
        self.pitch_des = np.arcsin(self.x_double_dot/(f*np.cos(self.roll_des)))

        self.roll_des_base = self.roll_des*np.cos(self.yaw)+self.pitch_des*np.sin(self.yaw)
        self.pitch_des_base = -self.roll_des*np.sin(self.yaw) + self.pitch_des*cos(self.yaw)
        
        self.yaw_dot_des = yaw_dot_P_gain*(yaw_des - self.yaw)
        self.z_dot_des = z_dot_P_gain*(z_des - self.z)

        return self.roll_des_base, self.pitch_des_base, self.yaw_dot_des, self.z_dot_des




