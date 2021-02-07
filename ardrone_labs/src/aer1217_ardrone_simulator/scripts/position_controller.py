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
        
        ## Current State
        
        # Position

        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Old position

        self.old_x = 0
        self.old_y = 0
        self.old_z = 0
        self.old_roll = 0
        self.old_pitch = 0
        self.old_yaw = 0

        # Velocity

        self.x_dot = 0
        self.y_dot = 0
        self.z_dot = 0
        self.roll_dot = 0
        self.pitch_dot = 0
        self.yaw_dot = 0
        
        # Old velocity

        self.old_x_dot = 0
        self.old_y_dot = 0
        self.old_z_dot = 0
        self.old_roll_dot = 0
        self.old_pitch_dot = 0
        self.old_yaw_dot = 0

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

    def updateState(self, currentPosition, currentOrientation, dt):
        self.updatePosition(currentPosition, currentOrientation)
        self.updateVelocity(dt)
        self.updateAcceleration(dt)
        return

    def updatePosition(self, currentPosition, currentOrientation):
        self.old_x = self.x
        self.x = currentPosition.x
        
        self.old_y = self.y
        self.y = currentPosition.y
        
        self.old_z = self.z
        self.z = currentPosition.z
        
        self.old_roll = self.roll
        self.roll = currentOrientation[0]
        
        self.old_pitch = self.pitch
        self.pitch = currentOrientation[1]
        
        self.old_yaw = self.yaw
        self.yaw = currentOrientation[2]

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
        self.x_double_dot = (self.x_dot - self.old_x_dot)/dt
        self.y_double_dot = (self.y_dot - self.old_y_dot)/dt
        self.z_double_dot = (self.z_dot - self.old_z_dot)/dt
        return

    def getDesiredState(self, currentPosition, currentOrientation, x_des, y_des, z_des, yaw_des, dt):

        self.updateState(currentPosition, currentOrientation, dt)

        # Gains
        x_double_dot_P_gain = 0.59  #0.08
        x_double_dot_D_gain = 1.4  #1.33 #0.1

        y_double_dot_P_gain = 0.59 #0.08
        y_double_dot_D_gain = 1.4 #1.33 #0.1

        yaw_dot_P_gain = 0.5 #3 #1
        z_dot_P_gain = 0.15 #0.05 0.74

        self.x_double_dot_des = x_double_dot_D_gain*(self.x_dot_des - self.x_dot) + x_double_dot_P_gain*(x_des - self.x)
        self.y_double_dot_des = y_double_dot_D_gain*(self.y_dot_des - self.y_dot) + y_double_dot_P_gain*(y_des - self.y)
        
        f = (self.z_double_dot + 9.8)/(np.cos(self.roll)*np.cos(self.pitch))
        
        asin_arg_roll = max(-self.y_double_dot_des/(f+1e-8),-1)
        asin_arg_roll = min(asin_arg_roll,1)        
        self.roll_des = np.arcsin(asin_arg_roll)
        
        asin_arg_pitch = max(self.x_double_dot_des/(f*np.cos(self.roll_des)+1e-8),-1)
        asin_arg_pitch = min(asin_arg_pitch,1)
        self.pitch_des = np.arcsin(asin_arg_pitch) 

        self.roll_des_base = self.roll_des*np.cos(self.yaw)+self.pitch_des*np.sin(self.yaw)
        self.pitch_des_base = -self.roll_des*np.sin(self.yaw) + self.pitch_des*np.cos(self.yaw)

        yaw_error = yaw_des - self.yaw
        if yaw_error > np.pi:
            yaw_error = yaw_error - 2*np.pi
        elif yaw_error < -np.pi:
            yaw_error = yaw_error + 2*np.pi
        
        self.yaw_dot_des = yaw_dot_P_gain*yaw_error 
        self.z_dot_des = z_dot_P_gain*(z_des - self.z)
        
        msg = Twist()
        msg.linear.x = min(self.roll_des_base,1.0)
        msg.linear.y = min(self.pitch_des_base, 1.0)

        
        msg.linear.x = max(msg.linear.x,-1.0)
        msg.linear.y = max(msg.linear.y,-1.0)

        msg.linear.z = self.z_dot_des
        msg.angular.z = self.yaw_dot_des

        return msg





