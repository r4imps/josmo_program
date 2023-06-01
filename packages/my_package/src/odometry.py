#!/usr/bin/env python3
import rospy
import time
import numpy as np
from math import sin, cos, pi

from std_msgs.msg import String




class ODOMEETRIA():
    def __init__(self):
        self.x = 0
        self.y = 0

        #Data
        self.theta0 = 0
        self.R = 0.0318
        self.baseline_wheel2wheel = 0.1
        N_tot = 135 #total number of ticks per wheel revolution
        self.alpha = 2*np.pi/N_tot # rotation per tick in radians 

        # Previous data
        self.prev_tick_left = 0
        self.prev_tick_right = 0
        self.x_prev=0
        self.y_prev=0
        self.theta_prev=0


    def DeltaPhi(self,R_encoder,L_encoder, R_prev_ticks,L_prev_ticks):
        # Read the number of ticks
        Rticks = R_encoder
        Lticks = L_encoder
        # Evaluate the number of ticks since the last call 
        R_delta_ticks = Rticks-R_prev_ticks
        L_delta_ticks = Lticks-L_prev_ticks     

        # Evaluate the wheel rotation
        R_delta_phi = self.alpha*R_delta_ticks # in radians
        L_delta_phi = self.alpha*L_delta_ticks
        return R_delta_phi,L_delta_phi
        

    def ODOMETRY_FUNC(self,R_encoder,L_encoder):
        delta_ticks_left = L_encoder-self.prev_tick_left # delta ticks of left wheel 
        delta_ticks_right = R_encoder-self.prev_tick_right # delta ticks of right wheel 
        rotation_wheel_left = self.alpha * delta_ticks_left # total rotation of left wheel 
        rotation_wheel_right = self.alpha * delta_ticks_right # total rotation of right wheel 
        
        # Distance travelled by each wheel
        d_left = self.R * rotation_wheel_left 
        d_right = self.R * rotation_wheel_right
        # Robots distance travelled
        d_A = (d_left + d_right)/2
        # Robot rotation in total
        Delta_Theta = (d_right-d_left)/self.baseline_wheel2wheel # expressed in radians
        # DeltaPhi
        delta_phi_right,delta_phi_left=self.DeltaPhi(R_encoder,L_encoder,self.prev_tick_right,self.prev_tick_left)
    
        #print(delta_phi_left,delta_phi_right)
        # Define wheel radii [m]
        R_left = self.R
        R_right = self.R
        # Define distance travelled by each wheel [m]
        d_left = R_left * delta_phi_left
        d_right = R_right * delta_phi_right
        # Define distance travelled by the robot, in body frame [m]
        d_A = (d_left + d_right)/2
        # Define rotation of the robot [rad]
        Dtheta = (d_right - d_left)/self.baseline_wheel2wheel
        # Define distance travelled by the robot, in world frame [m]
        Dx = d_A * np.cos(self.theta_prev)
        Dy = d_A * np.sin(self.theta_prev)
        # Update pose estimate
        x_curr = self.x_prev + Dx
        y_curr = self.y_prev + Dy
        theta_curr = self.theta_prev + Dtheta
        #Printout HelloWorld
        print(f"The left wheel rotated: {np.rad2deg(rotation_wheel_left)} degrees")
        print(f"The right wheel rotated: {np.rad2deg(rotation_wheel_right)} degrees")  
        print(f"The left wheel travelled: {d_left} meters")
        print(f"The right wheel rotated: {d_right} meters")
        print(f"The robot has travelled: {d_A} meters")
        print(f"The robot has rotated: {np.rad2deg(Delta_Theta)} degrees")
        print("\n")
        return x_curr, y_curr, theta_curr

        


































        """self.rate= time.time() 
        #==================ODOMEETRIA====================
        delta_left = self.left_ticks - self.last_left_ticks
        delta_right = self.right_ticks - self.last_right_ticks
        # Calculate the left and right wheel velocities
        vel_left = delta_left / (self.rate * self.wheel_radius)
        vel_right = delta_right / (self.rate* self.wheel_radius)
        # Calculate the robot's linear and angular velocities
        linear_velocity = (vel_left + vel_right) / 2
        angular_velocity = (vel_right - vel_left) / self.wheel_sep
        # Calculate the robot's new position and orientation
        delta_theta = angular_velocity * (self.rate)
        delta_x = linear_velocity * cos(self.theta + delta_theta/2) * (self.rate)
        delta_y = linear_velocity * sin(self.theta + delta_theta/2) * (self.rate)
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        # Set the last left and right wheel encoder readings
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        #print([self.x, self.y])
        return[self.x,self.y]"""

