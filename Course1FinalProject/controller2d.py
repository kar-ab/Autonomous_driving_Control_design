#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

        
    def set_yaw_range(self, yaw): 
        if yaw > self._pi:
            yaw -= self._2pi
        if yaw < - self._pi:
            yaw += self._2pi
        return yaw
    
    
    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        # longitudinal control variables
        
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('v_delta_previous', 0.0) 
        self.vars.create_var('t_previous',0)
        self.vars.create_var('i_factor_previous',0) 
        # Kp, ki and kd gain values are chosen by trial and error method
        self.vars.create_var('kp', 1)
        self.vars.create_var('ki', 1)
        self.vars.create_var('kd', 0.01)
        
        
        # lateral control variables
        
            
        
        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            throttle_output = 0
            brake_output    = 0
            
            # High level controller: 
                # input: v_desired and current velocity
                # output: accelration
            # Low level Controller:
                # input: acceleration
                # output: throttle and brake
            
            # Implementation of Longitudinal speed control using PID
            # reference: https://projects.raspberrypi.org/en/projects/robotPID/3 
            
            dt = t - self.vars.t_previous
            v_delta = v_desired - v

            # Proportional = kp * error 
            # Integral = ki * integrate error over time
            # Derivative = kd * derivate error with repect to time
            
            # Discretization: 
            # integration is dicretized to summation of error over time 
            # derivate = difference between with repect to previous and current time step
            
            p_factor = v_delta 
            i_factor = self.vars.i_factor_previous + v_delta * dt
            d_factor = (v_delta - self.vars.v_delta_previous) / dt
            
            desired_acc = self.vars.kp * p_factor + self.vars.ki * i_factor + self.vars.kd * d_factor
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            
            # low level convert neeeds torque load and desired accleraration to compute the output throttle
            # since torque load is not given, desired acceleration is used to set throttle output.
            # but this isn't true in practise, it is only done for the sake of this assignment.
            
            # throttle and brake is clamped to range 0 to 1 using set_throttle method
            
            if desired_acc >= 0:
                throttle_output = desired_acc
                brake_output    = 0
            else: 
                throttle_output = 0
                brake_output    = -desired_acc

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            steer_output = 0
        
            # To implement Stanley controller for lateral control, following things are needed
            
            # 1. heading_yaw_error = trajectory_heading - vehicle heading (i.e. yaw)
            # 2. cross_track_error = nearest point to the trajectory line from the front axle of vehicle
                                     # or perpendicular distance from trajectory line to front axle
            # 3. max_steering_angle_bounds = (-pi to pi) 
            
            # trajectory line = previous waypoint and current waypoint
            # slope of trajectory line = (x1-x2) / (y1-y2)
            # yaw is also referred to as heading
            # waypoint_heading / trajectory_heading = tan_inverse (slope of line)
                      
            k_cte = 0.3 # proportial gain for cross track error (determined experimentally)
            k_soft =10 # softening constant to increase stability of vehicle at low speeds
            waypoint_yaw = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
            
            # 1. Align heading to desired heading (proportional to heading error)
            heading_yaw_diff = waypoint_yaw - yaw 
            heading_yaw_diff = self.set_yaw_range(heading_yaw_diff)

            # 2. Eliminate cross track error 
            cross_track_error = np.min(np.sum((np.array([x,y]) - np.array(waypoints)[:, :2])**2, axis=1))
            cross_track_yaw = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
            cros_path_diff = waypoint_yaw - cross_track_yaw 
            cros_path_diff = self.set_yaw_range(cros_path_diff)
            
            if cros_path_diff > 0:
                cross_track_error = abs(cross_track_error)
            else:
                cross_track_error = - abs(cross_track_error)

            cross_track_yaw_diff = np.arctan(k_cte * cross_track_error / (k_soft + v))
            cros_path_diff = self.set_yaw_range(cros_path_diff)
            steering_value = heading_yaw_diff + cross_track_yaw_diff
            
            # 3. Maximum and Minimum steering angles
            steering_value = min(1.22, steering_value)
            steering_value = max(-1.22, steering_value)
            
            steer_output = steering_value
            
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t 
        self.vars.i_factor_previous = i_factor
        self.vars.v_delta_previous = v_delta
        
        
       