#! /usr/bin/env python3
import numpy as np
import math


class BUG2:
    def __init__(self):
        self.regions = [0, 0, 0, 0, 0]
        self.flag = 1   #moving forward or turning
        self.pose_flag = 0
        self.action = [0.0, 0.1]
        self.flag_1 = 0 # obstacle avoidance or moving to target
        self.first = True
        self.colission_distance = 0.18 * 3
        
    def reset(self):
        self.regions = [0, 0, 0, 0, 0]
        self.flag = 1 #moving forward or turning
        self.pose_flag = 0
        self.action = [0.0, 0.1]
        self.flag_1 = 0 # obstacle avoidance or moving to target
        self.first = True
        self.colission_distance = 0.18 * 3

    def angle_towards_goal(self, angle):
        difference_angle = angle
        if math.fabs(difference_angle) > 0.05:
            self.action[0] = 0.1 if difference_angle > 0 else -0.1
            self.action[1] = 0.0
        if math.fabs(difference_angle) <= 0.05:
            self.flag_shift(1)

    def obstacle_avoidance(self):
        reg_values = self.regions
        if reg_values[2] < self.colission_distance and reg_values[3] < self.colission_distance and reg_values[1] < self.colission_distance:
            self.action[1] = 0.005 # obstacles on all sides, stop and turn
            self.action[0] = -0.3 /4
        elif reg_values[2] < self.colission_distance and reg_values[3] < self.colission_distance and reg_values[1] > self.colission_distance:
            self.action[1] = 0.005  #obstacless on the front and right, turn left
            self.action[0] = 0.4/ 4
        elif reg_values[2] < self.colission_distance and reg_values[3] > self.colission_distance and reg_values[1] < self.colission_distance:
            self.action[1] = 0.005 # obstacles in the front and left, turn right
            self.action[0] = -0.4 /4  # -0.4
        elif reg_values[2] > self.colission_distance and reg_values[3] > self.colission_distance and reg_values[1] > self.colission_distance:
            self.action[1] = 0.1
            self.action[0]= 0.
        elif reg_values[2] > self.colission_distance and reg_values[3] < self.colission_distance and reg_values[1] > self.colission_distance:
            self.action[1] = 0.3 / 4 # obstacles in left, turn right and go forward
            self.action[0] = -0.2  / 4 
        elif reg_values[2] < self.colission_distance and reg_values[3] > self.colission_distance and reg_values[1] > self.colission_distance:
            self.action[1] = 0.005    # obstacles in front, stop and turn
            self.action[0] = -0.3 / 4
        elif reg_values[2] > self.colission_distance and reg_values[3] > self.colission_distance and reg_values[1] < self.colission_distance:
            self.action[1] = 0.4 / 4 # obstacles right, turn left and go forward
            self.action[0] = 0.1/ 4 

    def flag_shift(self, f):
        self.flag = f

    def move(self, angle, distance):
        difference_angle = angle
        difference_pos = distance

        #move towards target
        if difference_pos > .1:
            self.action[1] = 0.05
            self.action[0]= 0. 
        else:
            self.flag_shift(2) # reached target
        # state change conditions
        if math.fabs(difference_angle) > 0.03:
            self.flag_shift(0)

    def laser_scan(self, laser_msg):
        laser_msg = np.array(laser_msg)

        self.regions = [
                  min(laser_msg[865:1081]), #Right 
                  min(laser_msg[649:864]),        #Front Right
                  min(laser_msg[433:648]),       #Front
                  min(laser_msg[216:432]),      #Front Left
                  min(laser_msg[0:215]),       #Left
                 ]

    def get_action(self, state):
        self.laser_scan(state[0:-2])
        reg_values = self.regions
        self.dist = state[-1]
        self.angle = state[-2]
        if (reg_values[2] < self.colission_distance or reg_values[3] < self.colission_distance or reg_values[1] < self.colission_distance):
                self.flag_1 = 1
                self.obstacle_avoidance()
        
        #if coliding and close to target
        elif self.dist < 4 and reg_values[2] < self.colission_distance:
            if self.dist < 0.5:
                self.action[0] = 0.0
                self.action[1] = 0.0
                self.angle_towards_goal(angle=state[-2])
                self.move(angle=state[-2], distance=state[-1])
                self.flag_1 = 0

            else:
                self.flag_1 = 1
                self.obstacle_avoidance()
        #if coliding, avoid obstacle#if not coliding and far to target
        elif self.dist > 4 or (reg_values[2] > self.colission_distance and reg_values[3] > self.colission_distance and reg_values[1] > self.colission_distance):
            if self.flag == 0:
                self.angle_towards_goal(angle=state[-2])

            elif self.flag == 1:
                self.move(angle=state[-2], distance=state[-1])
                
        elif self.dist < 4 and self.flag_1 == 1:
            if self.flag == 0:
                self.angle_towards_goal(angle=state[-2])

            elif self.flag == 1:
                self.move(angle=state[-2], distance=state[-1])

            self.flag_1 = 0

        elif self.dist < 0.5:
            self.action[0] = 0.0
            self.action[1] = 0.0
            self.angle_towards_goal(angle=state[-2])
            self.move(angle=state[-2], distance=state[-1])
            self.flag_1 = 0
        
        return self.action
