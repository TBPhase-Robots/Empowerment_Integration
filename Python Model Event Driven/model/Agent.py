import pygame
import colours
import numpy as np
from logging import root
import pygame
import numpy as np
import colours
import sys

import random
import math
from math import degrees, atan2


import numpy.linalg as LA


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

from model.Listener import Listener

from geometry_msgs.msg import Pose

class Agent(pygame.sprite.Sprite):
    
    
    def AgentCallback(self, msg):
        # decode position and rotation data, set agent position and rotation
        newPose = msg
        self.position[0] = newPose.position.x
        self.position[1] = newPose.position.y
        self.rotation = newPose.orientation.z
        
        self.callback(msg)

    # call this function from central poller
    def RosUpdate(self):
        rclpy.spin_once(self.listener, timeout_sec=0)


    def __init__(self, position, id, cfg, rotation, callback) -> None:
        
        pygame.sprite.Sprite.__init__(self)
        
        # generic
        self.direction = np.array([1, 0])
        self.rotation = rotation
        self.callback = callback
        self.position = position
        self.id = id
        self.cfg = cfg
        self.topicString = "/robot" + str(self.id) + "/pose"

        # dog
        self.sub_flock = pygame.sprite.Group()
        self.choice_tick_count = 0
        self.target_sheep = None
        self.driving_point = np.zeros(2)
        self.state = 'collecting'
        self.steering_point = np.zeros(2)
        self.empowerment = 0
        

        # sheep 
        self.closest_dog = None
        self.grazing = True
        self.grazing_direction = np.array([1, 0])

        
        self.listener = Listener(self.topicString, self.AgentCallback) 


        

    #end function





    # a and b must be np arrays
    def CalcAngleBetweenVectors(self, a, b):
        # NORMALISE A AND B
        a = a / np.linalg.norm(a)
        b = b / np.linalg.norm(b)
        dot = np.dot(a, b)
        if (dot > 1):
            dot = 1
        theta = np.arccos(dot)
        if ((np.cross([a[0], a[1], 0], [b[0], b[1], 0])[2] > 0)   ):
            theta = - theta
        return math.degrees(theta)

   


    def CalcBearing(x, y, center_x, center_y):
        angle = degrees(atan2(y - center_y, x - center_x))
        bearing1 = (angle + 360) % 360
        bearing2 = (90 - angle) % 360
        return bearing1, bearing2
    #end function 






    def SimulationUpdate_Dog(self, screen, flock, pack, cfg):
        target = cfg['target_position']
        if (len(self.sub_flock) > 0):
            sheep_positions = []
            for sheep in flock:
                sheep_positions.append(sheep.position)
            C = Agent.calcCoM(self, sheep_positions)
            furthest_sheep_position = C

            if (self.choice_tick_count == 0):
                self.driving_point = np.add(C, cfg['driving_distance_from_flock_radius'] * (C - target) / np.linalg.norm(C - target))
                for sheep in self.sub_flock:
                    if (np.linalg.norm(sheep.position - C) > np.linalg.norm(furthest_sheep_position - C)):
                        furthest_sheep_position = sheep.position
                        self.target_sheep = sheep
            
            try:
                furthest_sheep_position = self.target_sheep.position
            except:
                furthest_sheep_position = C

            if (self.choice_tick_count == 0):
                if (np.linalg.norm(furthest_sheep_position - C) < cfg['collection_radius']):
                    self.state = 'driving'
                else:
                    self.state = 'collecting'

            if (self.state == 'driving'):
                self.steering_point = self.driving_point
            elif (self.state == 'collecting'):
                self.steering_point = np.add(furthest_sheep_position, cfg['collection_distance_from_target_sheep'] * (furthest_sheep_position - C) / np.linalg.norm(furthest_sheep_position - C))
        else:
            self.state = 'unassigned'
            sheep_positions = []
            for sheep in flock:
                sheep_positions.append(sheep.position)
            C = Agent.calcCoM(self, sheep_positions)
            furthest_sheep_position = C
            
            if (self.choice_tick_count == 0):
                for sheep in flock:
                    if (np.linalg.norm(sheep.position - C) > np.linalg.norm(furthest_sheep_position - C)):
                        furthest_sheep_position = sheep.position
            
            outer_flock_radius_point = np.add(C, np.linalg.norm(C - furthest_sheep_position) * ((C - target) / np.linalg.norm(C - target)))
            self.steering_point = np.add(outer_flock_radius_point, cfg['driving_distance_from_flock_radius'] * ((C - target) / np.linalg.norm(C - target)))

        F_H = self.calc_F_H_Dog(screen, cfg, self.steering_point, flock)
        F_D = self.calc_F_D_Dog(pack)
        
        F = (cfg['dog_forces_with_flock'] * F_H) + (cfg['dog_repulsion_from_dogs'] * F_D)

        if (cfg['debug_dog_forces']):
            pygame.draw.line(screen, colours.ORANGE, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_dogs'] * F_D), 8)
        if (cfg['debug_steering_points']):
            pygame.draw.circle(screen, colours.BLACK, self.steering_point, 4)


        # calculate forward vector
        forwardX = math.sin(self.rotation)
        forwardY = math.cos(self.rotation)

        if(cfg['realistic_agent_movement_markers']):
            # black line is target rotation
            pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, np.array(F)*10) ,8)
            # draw line in forward vector
            pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*80) ,5)

        # calculate angle between current dir and target dir:
        angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), np.array(F))

        # F is the movement vector for this frame.

        # Unrealistic movement
        if(not cfg['realistic_agent_movement']):
            self.position = np.add(self.position, F)
        # Realistic movement
        else:
            # differential drive rotation towards target direction

            # rotate until forward vector is parallel to force within reason

            # if vector is parallel, then go forward
            if(angle > 5):
                self.rotation -= 0.1
                self.position = np.add(self.position, [2*forwardX, -2*forwardY])
            elif(angle < -5):
                self.rotation += 0.1
                self.position = np.add(self.position, [2*forwardX, -2*forwardY])
            else:
                self.position = np.add(self.position, F)

            # at this point, we would attempt to transmit this agent's movement command to the bot via ROS
            
            # we should transmit the current position, current rotation, target position. 
            # The robot will attempt to drive to the position, then keep going

        self.choice_tick_count += 1
        if (self.choice_tick_count >= cfg['ticks_per_choice']):
            self.choice_tick_count = 0

        collision_check = True
        while (collision_check):
            collision_check = False
            for dog in pack:
                if (dog.id != self.id):
                    if (np.linalg.norm(self.position - dog.position) <= 8):
                        self.position = np.add(self.position, self.position - dog.position)
                        collision_check = True
        
        if (self.position[0] > cfg['world_width'] - 10): self.position[0] = cfg['world_width'] - 10
        elif (self.position[0] < 10): self.position[0] = 10

        if (self.position[1] > cfg['world_height'] - 10): self.position[1] = cfg['world_height'] - 10
        elif (self.position[1] < 10): self.position[1] = 10

        if (cfg['empowerment_type'] == 0):
            self.empowerment = len(self.sub_flock)
        elif (cfg['empowerment_type'] == 1):
            if (len(self.sub_flock) > 0):
                self.empowerment = 5
            else:
                self.empowerment = 0
            for sheep in flock:
                if (np.linalg.norm(self.position - sheep.position) <= 50):
                    self.empowerment += 5 - math.floor(np.linalg.norm(self.position - sheep.position) / 10)

        super().update(screen)
        if (cfg['debug_dog_states']):
            if (self.state == 'driving'):
                pygame.draw.circle(screen, colours.DRIVE, self.position, 5)
            elif (self.state == 'collecting'):
                pygame.draw.circle(screen, colours.COLLECT, self.position, 5)
            else:
                pygame.draw.circle(screen, colours.BLUE, self.position, 5)
        else:
            if (cfg['show_empowerment']):
                if (self.empowerment < 5):
                    colour = np.array([155 + round(100 * self.empowerment / 5), 0, 0])
                elif (self.empowerment < 10):
                    colour = np.array([255, round(255 * (self.empowerment - 5) / 5), 0])
                elif (self.empowerment < 15):
                    colour = np.array([255 - round(255 * (self.empowerment - 10) / 5), 255, 0])
                elif (self.empowerment < 20):
                    colour = np.array([0, 255 - round(100 * (self.empowerment - 15) / 5), 0])
                else:
                    colour = np.array([0, 155, 0])
                pygame.draw.circle(screen, colour, self.position, 5)
            else:
                pygame.draw.circle(screen, colours.BLUE, self.position, 5)

        if (cfg['debug_sub_flocks']):
            if (self.id < 5):
                pygame.draw.circle(screen, colours.SRANGE[self.id], self.position, 4)
            else:
                pygame.draw.circle(screen, colours.BLACK, self.position, 4)
    #end function

    def empty_sub_flock(self):
        self.sub_flock.empty()
    #end function

    def add_sheep_to_sub_flock(self, sheep):
        self.sub_flock.add(sheep)
    #end function

    def calc_F_D_Dog(self, pack):
        F_D_D = np.zeros(2)
        for dog in pack:
            if (dog.id != self.id):
                if (np.array_equal(self.position, dog.position)):
                    F_D_D = np.add(F_D_D, (self.position - dog.position) / np.linalg.norm(self.position - dog.position))

        F_D = F_D_D + (0.75 * np.array([F_D_D[1], -F_D_D[0]]))
        return F_D
    #end function

    def sine_step(self, theta):
        if ((-math.pi < theta and -math.pi / 2 >= theta) or (math.pi < theta and 3 * math.pi / 2 >= theta)):
            return 1
        elif ((-3 * math.pi / 2 < theta and -math.pi >= theta) or (math.pi / 2 < theta and math.pi >= theta)):
            return -1
        else:
            return -math.sin(theta)
    #end function

    def calc_F_H_Dog(self, screen, cfg, steering_point, flock):
        sheep_positions = []
        if (len(self.sub_flock) > 0):
            for sheep in self.sub_flock:
                sheep_positions.append(sheep.position)
        else:
            for sheep in flock:
                sheep_positions.append(sheep.position)
        C = Agent.calcCoM(self, sheep_positions)
        W = steering_point

        R_C_D = (self.position - C) / np.linalg.norm(self.position - C)
        R_C_W = (W - C) / np.linalg.norm(W - C)

        dot = np.dot(R_C_D, R_C_W)
        if (dot > 1):
            dot = 1
        theta_D_C_W = np.arccos(dot)
        if (np.cross([R_C_D[0], R_C_D[1], 0], [R_C_W[0], R_C_W[1], 0])[2] < 0):
            theta_D_C_W = - theta_D_C_W

        R_D_W = (W - self.position) / np.linalg.norm(W - self.position)
        R_D_T = np.array([R_C_D[1], -R_C_D[0]]) 

        H_F = 1 - math.exp(-2 * abs(math.degrees(theta_D_C_W)))
        H_T = self.sine_step(theta_D_C_W)

        sum = np.zeros(2)
        for sheep in flock:
            sum = np.add(sum, (self.position - sheep.position) / (2 *np.linalg.norm(self.position - sheep.position)))

        F_F = H_F * sum
        F_W = R_D_W
        F_T = H_T * R_D_T

        if (cfg['debug_dog_forces']):
            pygame.draw.line(screen, colours.GREEN, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_sheep'] * F_F), 8)
            pygame.draw.line(screen, colours.RED, self.position, np.add(self.position, 10 * cfg['dog_attraction_to_steering_point'] * F_W), 8)
            pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, 10 * cfg['dog_orbital_around_flock'] * F_T), 8)
        F_H = (cfg['dog_repulsion_from_sheep'] * F_F) + (cfg['dog_attraction_to_steering_point'] * F_W) + (cfg['dog_orbital_around_flock'] * F_T)

        return F_H
    #end function

    def calcCoM(self, vector_list):
        #Calculates the centre of mass as the average position of the
        #vectors listed in vector_list
        #vector_list = [x1,y1;x2,y2;....]

        if np.any(vector_list):
            V = np.atleast_2d(vector_list)
            N = V.shape[0]
            com = np.sum(vector_list,axis=0)/N
        else:
            com = np.array([])
        return com
    #end function


    # ================================ SHEEP =================================#


    def SimulationUpdate_Sheep(self, screen, flock, pack, cfg):

        # calculate forward vector
        forwardX = math.sin(self.rotation)
        forwardY = math.cos(self.rotation)



        

        
        if (self.closest_dog != None):
            if (np.linalg.norm(self.position - self.closest_dog.position) <= cfg['sheep_vision_range']):
                self.grazing = False
            else:
                if (random.random() < 0.05):
                    self.grazing_direction = np.array([random.uniform(-3, 3), random.uniform(-3, 3)])
                self.grazing = True
        else:
            self.grazing = True
            if (random.random() < 0.05):
                self.grazing_direction = np.array([random.uniform(-3, 3), random.uniform(-3, 3)])


        if (self.grazing):
            if (len(flock) > 1):
                F_S = self.calc_F_S_Sheep(flock, cfg)
            else:
                F_S = 0
            self.position = np.add(self.position, (cfg['sheep_repulsion_from_sheep'] * F_S))

            angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), self.grazing_direction)
            if(cfg['realistic_agent_movement_markers']):
                # black line is target rotation
                pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, self.grazing_direction*5) ,8)
                # draw line in forward vector
                pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*30) ,5)

            if (random.random() < cfg['grazing_movement_chance']):
                if(not cfg['realistic_agent_movement']):
                    self.position = np.add(self.position, self.grazing_direction)
                else:
                    if(angle > 10):
                        self.rotation -= 0.4
                        self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                    elif(angle < -10):
                        self.rotation += 0.4
                        self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                    else:
                        self.position = np.add(self.position, self.grazing_direction)
                    
        else:
            F_D = self.calc_F_D_Sheep(pack, cfg)
            if (len(flock) > 1):
                F_S = self.calc_F_S_Sheep(flock, cfg)
                F_G = self.cal_F_G_Sheep(flock, cfg)
            else:
                F_S = 0
                F_G = 0

            F = (cfg['sheep_resulsion_from_dogs'] * F_D) + (cfg['sheep_repulsion_from_sheep'] * F_S) + (cfg['sheep_attraction_to_sheep'] * F_G)

            angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), np.array(F))
            if(cfg['realistic_agent_movement_markers']):
                # black line is target rotation
                pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, self.grazing_direction*8) ,8)
                # draw line in forward vector
                pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*40) ,5)


            if(not cfg['realistic_agent_movement']):
                self.position = np.add(self.position, F)
            else:
                if(angle > 10):
                    self.rotation -= 0.2
                    self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                elif(angle < -10):
                    self.rotation += 0.2
                    self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                else:
                    self.position = np.add(self.position, np.array(F))

            if (cfg['debug_sheep_forces']):
                pygame.draw.line(screen, colours.ORANGE, self.position, np.add(self.position, 10 * cfg['sheep_resulsion_from_dogs'] * F_D), 8)
                pygame.draw.line(screen, colours.GREEN, self.position, np.add(self.position, 10 * cfg['sheep_repulsion_from_sheep'] * F_S), 8)
                pygame.draw.line(screen, colours.RED, self.position, np.add(self.position, 10 * cfg['sheep_attraction_to_sheep'] * F_G), 8)

        collision_check = True
        while (collision_check):
            collision_check = False
            for sheep in flock:
                if (sheep.id != self.id):
                    if (np.linalg.norm(self.position - sheep.position) <= 8):
                        self.position = np.add(self.position, self.position - sheep.position)
                        collision_check = True

        if (self.position[0] > cfg['world_width'] - 10): self.position[0] = cfg['world_width'] - 10
        elif (self.position[0] < 10): self.position[0] = 10

        if (self.position[1] > cfg['world_height'] - 10): self.position[1] = cfg['world_height'] - 10
        elif (self.position[1] < 10): self.position[1] = 15

        super().update(screen)
        if (cfg['debug_sheep_states']):
            if (self.grazing):
                pygame.draw.circle(screen, colours.GRAZE, self.position, 5)
                
            else:
                pygame.draw.circle(screen, colours.HERD, self.position, 5)
        else:
            pygame.draw.circle(screen, colours.WHITE, self.position, 5)
        if (cfg['debug_sub_flocks']):
            if (self.closest_dog != None):
                if (self.closest_dog.id < 5):
                    pygame.draw.circle(screen, colours.SRANGE[self.closest_dog.id], self.position, 4)
                else:
                    pygame.draw.circle(screen, colours.BLACK, self.position, 4)
    #end function

    def set_closest_dog(self, dog):
        self.closest_dog = dog
    #end function

    def calc_F_D_Sheep(self, pack, cfg):
        sum = np.zeros(2)
        for dog in pack:
            direction = self.position - dog.position
            magnitude = np.linalg.norm(direction)
            if (magnitude != 0):
                sum += (direction / magnitude) * math.exp(- cfg['lambda_D'] * magnitude)
        return sum
    #end function

    def calc_F_S_Sheep(self, flock, cfg):
        sum = np.zeros(2)
        for sheep in flock:
            if (sheep.id != self.id):  
                direction = self.position - sheep.position
                magnitude = np.linalg.norm(direction)
                if (magnitude != 0):
                    sum += (direction / magnitude) * math.exp(- cfg['lambda_S'] * magnitude)
        return sum
    #end function

    def cal_F_G_Sheep(self, flock, cfg):
        sheep_positions_ordered = []
        for sheep in flock:
            if (sheep.id != self.id):
                if not sheep_positions_ordered:
                    sheep_positions_ordered.append(sheep.position) #if list is empty
                else:
                    for i in range(0, len(sheep_positions_ordered)):
                        if (np.linalg.norm(sheep.position - self.position) < np.linalg.norm(self.position - sheep_positions_ordered[i])):
                            sheep_positions_ordered.insert(i, sheep.position)
                            break
                        else:
                            if (i == len(sheep_positions_ordered) - 1):
                                sheep_positions_ordered.append(sheep.position)

        social_group_positions = sheep_positions_ordered[:cfg['no_of_sheep_in_social_group']]      
        external_group_positions = sheep_positions_ordered[cfg['no_of_sheep_in_social_group']:]                            

        sheep_positons = []
        for sheep in flock:
            if (sheep.id != self.id):
                sheep_positons.append(sheep.position)

        C = Agent.calcCoM(self, sheep_positons)
        C_i = Agent.calcCoM(self, social_group_positions)
        C_i_prime = Agent.calcCoM(self, external_group_positions)
        
        C_direction = C - self.position
        C_magnitude = np.linalg.norm(C_direction)

        if (cfg['lambda_G'] > 0):
            C_i_direction = C_i - self.position
            C_i_magnitude = np.linalg.norm(C_i_direction)
            F_G = (cfg['lambda_G'] * (C_i_direction / C_i_magnitude)) + ((1 - cfg['lambda_G']) * (C_direction / C_magnitude))
        else:
            if (len(external_group_positions) > 0):
                C_i_prime_direction = C_i_prime - self.position
                C_i_prime_magnitude = np.linalg.norm(C_i_prime_direction)
                F_G = (-cfg['lambda_G'] * (C_i_prime_direction / C_i_prime_magnitude)) + ((1 + cfg['lambda_G']) * (C_direction / C_magnitude))
            else:
                F_G = 0

        return F_G
    #end function





