import pygame
import numpy as np
import colours
import sys
from model.Agent import Agent
import random
import math

from model.Sheep import Sheep

class Dog(Agent):

    def __init__(self, position, id, cfg) -> None:
        super().__init__(position, id, cfg)
        self.sub_flock = pygame.sprite.Group()
        self.direction = np.array([1, 0])
        self.choice_tick_count = 0
        self.target_sheep = None
        self.driving_point = np.zeros(2)
        self.state = 'collecting'
        self.steering_point = np.zeros(2)
        self.empowerment = 0
    #end function 

    def update(self, screen, flock, pack, cfg):
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

        F_H = self.calc_F_H(screen, cfg, self.steering_point, flock)
        F_D = self.calc_F_D(pack)
        
        F = (cfg['dog_forces_with_flock'] * F_H) + (cfg['dog_repulsion_from_dogs'] * F_D)

        if (cfg['debug_dog_forces']):
            pygame.draw.line(screen, colours.ORANGE, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_dogs'] * F_D), 8)
        if (cfg['debug_steering_points']):
            pygame.draw.circle(screen, colours.BLACK, self.steering_point, 4)

        self.position = np.add(self.position, F)

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
                    self.empowerment += 5

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

    def calc_F_D(self, pack):
        F_D_D = np.zeros(2)
        for dog in pack:
            if (dog.id != self.id):
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

    def calc_F_H(self, screen, cfg, steering_point, flock):
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