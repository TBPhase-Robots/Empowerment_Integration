from site import sethelper
from turtle import fd
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
        self.furthest_sheep_tick_count = 0
        self.target_sheep = Sheep
    #end function 

    def update(self, screen, flock, herd, cfg):
        if (len(self.sub_flock) > 0):
            sheep_positions = []
            for sheep in flock:
                sheep_positions.append(sheep.position)
            C = Agent.calcCoM(self, sheep_positions)
            furthest_sheep_position = C

            if (self.furthest_sheep_tick_count == 0):
                for sheep in self.sub_flock:
                    if (np.linalg.norm(sheep.position - C) > np.linalg.norm(furthest_sheep_position - C)):
                        furthest_sheep_position = sheep.position
                        self.target_sheep = sheep
            
            furthest_sheep_position = self.target_sheep.position

            self.furthest_sheep_tick_count += 1
            if (self.furthest_sheep_tick_count >= cfg['ticks_per_target_sheep']):
                self.furthest_sheep_tick_count = 0

            steering_point = np.add(furthest_sheep_position, 20 * (furthest_sheep_position - C) / np.linalg.norm(furthest_sheep_position - C))

            #print(self.position, steering_point, furthest_sheep_position, C)

            F_H = self.calc_F_H(screen, cfg, steering_point, flock)
            F_D = self.calc_F_D(herd)
            
            F = (cfg['dog_forces_with_flock'] * F_H) + (cfg['dog_repulsion_from_dogs'] * F_D)

            #pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, 10 * cfg['dog_forces_with_flock'] * F_H), 8)
            #pygame.draw.line(screen, colours.ORANGE, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_dogs'] * F_D), 8)
            pygame.draw.circle(screen, colours.BLACK, steering_point, 4)

            self.position = np.add(self.position, F)
            #print(self.position)

        super().update(screen)
        pygame.draw.circle(screen, colours.BLUE, self.position, 5)
        #pygame.draw.circle(screen, colours.SRANGE[self.id], self.position, 4)
    #end function

    def calcCoM(self, vector_list):
        super().calcCoM()
    #end function

    def empty_sub_flock(self):
        self.sub_flock.empty()
    #end function

    def add_sheep_to_sub_flock(self, sheep):
        self.sub_flock.add(sheep)
    #end function

    def calc_F_D(self, herd):
        F_D_D = np.zeros(2)
        for dog in herd:
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
        for sheep in self.sub_flock:
            sheep_positions.append(sheep.position)
        C = Agent.calcCoM(self, sheep_positions)
        W = steering_point

        #print(self.position, C, W)

        R_C_D = (self.position - C) / np.linalg.norm(self.position - C)
        R_C_W = (W - C) / np.linalg.norm(W - C)

        dot = np.dot(R_C_D, R_C_W)
        #print(R_C_D, R_C_W, dot)
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

        #print((cfg['dog_repulsion_from_sheep'] * F_F) , (cfg['dog_attraction_to_steering_point'] * F_W) , (cfg['dog_orbital_around_flock'] * F_T))
        # pygame.draw.line(screen, colours.GREEN, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_sheep'] * F_F), 8)
        # pygame.draw.line(screen, colours.RED, self.position, np.add(self.position, 10 * cfg['dog_attraction_to_steering_point'] * F_W), 8)
        # pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, 10 * cfg['dog_orbital_around_flock'] * F_T), 8)
        F_H = (cfg['dog_repulsion_from_sheep'] * F_F) + (cfg['dog_attraction_to_steering_point'] * F_W) + (cfg['dog_orbital_around_flock'] * F_T)

        return F_H
    #end function