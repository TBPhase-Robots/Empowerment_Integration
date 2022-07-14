from cv2 import magnitude
import pygame
import numpy as np
import colours
from model.Agent import Agent
import math
import random

class Sheep(Agent):

    def __init__(self, position, id, cfg) -> None:
        super().__init__(position, id, cfg)
        self.closest_dog = None
        self.grazing = True
        self.grazing_direction = np.array([1, 0])
    #end function

    def update(self, screen, flock, pack, cfg):
        if (self.closest_dog != None):
            if (np.linalg.norm(self.position - self.closest_dog.position) <= cfg['sheep_vision_range']):
                self.grazing = False
            else:
                if (random.random() < 0.05):
                    self.grazing_direction = np.array([random.uniform(-1, 1), random.uniform(-1, 1)])
                self.grazing = True
        else:
            self.grazing = True
            if (random.random() < 0.05):
                self.grazing_direction = np.array([random.uniform(-1, 1), random.uniform(-1, 1)])

        if (self.grazing):
            if (len(flock) > 1):
                F_S = self.calc_F_S(flock, cfg)
            else:
                F_S = 0
            self.position = np.add(self.position, (cfg['sheep_repulsion_from_sheep'] * F_S))
            if (random.random() < cfg['grazing_movement_chance']):
                self.position = np.add(self.position, self.grazing_direction)
        else:
            F_D = self.calc_F_D(pack, cfg)
            if (len(flock) > 1):
                F_S = self.calc_F_S(flock, cfg)
                F_G = self.cal_F_G(flock, cfg)
            else:
                F_S = 0
                F_G = 0

            F = (cfg['sheep_resulsion_from_dogs'] * F_D) + (cfg['sheep_repulsion_from_sheep'] * F_S) + (cfg['sheep_attraction_to_sheep'] * F_G)

            self.position = np.add(self.position, F)

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

    def calc_F_D(self, pack, cfg):
        sum = np.zeros(2)
        for dog in pack:
            direction = self.position - dog.position
            magnitude = np.linalg.norm(direction)
            sum += (direction / magnitude) * math.exp(- cfg['lambda_D'] * magnitude)
        return sum
    #end function

    def calc_F_S(self, flock, cfg):
        sum = np.zeros(2)
        for sheep in flock:
            if (sheep.id != self.id):  
                direction = self.position - sheep.position
                magnitude = np.linalg.norm(direction)
                sum += (direction / magnitude) * math.exp(- cfg['lambda_S'] * magnitude)
        return sum
    #end function

    def cal_F_G(self, flock, cfg):
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