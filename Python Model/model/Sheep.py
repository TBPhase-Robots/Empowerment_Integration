import pygame
import numpy as np
import colours
from model.Agent import Agent
import math
import random
from model.Dog import Dog

class Sheep(Agent):

    def __init__(self, position, id, cfg) -> None:
        super().__init__(position, id, cfg)
        self.closest_dog = None
    #end function

    def update(self, screen, flock, herd, cfg):
        if (np.linalg.norm(self.position - self.closest_dog.position) <= cfg['sheep_vision_range']):
            F_D = self.calc_F_D(herd, cfg)
            F_S = self.calc_F_S(flock, cfg)
            F_G = self.cal_F_G(flock, cfg)

            F = (cfg['K_D'] * F_D) + (cfg['K_S'] * F_S) + (cfg['K_G'] * F_G)

            self.position = np.add(self.position, F)
        else:
            if (random.randint(0, 100) < 5):
                self.position = np.add(self.position, np.array([random.uniform(-1, 1), random.uniform(-1, 1)]))

        super().update(screen)
        pygame.draw.circle(screen, colours.WHITE, self.position, 5)
    #end function

    def set_closest_dog(self, dog):
        self.closest_dog = dog
    #end function

    def set_do_stuff(self, bool):
        self.do_stuff = bool

    def calcCoM(self, vector_list):
        super().calcCoM()
    #end function

    def calc_F_D(self, herd, cfg):
        sum = np.zeros(2)
        for dog in herd:
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
        C_i_direction = C_i - self.position
        C_i_magnitude = np.linalg.norm(C_i_direction)
        C_i_prime_direction = C_i_prime - self.position
        C_i_prime_magnitude = np.linalg.norm(C_i_prime_direction)

        if (cfg['lambda_G'] > 0):
            F_G = (cfg['lambda_G'] * (C_i_direction / C_i_magnitude)) + ((1 - cfg['lambda_G']) * (C_direction / C_magnitude))
        else:
            F_G = (-cfg['lambda_G'] * (C_i_prime_direction / C_i_prime_magnitude)) + ((1 + cfg['lambda_G']) * (C_direction / C_magnitude))

        return F_G
    #end function