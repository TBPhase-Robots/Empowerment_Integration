import pygame
import numpy as np
import colours
import sys
from model.Agent import Agent
import random

class Dog(Agent):

    def __init__(self, position, id, cfg) -> None:
        super().__init__(position, id, cfg)
        self.sub_flock = pygame.sprite.Group()
        self.direction = np.array([1, 0])
    #end function

    def update(self, screen, flock, herd, cfg):
        if (random.randint(0, 100) < 1):
            self.direction = np.array([random.randint(-1, 1), random.randint(-1, 1)])

        if (self.position[0] + self.direction[0] < 0 or self.position[0] + self.direction[0] > cfg['screen_width']):
            self.direction[0] = -self.direction[0]
        
        if (self.position[1] + self.direction[1] < 0 or self.position[1] + self.direction[1] > cfg['screen_height']):
            self.direction[1] = -self.direction[1]
        
        self.position = np.add(self.position, self.direction)

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