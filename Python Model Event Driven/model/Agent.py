import pygame
import colours
import numpy as np

class Agent(pygame.sprite.Sprite):
    def __init__(self, position, id, cfg, rotation, callback) -> None:
        pygame.sprite.Sprite.__init__(self)
        self.position = position
        self.id = id
        self.cfg = cfg
        self.rotation = rotation
        self.callback = callback
        
    #end function



    def update(self, screen):
        pygame.draw.circle(screen, colours.BLACK, self.position, 7)
        
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





