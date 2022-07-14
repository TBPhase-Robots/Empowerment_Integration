import pygame
import colours
import numpy as np

class Agent(pygame.sprite.Sprite):
    def __init__(self, position, id, cfg, rotation) -> None:
        pygame.sprite.Sprite.__init__(self)
        self.position = position
        self.id = id
        self.cfg = cfg
        self.rotation = rotation
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






    # def update(self, screen):
    #     if (np.random.randint(0, 1000) == 0):
    #         self.direction = np.array([np.random.randint(-1, 2), np.random.randint(-1, 2)])
        
    #     self.position = np.add(self.position, 0.1 * self.direction)

    #     if (self.position[0] < 0):
    #         self.position[0] = 0
    #     if (self.position[1] < 0):
    #         self.position[1] = 0
    #     if (self.position[0] > config.SCREEN_WIDTH):
    #         self.position[0] = config.SCREEN_WIDTH
    #     if (self.position[1] > config.SCREEN_HEIGHT):
    #         self.position[1] = config.SCREEN_HEIGHT


    #     pygame.draw.circle(screen, colours.BLACK, self.position, 7)
    #     pygame.draw.circle(screen, colours.ERANGE[self.id], self.position, 5)
    # #end function