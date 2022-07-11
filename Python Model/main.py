from cv2 import AgastFeatureDetector_NONMAX_SUPPRESSION
import pygame
import colours
import sys
from pygame.locals import *
import numpy as np
from model.Dog import Dog
from model.Sheep import Sheep
import json
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import random

config_file_name = 'config.json'

def calc_voronoi_partitioning(flock, herd):
    for dog in herd:
        dog.empty_sub_flock()

    for sheep in flock:
        min_dist = 10000
        for dog in herd:
            dist = np.linalg.norm(sheep.position - dog.position)
            if dist < min_dist:
                min_dist = dist
                sheep.set_closest_dog(dog)
        sheep.closest_dog.add_sheep_to_sub_flock(sheep)
#end function

def add_sheep(flock, position, cfg, flock_id):
    agent = Sheep(position = position, id = flock_id, cfg = cfg)
    flock.add(agent)
    return flock_id + 1
#end function

def add_dog(herd, position, cfg, herd_id):
    agent = Dog(position = position, id = herd_id, cfg = cfg)
    herd.add(agent)
    return herd_id + 1
#end function

def main():
    with open(config_file_name) as json_file:
        cfg = json.load(json_file)

    global screen

    pygame.init()

    screen = pygame.display.set_mode([cfg['screen_width'],cfg['screen_width']])
    herd = pygame.sprite.Group()
    flock = pygame.sprite.Group()
    herd_id = 0
    flock_id = 0

    herd_id = add_dog(herd, np.array([400, 100]), cfg, herd_id)
    #herd_id = add_dog(herd, np.array([100, 400]), cfg, herd_id)
    #herd_id = add_dog(herd, np.array([700, 400]), cfg, herd_id)
    #herd_id = add_dog(herd, np.array([400, 700]), cfg, herd_id)

    for i in range(0, 30):
        flock_id = add_sheep(flock, np.array([random.uniform(cfg['screen_width'] / 2 - 200, cfg['screen_width'] / 2 + 200), random.uniform(cfg['screen_height'] / 2 - 200, cfg['screen_height'] / 2 + 200)]), cfg, flock_id)

    while True:
        for event in pygame.event.get():
            if event.type==QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    herd_id = add_dog(herd, np.array([event.pos[0], event.pos[1]]), cfg, herd_id)
                elif event.button == 3:
                    if (len(herd) > 0):
                        closest_dog = None
                        for dog in herd:
                            if closest_dog == None:
                                closest_dog = dog
                            else:
                                if (np.linalg.norm(np.array([event.pos[0], event.pos[1]]) - dog.position)) < np.linalg.norm(np.array([event.pos[0], event.pos[1]] - closest_dog.position)):
                                    closest_dog = dog
                        herd.remove(closest_dog)


        screen.fill(colours.GREY)

        if (len(herd) > 0):
            calc_voronoi_partitioning(flock, herd)
            herd.update(screen, flock, herd, np.array([700, 700]), cfg)
        else:
            for sheep in flock:
                sheep.closest_dog = None

        flock.update(screen, flock, herd, cfg)

        pygame.display.flip()
        #pygame.display.update()

        pygame.time.wait(0)
#end function

main()