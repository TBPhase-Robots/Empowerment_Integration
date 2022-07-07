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

# def voronoi_plot_agents(flock, herd):
#     sheep_positions = []
#     dog_positions = []

#     xs = []
#     ys = []

#     for sheep in flock:
#         xs.append(sheep.position[0])
#         ys.append(sheep.position[1])
#         sheep_positions.append(sheep.position)

#     for dog in herd:
#         dog_positions.append(dog.position)
    
#     vor = Voronoi(dog_positions)
#     fig = voronoi_plot_2d(vor)
    
#     for dog in herd:
#         plt.scatter([dog.position[0]], [dog.position[1]], color=np.array(colours.ERANGE[dog.id]) / 255)

#     for sheep in flock:
#         plt.scatter([sheep.position[0]], [sheep.position[1]], color=np.array(colours.ERANGE[sheep.closest_dog.id]) / 255)


#     plt.scatter(xs, ys, color='red', s=3)
#     plt.gca().invert_yaxis()
#     plt.show()
# #end function

def main():
    with open(config_file_name) as json_file:
        cfg = json.load(json_file)

    global screen

    pygame.init()

    screen = pygame.display.set_mode([cfg['screen_width'],cfg['screen_width']])
    herd = pygame.sprite.Group()
    flock = pygame.sprite.Group()

    agent = Dog(position = np.array([600, 410.76]), id = 0, cfg = cfg)
    herd.add(agent)
    agent = Dog(position = np.array([600, 408.76]), id = 1, cfg = cfg)
    herd.add(agent)
    agent = Dog(position = np.array([400, 710.76]), id = 2, cfg = cfg)
    herd.add(agent)
    agent = Dog(position = np.array([870, 580]), id = 3, cfg = cfg)
    herd.add(agent)
    agent = Dog(position = np.array([280, 710.76]), id = 4, cfg = cfg)
    herd.add(agent)

    agent = Sheep(position = np.array([880, 580]), id = 3, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([600, 600]), id = 1, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([60, 560]), id = 2, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([280, 600]), id = 3, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([370, 370]), id = 4, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([810, 580]), id = 5, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([120, 600]), id = 6, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([60, 760]), id = 7, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([280, 20]), id = 8, cfg = cfg)
    flock.add(agent)
    agent = Sheep(position = np.array([390, 370]), id = 9, cfg = cfg)
    flock.add(agent)

    while True:
        for event in pygame.event.get():
            if event.type==QUIT:
                pygame.quit()
                sys.exit()

        screen.fill(colours.GREY)

        calc_voronoi_partitioning(flock, herd)
        #voronoi_plot_agents(flock, herd)

        herd.update(screen, flock, herd, cfg)
        flock.update(screen, flock, herd, cfg)

        pygame.display.flip()
        #pygame.display.update()

        pygame.time.wait(10)

        for sheep in flock:
            sheep.do_stuff = True
#end function

main()