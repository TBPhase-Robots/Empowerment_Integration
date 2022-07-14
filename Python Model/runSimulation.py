from tkinter import Pack
import pygame

import colours
import sys
from pygame.locals import *
import numpy as np
from model.Dog import Dog
from model.Sheep import Sheep
from ProtoInputHandler import ProtoInputHandler
import json
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import random

config_file_name = 'config.json'




def calc_voronoi_partitioning(flock, pack):
    for dog in pack:
        dog.empty_sub_flock()

    for sheep in flock:
        min_dist = 10000
        for dog in pack:
            dist = np.linalg.norm(sheep.position - dog.position)
            if dist < min_dist:
                min_dist = dist
                sheep.set_closest_dog(dog)
        sheep.closest_dog.add_sheep_to_sub_flock(sheep)
#end function

def add_sheep(flock, position, cfg, flock_id, rot):
    agent = Sheep(position = position, id = flock_id, cfg = cfg, rotation = rot)
    flock.add(agent)
    return flock_id + 1
#end function

def add_dog(pack, position, cfg, pack_id, rot):
    agent = Dog(position = position, id = pack_id, cfg = cfg, rotation = rot)
    pack.add(agent)
    return pack_id + 1
#end function

def main():
    with open(config_file_name) as json_file:
        cfg = json.load(json_file)

    global screen

    dogStartId = cfg['dog_id_start']
    sheepStartId = cfg['sheep_id_start']

    pygame.init()

    screen = pygame.display.set_mode([cfg['screen_width'],cfg['screen_width']])
    pack = pygame.sprite.Group()
    flock = pygame.sprite.Group()
    protoInputHandler = ProtoInputHandler(dogStartId,sheepStartId)
    pack_id = dogStartId
    flock_id = sheepStartId
    standardRotation = 0
    for i in range(0, cfg['no_of_sheep']):
        flock_id = add_sheep(flock, np.array([random.uniform(cfg['screen_width'] / 2 - 200, cfg['screen_width'] / 2 + 200), random.uniform(cfg['screen_height'] / 2 - 200, cfg['screen_height'] / 2 + 200)]), cfg, flock_id, standardRotation)

    while True:
        for event in pygame.event.get():
            if event.type==QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if (event.button == 1 and len(pack) < cfg['max_number_of_dogs']):
                    pack_id = add_dog(pack, np.array([event.pos[0], event.pos[1]]), cfg, pack_id, standardRotation)
                elif event.button == 3:
                    if (len(pack) > 0):
                        closest_dog = None
                        for dog in pack:
                            if closest_dog == None:
                                closest_dog = dog
                            else:
                                if (np.linalg.norm(event.pos - dog.position) < np.linalg.norm(event.pos - closest_dog.position)):
                                    closest_dog = dog
                        pack.remove(closest_dog)


        screen.fill(colours.GREY)

        if (len(pack) > 0):
            calc_voronoi_partitioning(flock, pack)
            pack.update(screen, flock, pack, np.array([700, 700]), cfg)
        else:
            for sheep in flock:
                sheep.closest_dog = None

        flock.update(screen, flock, pack, cfg)

        pygame.display.flip()
        #pygame.display.update()

        
        protoInputHandler.LoadAgentTransforms(pack, flock)

        protoInputHandler.RandomiseAgentTransforms(0)

        dogTransforms = protoInputHandler.GetAgentTransforms()

        for transform in dogTransforms:
            # transform defined by ID, position(x,y), rotation
            id = transform[0]
            position = transform[1]
            x = position[0]
            y = position[1]
            rotation = transform[2]

            # get dog with relevant ID
            for dog in pack:
                if(dog.id == id):
                    dog.position[0] = x
                    dog.position[1] = y


        pygame.time.wait(10)
#end function

main()