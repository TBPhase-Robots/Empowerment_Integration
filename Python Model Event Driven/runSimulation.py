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
import math

from datetime import datetime

import time
import os
from ProtoInputHandler import ProtoInputHandler


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def ControllerCallback(data):
    print("topic contents:")
    print(data)

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

def add_sheep(flock, agents, position, cfg, flock_id):
    agent = Sheep(position = position, id = flock_id, cfg = cfg, rotation=0, callback=ControllerCallback)
    flock.add(agent)
    agents.add(agent)
    return flock_id + 1
#end function

def add_dog(pack, agents, position, cfg, pack_id):
    agent = Dog(position = position, id = pack_id, cfg = cfg, rotation=0, callback=ControllerCallback)
    pack.add(agent)
    agents.add(agent)

    return pack_id + 1
#end function



def DrawWorld(cfg):
    #protoInputHandler = ProtoInputHandler(0, 99)

    screen.fill(colours.DGREY)
    pygame.draw.rect(screen, colours.GREY, pygame.Rect(0, 0, cfg['world_width'], cfg['world_height']))

    # Draw target box
    pygame.draw.rect(screen, colours.RED, pygame.Rect(cfg['target_position'][0] - 100, cfg['target_position'][1] - 100, 200, 200), 3)



    pygame.display.flip()
    #pygame.display.update()

# calls standard behaviour on all sheep and dog agents for simulation
def ExperimentUpdateTimestep(pack, flock, cfg):

    if (len(pack) > 0):
        calc_voronoi_partitioning(flock, pack)
        pack.SimulationUpdate(screen, flock, pack, cfg)
    else:
        for sheep in flock:
            sheep.closest_dog = None

    flock.SimulationUpdate(screen, flock, pack, cfg)                  

       

def main(config_name='defaultConfig', show_empowerment=False):

    rclpy.init(args=None)

    with open(f"experiment_config_files/{config_name}.json") as json_file:
        cfg = json.load(json_file)

    if ('show_empowerment' not in cfg):
        cfg['show_empowerment'] = show_empowerment

    global screen
    

    end_game = False

    pygame.init()

    screen = pygame.display.set_mode([cfg['world_width'] + 80,cfg['world_height']])
    pack = pygame.sprite.Group()
    flock = pygame.sprite.Group()
    agents = pygame.sprite.Group()
    pack_id = 0
    flock_id = 99

    for pos in cfg['initial_dog_positions']:
        pack_id = add_dog(pack, agents, np.array(pos), cfg, pack_id)

    for pos in cfg['initial_sheep_positions']:
        flock_id = add_sheep(flock, agents, np.array(pos), cfg, flock_id)

    
    state = "park"
    
    
    while (not end_game):

        # polls ROS topics for dogs, sheep, unnassigned for poses
        for dog in pack:
            dog.RosUpdate()

        for sheep in flock:
            sheep.RosUpdate()

        # draw world 
        DrawWorld(cfg=cfg)

        # send all agents to park
        if(state == "park"):
            ExperimentUpdateTimestep(pack = pack, flock=flock, cfg=cfg)

        if(state == "experiment"):
            ExperimentUpdateTimestep(pack = pack, flock=flock, cfg=cfg)
    

        

        # legacy add/remove dog handler
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                if (event.button == 1 and len(pack) < cfg['max_number_of_dogs']):
                
                    pack_id = add_dog(pack, np.array([event.pos[0], event.pos[1]]), cfg, pack_id)
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

        
        time.sleep(0.01)

        







#end function

if __name__ == '__main__':
    main()
        



