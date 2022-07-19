import pygame
import colours
import sys
from pygame.locals import *
import numpy as np
from model.Agent import Agent
import json
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import math

from datetime import datetime

import time
import os
from ProtoInputHandler import ProtoInputHandler
from model.Listener import Listener
from model.CommandListener import CommandListener
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def ControllerCallback(data):
    print("topic contents:")
    print(data)

def CommandListenerCallback(data):
    print("CommandListenerData:")
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
    print("aaa")
#end function

def add_dog(pack, agents, position, cfg, pack_id):
    print("aaa")

#end function

def add_agent(agents, position, cfg, id):
    agent = Agent(position = position, id = id, cfg = cfg, rotation=0, poseAgentCallback=ControllerCallback, role = "agent")
    agents.add(agent)
    return id + 1



def DrawWorld(cfg):
    screen.fill(colours.DGREY)
    pygame.draw.rect(screen, colours.GREY, pygame.Rect(0, 0, cfg['world_width'], cfg['world_height']))
    # Draw target box
    pygame.draw.rect(screen, colours.RED, pygame.Rect(cfg['target_position'][0] - 100, cfg['target_position'][1] - 100, 200, 200), 3)
    pygame.display.flip()


# calls standard behaviour on all sheep and dog agents for simulation
def ExperimentUpdateTimestep(agents, pack, flock, cfg):
    if (len(pack) > 0):
        calc_voronoi_partitioning(flock, pack)
        for dog in pack:
            dog.SimulationUpdate_Dog(screen, flock, pack, cfg)
        
    else:
        for sheep in flock:
            sheep.closest_dog = None

    if(len(flock) > 0):
        for sheep in flock:

            sheep.SimulationUpdate_Sheep(screen, flock, pack, cfg)                  


def StandbySetupUpdateTimestep(agents, cfg):
    # make all agents go to top



    point_x = 30 

    point_y = 60
    i = 0
    for agent in agents:
        i +=1 
        point_x = 30 + i * 15
        agent.MoveToPoint(point_x = point_x, point_y = point_y, screen = screen, agents = agents, cfg = cfg)

    

def SortAgentsByRole():
    print("sort agents by role")

    pack.empty()
    flock.empty()
    pigs.empty()
    standby.empty()
    
    for agent in agents:
        if(agent.role == "dog"):
            pack.add(agent)
        if(agent.role == "sheep"):
            flock.add(agent)
        if(agent.role == "pig"):
            pigs.add(agent)
        if(agent.role == "standby"):
            standby.add(agent)

pack = pygame.sprite.Group()
flock = pygame.sprite.Group()
pigs = pygame.sprite.Group()
standby = pygame.sprite.Group()
agents = pygame.sprite.Group()

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

    agent_id = 0


    commandListenerTopicName = "/controller/command"
    # define the state command listener:
    commandListener = CommandListener(commandListenerTopicName, CommandListenerCallback) 

    ## add n agents
    for i in range (2):
        # adds agents with role agent (un initialised)
        agent_id = add_agent(agents = agents, position = np.array([30 + (i*15), 20]), cfg = cfg, id = agent_id)
    
    # put all robots into standby.
    for agent in agents:
        agent.role = "standby"

    SortAgentsByRole()

    


    
    state = "setup"
    
    
    while (not end_game):

        pygame.display.update()
        time.sleep(0.01)

        # poll for poses for each agent
        for agent in agents:
            agent.RosUpdate()
        

        # look out for commands send to this script
        rclpy.spin_once(commandListener, timeout_sec=0.01)


        # draw world 
        DrawWorld(cfg=cfg)


        if(state == "setup"):
            

            print("setup")
            print("agents", agents)
            print("pack",pack)
            print("flock",flock)
            print("pigs",pigs)
            print("standby",standby)
            StandbySetupUpdateTimestep(agents = agents, cfg=cfg)

        elif(state == "experiment"):
            ExperimentUpdateTimestep(agents = agents, pack = pack, flock=flock, cfg=cfg)

            
    

        

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

        

        







#end function

if __name__ == '__main__':
    main()
        



