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
from model.SimLog import Logger
from datetime import datetime
import VideoRecorder
import time
import os

RECORD_VIDEO = True
#INITIAL_PAUSE_TIME = 200

log = Logger()
ticks = 0
log_path = os.path.join("Empowerment Results")

# Dogs have a subflock which references the sheep assigned to them in their voronoi partition.
# First, clear the previous subflock.
# Then, assign each sheep to the subflock of the dog closest dog.
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

# Creates a sheep agent at a given position, and assigns it to the flock.
# Returns an incremented id so that each sheep has a unique id.
def add_sheep(flock, position, cfg, flock_id):
    agent = Sheep(position = position, id = flock_id, cfg = cfg)
    flock.add(agent)
    log.addNewAgentInLog('sheep', flock_id, position, ticks)
    return flock_id + 1
#end function

# Creates a dog agent at a given position, and assigns it to the pack.
# Returns an incremented id so that each sheep has a unique id.
def add_dog(pack, position, cfg, pack_id):
    agent = Dog(position = position, id = pack_id, cfg = cfg)
    pack.add(agent)
    log.addNewAgentInLog('dog', pack_id, position, ticks)
    return pack_id + 1
#end function

# Runs one instance of an experiment. Requires inputs of the config file to use, whether or not to show empowerment, the id to be used in data logging, and the name of the file to log to. 
# Task-weighted empowerment is not implemented, so this value makes no difference to the experiement.
def main(config_name='config_exp_5', show_empowerment=False, use_task_weighted_empowerment=False, sim_session_id='000000T000000', log_file_name=''):
    # Get the operating system which is currently being used. This is needed to have the correct file path delimeters when logging and recording video.
    myOs = ""
    if (sys.platform.startswith("linux") or sys.platform.startswith("linux2")):
        # linux
        print("running on linux a")
        myOs = "linux"
    elif (sys.platform.startswith("darwin")):
        # OS X
        print("running on mac a")
        myOs = "mac"
    elif (sys.platform.startswith("win32")):
        # Windows...
        print("running on windows a")
        myOs = "win"
    
    # Load the config file
    fname = ""
    if(myOs == "win"):
        fname = f"experiment_config_files\\{config_name}.json"
    elif(myOs == "linux" or myOs == "mac"):
        fname = f"experiment_config_files/{config_name}.json"
    print(os.path.abspath(os.getcwd()))
    with open(fname) as json_file:
        cfg = json.load(json_file)

    # If the config file specifies whether or not to show empowerment, then it overrides the passed in argument.
    if ('show_empowerment' not in cfg):
        cfg['show_empowerment'] = show_empowerment

    # Initialise variables for use in the simulation.
    global screen
    log.initialise(sim_session_id, config_name, show_empowerment, use_task_weighted_empowerment)
    ticks = 0
    end_game = False
    pygame.init()
    screen = pygame.display.set_mode([cfg['world_width'] + 80,cfg['world_height']])
    pack = pygame.sprite.Group()
    flock = pygame.sprite.Group()
    pack_id = 0
    flock_id = 99

    # Add any dogs to the scene that are present in the config file.
    for pos in cfg['initial_dog_positions']:
        pack_id = add_dog(pack, np.array(pos), cfg, pack_id)

    # Add any sheep to the scene that are present in the config file.
    for pos in cfg['initial_sheep_positions']:
        flock_id = add_sheep(flock, np.array(pos), cfg, flock_id)

    # Log an initial state
    log.logPopulationStates('dog', pack, ticks)
    log.logPopulationStates('sheep', flock, ticks)
    log.logPopulations([pack, flock], ticks)

    # Start the video recorder if necessary.
    if RECORD_VIDEO:
        (screen_width,screen_height)= screen.get_size()
        resolution = (screen_width, screen_height)
        filename = ""
        if(myOs == "win"):
            filename = f"{log_path}\\{sim_session_id}\\{config_name}_recording.avi"
        elif(myOs == "linux" or myOs == "mac"):
            filename = f"{log_path}/{sim_session_id}/{config_name}_recording.avi"
        video = VideoRecorder.VideoRecorder()
        fps = 30
        video.setConfig(filename, fps, resolution)
        video.filename = filename
        video.startRecorder()

    # The main simulation loop, runs until the game ends or the time limit is reached.
    while (ticks < cfg['time_limit'] and not end_game):
        # Record the start time
        start_time = round(time.time() * 1000)
        if (ticks == 0):
            log.recordStartTime(datetime.now())
        
        # Handle events
        for event in pygame.event.get():
            # Allows the window to close with the 'x' button.
            if event.type==QUIT:
                # Saves the video early if the window is closed.
                if RECORD_VIDEO:
                    video.stopRecorder()
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # On leftclick, add a dog to the play area. (Only if the max number of dogs has not been reached.)
                if (event.button == 1 and len(pack) < cfg['max_number_of_dogs']):
                    log.user_log.addMouseClick(ticks, datetime.now(), "MB:DOWN:LEFT", event.pos)
                    pack_id = add_dog(pack, np.array([event.pos[0], event.pos[1]]), cfg, pack_id)
                # On rightclick, remove the nearest dog to the cursor from the play area.
                elif event.button == 3:
                    log.user_log.addMouseClick(ticks, datetime.now(), "MB:DOWN:RIGHT", event.pos)
                    if (len(pack) > 0):
                        closest_dog = None
                        for dog in pack:
                            if closest_dog == None:
                                closest_dog = dog
                            else:
                                if (np.linalg.norm(event.pos - dog.position) < np.linalg.norm(event.pos - closest_dog.position)):
                                    closest_dog = dog
                        log.destroyAgentInLog('dog', closest_dog.id, closest_dog.position, plt.tick_params)
                        pack.remove(closest_dog)

        # Draw background and play area.
        screen.fill(colours.DGREY)
        pygame.draw.rect(screen, colours.GREY, pygame.Rect(0, 0, cfg['world_width'], cfg['world_height']))

        # draw soft play area (if you want to, no pressure)
        
     #   pygame.draw.rect(screen, colours.BLUE, pygame.Rect(cfg['play_area_x'], cfg['play_area_y'], cfg['play_area_width'], cfg['play_area_height']), 3)




        # Draw dogs in pen
        for i in range(0, cfg['max_number_of_dogs'] - len(pack)):
            pygame.draw.circle(screen, colours.BLACK, [cfg['world_width'] + 20 + ((i % 3) * 20), 20 + (math.floor(i / 3) * 20)], 7)
            pygame.draw.circle(screen, colours.BLUE, [cfg['world_width'] + 20 + ((i % 3) * 20), 20 + (math.floor(i / 3) * 20)], 5)

        # Draw target box
        pygame.draw.rect(screen, colours.RED, pygame.Rect(cfg['target_position'][0] - 100, cfg['target_position'][1] - 100, 200, 200), 3)

        # Call 'update' on every dog, performing the dogs behaviours.
        if (len(pack) > 0):
            # Partitions the sheep between the dogs
            calc_voronoi_partitioning(flock, pack)
            # Dogs run their behaviour.
            pack.update(screen, flock, pack, cfg)
        else:
            for sheep in flock:
                sheep.closest_dog = None

        # Sheep run their behaviour.
        flock.update(screen, flock, pack, cfg)

        # Draw the new state of the scene to the screen
        pygame.display.flip()

        # Increment time and ticks counter.
        current_time = round(time.time() * 1000)
        if (current_time - start_time < 10):
            pygame.time.wait(10 - (current_time - start_time))
        ticks += 1

        # Log the state of the scene
        log.logPopulationStates('dog', pack, ticks)
        log.logPopulationStates('sheep', flock, ticks)
        log.logPopulations([pack, flock], ticks)

        # Record a frame of video if necessary.
        if RECORD_VIDEO:
            # Screenshot the current pygame screen
            img = pygame.surfarray.array3d(screen)
            # Convert the screenshot to a numpy array
            frame = np.array(img)
            frame = np.fliplr(frame)
            frame = np.rot90(frame)
            video.grabScreen(frame)
        
        # Check if the sheep have all reached the target area. If they have, end the game.
        # There is a slight buffer around the inside of the target area. This helps make the end of the simulation be less abrupt. 
        inner_buffer = 10
        left_bound = cfg['target_position'][0] - 100 + inner_buffer
        right_bound = cfg['target_position'][0] + 100 - inner_buffer
        top_bound = cfg['target_position'][1] - 100 + inner_buffer
        bottom_bound = cfg['target_position'][1] + 100 - inner_buffer
        sheep_within_target = 0
        for sheep in flock:
            if (sheep.position[0] > left_bound and sheep.position[0] < right_bound and sheep.position[1] > top_bound and sheep.position[1] < bottom_bound):
                sheep_within_target += 1
        
        if (len(flock) == sheep_within_target):
            end_game = True
    # End of simulation loop

    log.recordEndTime(datetime.now())
    
    # Stop the video recording, thus saving the video file.
    if RECORD_VIDEO:
       video.stopRecorder()
    
    # Create a meaningful name for the log file if one isn't provided
    # NOTE: This code is identical to a block in run_simulation() in main.py
    if not log_file_name:
        log_name = config_name
        if show_empowerment:           
            log_name = log_name + "_empshown"       
        if use_task_weighted_empowerment:
            log_name = log_name + "_taskweighted"
    else:
        log_name = log_file_name
    
    # Save the log to disk
    log.pickleLog(os.path.join(log_path, sim_session_id, "{}_simlog".format(log_name)))
#end function

if __name__ == '__main__':
    main()
