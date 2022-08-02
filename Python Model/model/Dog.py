import pygame
import numpy as np
import colours
import sys
from model.Agent import Agent
import random
import math
from math import degrees, atan2
from model.Sheep import Sheep

class Dog(Agent):

    def __init__(self, position, id, cfg) -> None:
        super().__init__(position, id, cfg)
        self.sub_flock = pygame.sprite.Group()
        self.direction = np.array([1, 0])
        self.choice_tick_count = 0
        self.target_sheep = None
        self.driving_point = np.zeros(2)
        self.state = 'collecting'
        self.steering_point = np.zeros(2)
        self.empowerment = 0
        self.rotation = 0
    #end function 

    # a and b must be np arrays, returns an angle between two vectors.
    def CalcAngleBetweenVectors(self, a, b):
        # NORMALISE A AND B
        a = a / np.linalg.norm(a)
        b = b / np.linalg.norm(b)
        dot = np.dot(a, b)
        if (dot > 1):
            dot = 1
        theta = np.arccos(dot)
        if ((np.cross([a[0], a[1], 0], [b[0], b[1], 0])[2] > 0)   ):
            theta = - theta
        return math.degrees(theta)
    
    def CalcBearing(x, y, center_x, center_y):
        angle = degrees(atan2(y - center_y, x - center_x))
        bearing1 = (angle + 360) % 360
        bearing2 = (90 - angle) % 360
        return bearing1, bearing2
    #end function 

    # The main update loop called each frame, contains the behaviour of the agent.
    def update(self, screen, flock, pack, cfg):
        target = cfg['target_position']

        # If the dog is assigned to manage any sheep, exhibit either driving or collecting behaviour.
        # When the flock all withing a set radius, the dog moves to a driving position behind the flock, opposite to the target position.
        # Otherwise, the dog moves to collect the sheep within its subflock which is furthest from the centre of the whole flock.
        if (len(self.sub_flock) > 0):
            # Calculate the centre of mass of the flock.
            sheep_positions = []
            for sheep in flock:
                sheep_positions.append(sheep.position)
            C = Agent.calcCoM(self, sheep_positions)
            furthest_sheep_position = C

            # Only change target every n ticks.
            # Calculate a driving point behind the flock, opposite the target.
            # Also calcluate the sheep which is furthest from the centre of mass of the flock.
            if (self.choice_tick_count == 0):
                self.driving_point = np.add(C, cfg['driving_distance_from_flock_radius'] * (C - target) / np.linalg.norm(C - target))
                for sheep in self.sub_flock:
                    if (np.linalg.norm(sheep.position - C) > np.linalg.norm(furthest_sheep_position - C)):
                        furthest_sheep_position = sheep.position
                        self.target_sheep = sheep

            # If the dog is not assigned to any sheep, use the centre of mass as a default value.
            try:
                furthest_sheep_position = self.target_sheep.position
            except:
                furthest_sheep_position = C

            # Only change state every n ticks
            if (self.choice_tick_count == 0):
                if (np.linalg.norm(furthest_sheep_position - C) < cfg['collection_radius']):
                    self.state = 'driving'
                else:
                    self.state = 'collecting'

            # If driving, set the steering point to the driving point.
            if (self.state == 'driving'):
                self.steering_point = self.driving_point
            # If collecting, set the steering point to be slightly behind the target sheep.
            elif (self.state == 'collecting'):
                self.steering_point = np.add(furthest_sheep_position, cfg['collection_distance_from_target_sheep'] * (furthest_sheep_position - C) / np.linalg.norm(furthest_sheep_position - C))

        # When the dog is not assigned to any sheep, moves to a driving point slightly behind the outer radius of the flock, opposite to the target position.
        else:
            self.state = 'unassigned'
            # Calculate the centre of mass of the flock.
            sheep_positions = []
            for sheep in flock:
                sheep_positions.append(sheep.position)
            C = Agent.calcCoM(self, sheep_positions)
            furthest_sheep_position = C
            
            # Only change target every n ticks.
            if (self.choice_tick_count == 0):
                for sheep in flock:
                    if (np.linalg.norm(sheep.position - C) > np.linalg.norm(furthest_sheep_position - C)):
                        furthest_sheep_position = sheep.position
            
            # Set the steering point to be a driving position along the outer radius of the flock, opposite to the target and further back than a regular driving point would be.
            outer_flock_radius_point = np.add(C, np.linalg.norm(C - furthest_sheep_position) * ((C - target) / np.linalg.norm(C - target)))
            self.steering_point = np.add(outer_flock_radius_point, cfg['driving_distance_from_flock_radius'] * ((C - target) / np.linalg.norm(C - target)))

        # Forces relating to the flock.
        F_H = self.calc_F_H(screen, cfg, self.steering_point, flock)
        # Repulsion from dogs.
        F_D = self.calc_F_D(pack, cfg)
        
        # Resultant force vector from combining previous forces.
        F = (cfg['dog_forces_with_flock'] * F_H) + (cfg['dog_repulsion_from_dogs'] * F_D)

        # Debug draw forces
        if (cfg['debug_dog_forces']):
            pygame.draw.line(screen, colours.ORANGE, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_dogs'] * F_D), 8)
        # Debug draw steering point
        if (cfg['debug_steering_points']):
            pygame.draw.circle(screen, colours.BLACK, self.steering_point, 4)

        # calculate forward vector
        forwardX = math.sin(self.rotation)
        forwardY = math.cos(self.rotation)

        # Show movement markers if the debug option is enabled.
        # Otherwise, show a small direction line on the agent.
        if(cfg['realistic_agent_movement_markers']):
            # black line is target rotation
            pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, np.array(F)*10) ,8)
            # draw line in forward vector
            pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*80) ,5)
        else:
            pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, np.array([forwardX, -forwardY])*10) ,2)

        # calculate angle between current dir and target dir:
        angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), np.array(F))

        # Unrealistic movement
        if(not cfg['realistic_agent_movement']):
            self.position = np.add(self.position, F)
        # Realistic movement
        else:
            # differential drive rotation towards target direction

            # rotate until forward vector is parallel to force within reason

            # if vector is parallel, then go forward
            turnRate = cfg['realistic_dog_turn_rate']
            if(angle > 5):
                self.rotation -= turnRate

                deltaV = [2*forwardX, -2*forwardY]
                dog_max_speed = cfg['dog_max_speed']
                if(np.linalg.norm(deltaV) > dog_max_speed):
                    deltaV = deltaV * (dog_max_speed / np.linalg.norm(deltaV))

                newPos = np.add(self.position, deltaV)     
                self.position = newPos
            elif(angle < -5):
                self.rotation += turnRate
                deltaV = [2*forwardX, -2*forwardY]
                dog_max_speed = cfg['dog_max_speed']
                if(np.linalg.norm(deltaV) > dog_max_speed):
                    deltaV = deltaV * (dog_max_speed / np.linalg.norm(deltaV))

                newPos = np.add(self.position, deltaV)     
                self.position = newPos

            else:
                deltaV = F
                dog_max_speed = cfg['dog_max_speed']

                if(np.linalg.norm(deltaV) > dog_max_speed):
                    deltaV = deltaV * (dog_max_speed / np.linalg.norm(deltaV))
                
                self.position = np.add(self.position, deltaV)

        # Increment ticks, reset if limit reached.
        self.choice_tick_count += 1
        if (self.choice_tick_count >= cfg['ticks_per_choice']):
            self.choice_tick_count = 0

        # Very rudimentary collison detection to ensure dogs don't occupy the same space.
        collision_check = True
        while (collision_check):
            collision_check = False
            for dog in pack:
                if (dog.id != self.id):
                    if (np.linalg.norm(self.position - dog.position) <= 8):
                        self.position = np.add(self.position, self.position - dog.position)
                        collision_check = True
        
        # Ensure dogs don't move outside of the play area.
        if (self.position[0] > cfg['world_width'] - 10): self.position[0] = cfg['world_width'] - 10
        elif (self.position[0] < 10): self.position[0] = 10
        if (self.position[1] > cfg['world_height'] - 10): self.position[1] = cfg['world_height'] - 10
        elif (self.position[1] < 10): self.position[1] = 10

        # Show empowerment based on type selection.
        if (cfg['empowerment_type'] == 0):
            # Empowerment is equal the the size of the subflock allocated.
            self.empowerment = len(self.sub_flock)
        elif (cfg['empowerment_type'] == 1):
            # Empowerment scales based on how many sheep are within 50 units of the dog. The closer the sheeom the higher its contribution to the empowerment score.
            if (len(self.sub_flock) > 0):
                self.empowerment = 5
            else:
                self.empowerment = 0
            for sheep in flock:
                if (np.linalg.norm(self.position - sheep.position) <= 50):
                    self.empowerment += 5 - math.floor(np.linalg.norm(self.position - sheep.position) / 10)

        # Agent superclass update. (Currently just draws the black border around the agent)
        super().update(screen)

        # Debug draw states, or draw a blue infill.
        if (cfg['debug_dog_states']):
            if (self.state == 'driving'):
                pygame.draw.circle(screen, colours.DRIVE, self.position, 5)
            elif (self.state == 'collecting'):
                pygame.draw.circle(screen, colours.COLLECT, self.position, 5)
            else:
                pygame.draw.circle(screen, colours.BLUE, self.position, 5)

        # Show empowerment values if enabled in config.
        else:
            if (cfg['show_empowerment']):
                # Empowerment scaled from values 0 to 20, shifting from red...
                if (self.empowerment < 5):
                    colour = np.array([155 + round(100 * self.empowerment / 5), 0, 0])
                # ...to orange...
                elif (self.empowerment < 10):
                    colour = np.array([255, round(255 * (self.empowerment - 5) / 5), 0])
                # ...to yellow...
                elif (self.empowerment < 15):
                    colour = np.array([255 - round(255 * (self.empowerment - 10) / 5), 255, 0])
                # ...to green...
                elif (self.empowerment < 20):
                    colour = np.array([0, 255 - round(100 * (self.empowerment - 15) / 5), 0])
                else:
                    colour = np.array([0, 155, 0])
                pygame.draw.circle(screen, colour, self.position, 5)
            else:
                pygame.draw.circle(screen, colours.BLUE, self.position, 5)

        # Debug draw sublock.
        if (cfg['debug_sub_flocks']):
            if (self.id < 5):
                pygame.draw.circle(screen, colours.SRANGE[self.id], self.position, 4)
            else:
                pygame.draw.circle(screen, colours.BLACK, self.position, 4)
    #end function

    # Clear the subflock.
    def empty_sub_flock(self):
        self.sub_flock.empty()
    #end function

    # Add a sheep to the subflock.
    def add_sheep_to_sub_flock(self, sheep):
        self.sub_flock.add(sheep)
    #end function

    # Calculate the repulsion from other dogs.
    def calc_F_D(self, pack, cfg):
        moveRepelDistance = cfg['dog_dog_repulsion_range']
        F_D_D = np.zeros(2)
        for dog in pack:
            if (dog.id != self.id):
                if (np.linalg.norm(self.position - dog.position) < moveRepelDistance):
                    F_D_D = np.add(F_D_D, (self.position - dog.position) / np.linalg.norm(self.position - dog.position))

        F_D = F_D_D + (0.75 * np.array([F_D_D[1], -F_D_D[0]]))
        return F_D
    #end function

    # Return a value based on a given angle. (See paper for details: https://link.springer.com/chapter/10.1007/978-3-030-89177-0_15)
    def sine_step(self, theta):
        if ((-math.pi < theta and -math.pi / 2 >= theta) or (math.pi < theta and 3 * math.pi / 2 >= theta)):
            return 1
        elif ((-3 * math.pi / 2 < theta and -math.pi >= theta) or (math.pi / 2 < theta and math.pi >= theta)):
            return -1
        else:
            return -math.sin(theta)
    #end function

    # Calculate the forces relating to the flock.
    # See the paper for a full explanation (https://link.springer.com/chapter/10.1007/978-3-030-89177-0_15).
    def calc_F_H(self, screen, cfg, steering_point, flock):
        sheep_positions = []
        if (len(self.sub_flock) > 0):
            for sheep in self.sub_flock:
                sheep_positions.append(sheep.position)
        else:
            for sheep in flock:
                sheep_positions.append(sheep.position)
        C = Agent.calcCoM(self, sheep_positions)
        W = steering_point

        R_C_D = (self.position - C) / np.linalg.norm(self.position - C)
        R_C_W = (W - C) / np.linalg.norm(W - C)

        dot = np.dot(R_C_D, R_C_W)
        if (dot > 1):
            dot = 1
        theta_D_C_W = np.arccos(dot)
        if (np.cross([R_C_D[0], R_C_D[1], 0], [R_C_W[0], R_C_W[1], 0])[2] < 0):
            theta_D_C_W = - theta_D_C_W

        R_D_W = (W - self.position) / np.linalg.norm(W - self.position)
        R_D_T = np.array([R_C_D[1], -R_C_D[0]]) 

        H_F = 1 - math.exp(-2 * abs(math.degrees(theta_D_C_W)))
        H_T = self.sine_step(theta_D_C_W)

        sum = np.zeros(2)
        for sheep in flock:
            sum = np.add(sum, (self.position - sheep.position) / (2 *np.linalg.norm(self.position - sheep.position)))

        F_F = H_F * sum

        # Cap dog_max_repulsion_from_sheep, not part of the model explained in the paper.
        repulsionFromSheep = cfg['dog_repulsion_from_sheep'] * F_F
        if(np.linalg.norm(repulsionFromSheep) > cfg['dog_max_repulsion_from_sheep']):
            vectorMagnitude = np.linalg.norm(repulsionFromSheep)
            ratio = cfg['dog_max_repulsion_from_sheep'] / vectorMagnitude
            repulsionFromSheep = repulsionFromSheep * ratio
        

        F_W = R_D_W
        F_T = H_T * R_D_T

        if (cfg['debug_dog_forces']):
            pygame.draw.line(screen, colours.GREEN, self.position, np.add(self.position, 10 * cfg['dog_repulsion_from_sheep'] * F_F), 8)
            pygame.draw.line(screen, colours.RED, self.position, np.add(self.position, 10 * cfg['dog_attraction_to_steering_point'] * F_W), 8)
            pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, 10 * cfg['dog_orbital_around_flock'] * F_T), 8)
        F_H = repulsionFromSheep + (cfg['dog_attraction_to_steering_point'] * F_W) + (cfg['dog_orbital_around_flock'] * F_T)

        return F_H
    #end function