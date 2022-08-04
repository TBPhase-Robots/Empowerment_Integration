from cv2 import magnitude
import pygame
import numpy as np
import colours
from model.Agent import Agent
import math
import random
from math import degrees, atan2

class Sheep(Agent):

    def __init__(self, position, id, cfg) -> None:
        super().__init__(position, id, cfg)
        self.closest_dog = None
        self.grazing = True
        self.grazing_direction = np.array([1, 0])
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

        forwardX = math.sin(self.rotation)
        forwardY = math.cos(self.rotation)

        # If a dog is not within the sheep's vision range, the sheep performs grazing behaviour.
        # 5% of the time, the sheep will change the direction it moves when grazing.
        if (self.closest_dog != None):
            if (np.linalg.norm(self.position - self.closest_dog.position) <= cfg['sheep_vision_range']):
                self.grazing = False
            else:
                if (random.random() < 0.05):
                    self.grazing_direction = np.array([random.uniform(-3, 3), random.uniform(-3, 3)])
                self.grazing = True
        else:
            self.grazing = True
            if (random.random() < 0.05):
                self.grazing_direction = np.array([random.uniform(-3, 3), random.uniform(-3, 3)])

        # Grazing behaviour.
        if (self.grazing):
            # Repulsion force from other sheep.
            if (len(flock) > 1):
                F_S = self.calc_F_S(flock, cfg)
            else:
                F_S = 0
            self.position = np.add(self.position, (cfg['sheep_repulsion_from_sheep'] * F_S))

            angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), self.grazing_direction)

            # Show movement markers if the debug option is enabled.
            # Otherwise, show a small direction line on the agent.
            if(cfg['realistic_agent_movement_markers']):
                # black line is target rotation
                pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, self.grazing_direction*5) ,8)
                # draw line in forward vector
                pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*30) ,5)
            else:
                pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, np.array([forwardX, -forwardY])*10) ,2)

            # If a randomness check passes, move a small amount in the grazing direction.
            if (random.random() < cfg['grazing_movement_chance'] ):
                if(not cfg['realistic_agent_movement']):
                    self.position = np.add(self.position, self.grazing_direction)
                else:
                    turnRate = cfg['realistic_sheep_turn_rate']
                    if(angle > 10):
                        self.rotation -= turnRate
                        self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                    elif(angle < -10):
                        self.rotation += turnRate
                        self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                    else:
                        self.position = np.add(self.position, self.grazing_direction)

        # Herding behaviour.
        else:
            # Repulsion from dogs.
            F_D = self.calc_F_D(pack, cfg)
            if (len(flock) > 1):
                # Repulsion from sheep.
                F_S = self.calc_F_S(flock, cfg)
                # Attraction to sheep.
                F_G = self.cal_F_G(flock, cfg)
            else:
                F_S = 0
                F_G = 0

            # Combine the previous forces into one force vector.
            F = (cfg['sheep_resulsion_from_dogs'] * F_D) + (cfg['sheep_repulsion_from_sheep'] * F_S) + (cfg['sheep_attraction_to_sheep'] * F_G)

            # check if outside the play zone:


            x = self.position[0]
            y = self.position[1]
            playAreaLeftBound = cfg['play_area_x']
            playAreaTopBound = cfg['play_area_y']

            playAreaRightBound = playAreaLeftBound + cfg['play_area_width']
            playAreaBottomBound = playAreaTopBound + cfg['play_area_height']
            outOfBounds = False
            boundaryForce = np.array([0.0,0.0])
            # if outside of the play area, add a force
            r = random.uniform(-1, 1)
            if(x < playAreaLeftBound):
                outOfBounds = True
                boundaryForce += np.array([2.0,r])
               # print("agent too left at position ", x, y)
            if(x > playAreaRightBound):
                outOfBounds = True
                boundaryForce += np.array([-2.0, r])
               # print("agent too right at position ", x, y)
            if(y < playAreaTopBound):
                outOfBounds = True
              #  print("agent too high at position ", x, y)
                boundaryForce += np.array([r, 2.0])
            if( y > playAreaBottomBound):
                outOfBounds = True
              #  print("agent too low at position ", x, y)
                boundaryForce += np.array([r, -2.0])

            if(outOfBounds):
                F += boundaryForce * 5

            angle = self.CalcAngleBetweenVectors(np.array([forwardX, -forwardY]), np.array(F))

            # Show movement markers if the debug option is enabled.
            # Otherwise, show a small direction line on the agent.
            if(cfg['realistic_agent_movement_markers']):
                # black line is target rotation
                pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, self.grazing_direction*8) ,8)
                # draw line in forward vector
                pygame.draw.line(screen, colours.BLUE, self.position, np.add(self.position, np.array([forwardX, -forwardY])*40) ,5)
            else:
                pygame.draw.line(screen, colours.BLACK, self.position, np.add(self.position, np.array([forwardX, -forwardY])*10) ,2)
            

            # Move along the force vector if realistic movement is disabled
            if(not cfg['realistic_agent_movement']):
                self.position = np.add(self.position, F)
            # Turn towards the desired force vector and move based on a differential drive system.
            else:
                turnRate = cfg['realistic_sheep_turn_rate']
                if(angle > 10):
                    self.rotation -= turnRate
                    self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                elif(angle < -10):
                    self.rotation += turnRate
                    self.position = np.add(self.position, [2*forwardX, -2*forwardY])
                else:
                    if(np.linalg.norm(F) > 10):
                        F /= (np.linalg.norm(F) /10)
                    self.position = np.add(self.position, np.array(F))

            # Debug draw forces
            if (cfg['debug_sheep_forces']):
                pygame.draw.line(screen, colours.ORANGE, self.position, np.add(self.position, 10 * cfg['sheep_resulsion_from_dogs'] * F_D), 8)
                pygame.draw.line(screen, colours.GREEN, self.position, np.add(self.position, 10 * cfg['sheep_repulsion_from_sheep'] * F_S), 8)
                pygame.draw.line(screen, colours.RED, self.position, np.add(self.position, 10 * cfg['sheep_attraction_to_sheep'] * F_G), 8)

        # Very rudimentary collison detection to ensure sheep don't occupy the same space.
        collision_check = True
        while (collision_check):
            collision_check = False
            for sheep in flock:
                if (sheep.id != self.id):
                    if (np.linalg.norm(self.position - sheep.position) <= 8):
                        self.position = np.add(self.position, self.position - sheep.position)
                        collision_check = True

        # Ensure sheep don't move outside of the play area.
        if (self.position[0] > cfg['world_width'] - 10): self.position[0] = cfg['world_width'] - 10
        elif (self.position[0] < 10): self.position[0] = 10
        if (self.position[1] > cfg['world_height'] - 10): self.position[1] = cfg['world_height'] - 10
        elif (self.position[1] < 10): self.position[1] = 15

        # Agent superclass update. (Currently just draws the black border around the agent)
        super().update(screen)

        # Debug draw states, or draw a white infill.
        if (cfg['debug_sheep_states']):
            if (self.grazing):
                pygame.draw.circle(screen, colours.GRAZE, self.position, 5)
            else:
                pygame.draw.circle(screen, colours.HERD, self.position, 5)
        else:
            pygame.draw.circle(screen, colours.WHITE, self.position, 5)
        
        # Debug draw subflock.
        if (cfg['debug_sub_flocks']):
            if (self.closest_dog != None):
                if (self.closest_dog.id < 5):
                    pygame.draw.circle(screen, colours.SRANGE[self.closest_dog.id], self.position, 4)
                else:
                    pygame.draw.circle(screen, colours.BLACK, self.position, 4)
    #end function

    # Sets the closest dog value.
    def set_closest_dog(self, dog):
        self.closest_dog = dog
    #end function

    # Calculates the replusion force from dogs.
    def calc_F_D(self, pack, cfg):
        sum = np.zeros(2)
        for dog in pack:
            direction = self.position - dog.position
            magnitude = np.linalg.norm(direction)
            if (magnitude != 0):
                sum += (direction / magnitude) * math.exp(- cfg['lambda_D'] * magnitude)
        return sum
    #end function

    # Calculates the repulsion force from sheep.
    def calc_F_S(self, flock, cfg):
        sum = np.zeros(2)
        for sheep in flock:
            if (sheep.id != self.id):  
                direction = self.position - sheep.position
                magnitude = np.linalg.norm(direction)
                if (magnitude != 0):
                    sum += (direction / magnitude) * math.exp(- cfg['lambda_S'] * magnitude)
        return sum
    #end function

    # Calculates the attraction force between sheep.
    # Only a subgroup of the flock is used. 
    # The config value 'lambda_G' specifies if the sheep are homophillic or heterophillic.
    def cal_F_G(self, flock, cfg):
        sheep_positions_ordered = []
        for sheep in flock:
            if (sheep.id != self.id):
                if not sheep_positions_ordered:
                    sheep_positions_ordered.append(sheep.position) #if list is empty
                else:
                    for i in range(0, len(sheep_positions_ordered)):
                        if (np.linalg.norm(sheep.position - self.position) < np.linalg.norm(self.position - sheep_positions_ordered[i])):
                            sheep_positions_ordered.insert(i, sheep.position)
                            break
                        else:
                            if (i == len(sheep_positions_ordered) - 1):
                                sheep_positions_ordered.append(sheep.position)

        social_group_positions = sheep_positions_ordered[:cfg['no_of_sheep_in_social_group']]      
        external_group_positions = sheep_positions_ordered[cfg['no_of_sheep_in_social_group']:]                            

        sheep_positons = []
        for sheep in flock:
            if (sheep.id != self.id):
                sheep_positons.append(sheep.position)

        C = Agent.calcCoM(self, sheep_positons)
        C_i = Agent.calcCoM(self, social_group_positions)
        C_i_prime = Agent.calcCoM(self, external_group_positions)
        
        C_direction = C - self.position
        C_magnitude = np.linalg.norm(C_direction)

        if (cfg['lambda_G'] > 0):
            C_i_direction = C_i - self.position
            C_i_magnitude = np.linalg.norm(C_i_direction)
            F_G = (cfg['lambda_G'] * (C_i_direction / C_i_magnitude)) + ((1 - cfg['lambda_G']) * (C_direction / C_magnitude))
        else:
            if (len(external_group_positions) > 0):
                C_i_prime_direction = C_i_prime - self.position
                C_i_prime_magnitude = np.linalg.norm(C_i_prime_direction)
                F_G = (-cfg['lambda_G'] * (C_i_prime_direction / C_i_prime_magnitude)) + ((1 + cfg['lambda_G']) * (C_direction / C_magnitude))
            else:
                F_G = 0

        return F_G
    #end function