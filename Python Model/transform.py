import pygame
import numpy as np
import colours
import sys
from model.Agent import Agent
import random
import math
from model.Dog import Dog
from model.Sheep import Sheep

class Transform:

    position_x = 0
    position_y = 0

    rotation = 0
    
    
    def __init__(self, x, y, rotation):
        print("Transform _init_" , x, y, rotation)
