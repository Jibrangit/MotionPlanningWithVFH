# -*- coding: utf-8 -*-
"""
Created on Thu Oct  7 05:33:40 2021

@author: justi
"""

from vfh import *
from a_star_searching_from_two_side import *
import csv
import numpy as np
import matplotlib.pyplot as plt

def bestpath():
    
    hg_dim = (51, 51)
    '''
    current = start
    target_angle = None
    current_angle = wrap_angle(math.degrees(
        math.atan2(start[0] - end[0], start[1] - end[1])))
    previous_angle = current_angle
    '''
    def from_map(map_fname):
        """ Create grid from text file """
        with open(map_fname, 'r') as f:
            reader = csv.reader(f, delimiter=" ")
            lines = list(reader)
    
        lines = list(map(lambda l: list(map(int, l)), lines))
        return lines
    
    hg = HistogramGrid(hg_dim[0], hg_dim[1])
    hg.grid = from_map("map_no_sides.txt")
    
    top_vertex = [len(hg.grid)-1, len(hg.grid[0])-1]  # top right vertex of boundary
    bottom_vertex = [0, 0]  # bottom left vertex of boundary
    
    start = [bottom_vertex[0]+1,bottom_vertex[1]+1]
    goal = [top_vertex[0]-1,top_vertex[1]-1]
    
    ay = list(range(bottom_vertex[1], top_vertex[1]))
    ax = [bottom_vertex[0]] * len(ay)
    cy = ay
    cx = [top_vertex[0]] * len(cy)
    bx = list(range(bottom_vertex[0] + 1, top_vertex[0]))
    by = [bottom_vertex[1]] * len(bx)
    dx = [bottom_vertex[0]] + bx + [top_vertex[0]]
    dy = [top_vertex[1]] * len(dx)
    '''
    # generate random obstacles
    ob_x = np.random.randint(bottom_vertex[0] + 1,
                             top_vertex[0], obs_number).tolist()
    ob_y = np.random.randint(bottom_vertex[1] + 1,
                             top_vertex[1], obs_number).tolist()
    '''
    # x y coordinate in certain order for boundary
    x = ax + bx + cx + dx
    y = ay + by + cy + dy
    
    
    i = 1
    ob_x = []
    ob_y = []
    while i < len(hg.grid)-1:
        j = 1
        while j < len(hg.grid[0])-1:
            if hg.grid[i][j] == 1:
                ob_x.append(j)
                ob_y.append(i)
            j += 1
        i += 1
            
    
    obstacle = np.vstack((ob_x, ob_y)).T.tolist()
    # remove start and goal coordinate in obstacle list
    obstacle = [coor for coor in obstacle if coor != start and coor != goal]
    obs_array = np.array(obstacle)
    bound_temp = np.vstack((x, y)).T
    bound = np.vstack((bound_temp, obs_array))
    
    path = searching_control(start, goal, bound, obstacle)
    if not show_animation:
        print(path)
    #%%   
    plt.cla()
    plt.gcf().set_size_inches(11, 9, forward=True)
    plt.axis('equal')
    plt.plot(path[:, 0], path[:, 1], 'or')
    plt.plot(bound[:, 0], bound[:, 1], 'sk')
    plt.plot(goal[0], goal[1], '*b', label='Goal')
    plt.plot(start[0], start[1], '^b', label='Origin')
    plt.legend()
    plt.pause(0.0001)
    return path