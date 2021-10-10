from vfh import *
import sys
import math
import csv
from test import *
from vfh import *
from a_star_searching_from_two_side import *
import csv
import numpy as np
import matplotlib.pyplot as plt

def bestpath():
    global bound
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
 
    plt.cla()
    plt.gcf().set_size_inches(11, 9, forward=True)
    plt.axis('equal')
    plt.plot(path[:, 0], path[:, 1], 'or')
    plt.plot(bound[:, 0], bound[:, 1], 'sk')
    plt.plot(goal[0], goal[1], '*b', label='Goal')
    plt.plot(start[0], start[1], '^b', label='Origin')
    plt.legend()
    plt.pause(0.0001)
    return path,start,goal
#%%
""" Testing for VFH+ algorithm using map.txt grid from vfh-python"""
# USER-DEFINED VARIABLES--------
path,start,end = bestpath()
# Start/end location for robot in histogram grid
#start = random_coordinate(bottom_vertex, top_vertex)
#end = random_coordinate(bottom_vertex, top_vertex)

# Dimension of HistogramGrid
hg_dim = (50, 50)

# Number of sectors in PolarHistogram
nsectors = 72

# Window Length of active region
w_s = 11

# Max number of steps for loop (Prevent infinite loops)
MAX_STEPS = 150

# CONSTANTS USED IN CALCS-----------------------------
# Feel free to change these to see what happens

# Positive constants for calculating cell magnitude
# Should satisty mhp_a - mhp_b * sqrt(2) * (w_s - 1)/2 = 0
mhp_a = 1
mhp_b = 1

# Positive constant for smoothing polar histogram
mhp_l = 5

# Positive constant for certainty threshold of polar sector
gbd_t = 20

# Positive constant for number of consecutive sectors for a wide valley
# Should change in accordance with nsectors
gbd_smax = 18

# Positive constants for calculating cost of candidate angles
# gbd_a is for goal oriented steering, gbd_b and gbd_c are for smooth steering
# Should satisty gbd_a > gbd_b + gbd_c
gbd_a = 5
gbd_b = 2
gbd_c = 2
# ---------------


def from_map(map_fname):
    """ Create grid from text file """
    with open(map_fname, 'r') as f:
        reader = csv.reader(f, delimiter=" ")
        lines = list(reader)

    lines = list(map(lambda l: list(map(int, l)), lines))
    nrows = len(lines)
    ncols = len(lines[0])

    return lines


current = start
target_angle = None
current_angle = wrap_angle(math.degrees(
    math.atan2(start[0] - end[0], start[1] - end[1])))
previous_angle = current_angle

hg = HistogramGrid(hg_dim[0], hg_dim[1])
hg.grid = from_map("map.txt")

ph = None

steps = []
index = 0
i = 10

goal = (path[i][0],path[i][1])
j = 0
print ("VARS INITIALIZED STARTING LOOP")
while j < MAX_STEPS:
    if current == end:
            break
    while index < MAX_STEPS:
        print ("STEP", index)
        #hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current)
        if current == goal:
            break
    
        ph = VFH.map_active_hg_to_ph(hg, PolarHistogram(
            nsectors), current, w_s, mhp_a, mhp_b, mhp_l)
    
        target_angle = wrap_angle(math.degrees(
            math.atan2(current[0] - goal[0], current[1] - goal[1])))
        best_angle = VFH.get_best_direction(
            ph.polar_histogram, target_angle, current_angle, previous_angle, gbd_t, gbd_smax, gbd_a, gbd_b, gbd_c)
        print ("best_angle", best_angle)
        steps.append((index, current, target_angle,
                      wrap_angle(best_angle), ph.polar_histogram))
    
        # Compute next adjacent cell robot will be in
        next_angle = math.floor(wrap_angle(best_angle + 22.5) / 45) * 45
        print ("current", current)
        print ("next_angle", next_angle)
        next_x = int(round((math.sqrt(2) if next_angle % 90 != 0 else 1) *
                     math.cos(math.radians(next_angle + 90)),0)) + current[0]
        next_y = int(round((math.sqrt(2) if next_angle % 90 != 0 else 1) *
                     math.sin(math.radians(next_angle + 90)),0)) * -1 + current[1]
        print ("next x %d y %d" % (next_x, next_y))
    
        current = (next_x, next_y)
        previous_angle = current_angle
        current_angle = best_angle
    
        index += 1
        print ("-" * 16)
    i += 5
    if i >= len(path):
        i = len(path)-1
    goal = (path[i][0],path[i][1])
    j += 1
print ("COMPLETE")
for s in steps:
    print ("{0:2}. ({1:2}, {2:<2}) target_angle: {3:5.1f}   best_angle: {4:5.1f}".format(
        s[0], s[1][0], s[1][1],  s[2], s[3]))

hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current)

bot_temp = list(map(lambda s: s[1], steps))
bot = []
for s in bot_temp:
    bot.append([s[0],s[1]])

bot = np.array(bot)

plt.cla()
plt.gcf().set_size_inches(11, 9, forward=True)
plt.axis('equal')
plt.plot(path[:, 0], path[:, 1], 'or')
plt.plot(bot[:, 0], bot[:, 1], 'og')
plt.plot(bound[:, 0], bound[:, 1], 'sk')
plt.plot(goal[0], goal[1], '*b', label='Goal')
plt.plot(start[0], start[1], '^b', label='Origin')
plt.legend()
plt.pause(0.0001)
# vcp = (int(sys.argv[2]), int(sys.argv[3]))
# print vcp
# hg_dim = (50, 50)
# nsectors = 36
# w_s = 21

# old_hg = old.HistogramGrid.from_map("map.txt", 1)
# hg = HistogramGrid(hg_dim[0], hg_dim[1])
# hg.grid = old_hg.grid
# #print hg
# ph = VFH.map_active_hg_to_ph(hg, PolarHistogram(nsectors), vcp, w_s)
# print ph
# print VFH.get_best_direction(ph.polar_histogram, int(sys.argv[1]), smax=5)
# #ph = PolarHistogram(hg, (17, 12), 5, 16)
# #print ph
# #print ph.get_best_angle(180, 180)
