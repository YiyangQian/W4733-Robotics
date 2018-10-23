#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
import scipy
from scipy.spatial import ConvexHull

from create_map import *

def calculate_convex_hull(shift_vector):
    obstacles = load_obstacles("../data/world_obstacles.txt")
    result_convex_hull_array = []
    for ob in obstacles:
        new_obstacle = None


        for p in ob:
            p = np.array(p) # in case it is not already numpy array
            for i in shift_vector:

                new_p = p + np.array(i)
                # new_p = np.hstack((new_p, [0, 1]))
                if new_obstacle is None:
                    new_obstacle = new_p
                else:
                    new_obstacle = np.vstack((new_obstacle,new_p))

        #print(new_obstacle)
        hull = ConvexHull(new_obstacle)
        #print("Convex Hull:")
        #print(hull.vertices)

        convex_hull = None
        for i in hull.vertices:
            if convex_hull is None:
                convex_hull = hull.points[i]
            else:
                convex_hull = np.vstack((convex_hull,hull.points[i]))   
        #print("Next obstacle")      
        #print(convex_hull)
        #print('')
        # convex_hull = np.hstack((convex_hull, [0, 1]))
        result_convex_hull_array.append(convex_hull)



        
    return result_convex_hull_array
# center_shift_vector = []
#calculate_convex_hull([[-18,-18],[-18,18],[18,-18],[18,18]])


