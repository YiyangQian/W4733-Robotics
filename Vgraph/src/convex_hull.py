#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
import scipy
from scipy.spatial import ConvexHull

from create_map import *

def calculate_convex_hull(shift_vector):
    obstacles = load_obstacles("~/catkin_ws/src/vgraph/data/world_obstacles.txt")
    result_convex_hull_array = []
    for ob in obstacles:
        new_obstacle = None

        for p in ob:
            p = np.array(p) # in case it is not already numpy array
            for i in shift_vector:

                new_p = p + np.array(i)
                if new_obstacle is None:
                    new_obstacle = new_p
                else:
                    new_obstacle = np.vstack((new_obstacle,new_p))

        hull = ConvexHull(new_obstacle)
        convex_hull = None
        for i in hull.vertices:
            if convex_hull is None:
                convex_hull = hull.points[i]
            else:
                convex_hull = np.vstack((convex_hull,hull.points[i]))   
        result_convex_hull_array.append(convex_hull)        
    return result_convex_hull_array
