import numpy as np
from sklearn.neighbors import NearestNeighbors

# This class implements the computation of the closest occupied cell in the 
# map to the end of the laser beam.  

# Add the following line to import this file into localization.py (note the period, this is the 
# Python3 way of importing from the current directory): 
#       from .likelihood import LikelihoodField 

# You should create the object at the end of the get_map method in localization.py
# since the constructor requires the map 
#       self.likelihood = LikelihoodField(self.map)  
# 
# Then, a function call will return the distance to a point.  Note that the point must be in 
# meters assuming the origin of the map (do not use pixel coordinates).  In other words, make sure 
# to offset the computed x and y positions of the laser beam by the origin prior to calling the 
# distance function: 
#       dist = self.likelihood.get_closest_obstacle_distance(2,2) 

# This dist is the distance from the end of the laser beam to the closest occupied cell in 
# occupancy grid map in meters.  
 

class LikelihoodField(object):
    def __init__(self, m):
        
        # store the map 
        self.map = m 

        # The coordinates of each grid cell in the map
        X = np.zeros((self.map.info.width*self.map.info.height, 2))

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                X[curr, 0] = float(i)
                X[curr, 1] = float(j)
                curr += 1

        # The coordinates of each occupied grid cell in the map
        occupied = np.zeros((total_occupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    occupied[curr, 0] = float(i)
                    occupied[curr, 1] = float(j)
                    curr += 1
        # use super fast scikit learn nearest neighbor algorithm
        nbrs = NearestNeighbors(n_neighbors=1,
                                algorithm="ball_tree").fit(occupied)
        distances, indices = nbrs.kneighbors(X)

        self.closest_occ = np.zeros((self.map.info.width, self.map.info.height))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                self.closest_occ[i, j] = distances[curr][0]*self.map.info.resolution
                curr += 1


    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in
            the map.  If the (x,y) coordinate is out of the map boundaries, nan
            will be returned. """
        x_coord = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
        y_coord = int((y - self.map.info.origin.position.y)/self.map.info.resolution)

        is_valid = (x_coord >= 0) & (y_coord >= 0) & (x_coord < self.map.info.width) & (y_coord < self.map.info.height)
        return self.closest_occ[x_coord, y_coord] if is_valid else float('nan')
