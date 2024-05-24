import cv2
import math
import numpy as np
from matplotlib import pyplot as plt
from distance_field import df

class astar:
    def __init__(self, map_size, map_init, map_rslt):
        self.have_map = False
        self.map = None
        self.map_size = map_size
        self.map_rslt = map_rslt
        self.map_init = map_init

        self.target_point = np.array([0, 0])
        self.origin_point = np.array([0, 0])

        self.open_set  = np.zeros([0,5]).astype(int)
        self.close_set = np.zeros([0,5]).astype(int)

        self.search_success = False

        self.path_posi = []

        self.disf = df()

    def run(self, grid_map, start_posi, end_posi, obs_scope=10):
        self.SetMap(grid_map, obs_scope)
        self.SetPoint(start_posi, end_posi)
        self.SearchPath()


    def SetMap(self, grid_map, obs_scope=10):
        self.have_map = True
        self.map = grid_map

        self.disf.SetMap(grid_map, obs_scope)

    def SetPoint(self, start_posi, end_posi):
        if self.have_map:
            end_point = np.floor((end_posi - self.map_init)/self.map_rslt).astype(int)[::-1]
            end_point[0] = np.clip(end_point[0], 0, self.map_size[0])
            end_point[1] = np.clip(end_point[1], 0, self.map_size[1])
            self.target_point = end_point

            start_point = np.floor((start_posi - self.map_init)/self.map_rslt).astype(int)[::-1]
            start_point[0] = np.clip(start_point[0], 0, self.map_size[0])
            start_point[1] = np.clip(start_point[1], 0, self.map_size[1])
            self.origin_point = start_point 
        else:
            print("no map")


    def SearchPath(self):
        if self.have_map:

            self.search_success = False
            """ clear the history searching record """
            self.open_set = np.zeros([0, 5]).astype(int)
            self.close_set = np.zeros([0, 5]).astype(int)

            """ add start node into the open set """
            start_node = np.array([self.origin_point[0], self.origin_point[1], -1, 0, 0])
            start_node = self.CostCalculate(start_node)
            self.open_set = np.insert(self.open_set, 0, start_node, axis=0)
            
            while len(self.open_set) > 0:               
                """ change the min cost node """
                self.open_set  = self.open_set[np.argsort(self.open_set[:, -1])]
                current_node   = self.open_set[0]
                self.open_set  = np.delete(self.open_set, 0, axis=0)
                self.close_set = np.vstack([self.close_set, current_node])

                """ judge whether get the end point """
                if current_node[0] == self.target_point[0] and current_node[1] == self.target_point[1]:
                    self.search_success = True
                    self.BackTracking()
                    break

                """ expand child node """
                for shift in [[-1, 0], [1, 0], [0, -1], [0, 1]]:
                    row = int(current_node[0] + shift[0]); col = int(current_node[1] + shift[1])
                    if row < 0 or row > self.map_size[0]-1 or col < 0 or col > self.map_size[1]-1:
                        continue
                    elif self.map[row, col] == 1:
                        continue
                    else:
                        child_node = np.array([row, col, len(self.close_set)-1, current_node[-2]+1, 0])
                        child_node = self.CostCalculate(child_node)
                        in_open_set  = (self.open_set[:, 0:2] == [row, col]).all(axis=1)
                        in_close_set = (self.close_set[:,0:2] == [row, col]).all(axis=1)
                        if not in_close_set.any():                                     # the child node is not in close set
                            if in_open_set.any():                                      # the child node is already in open set
                                if child_node[-1] < self.open_set[in_open_set, -1]:
                                    self.open_set[in_open_set, :] = child_node
                            else:
                                self.open_set = np.insert(self.open_set, 0, child_node, axis=0)

    def BackTracking(self):
        path_grid = [self.close_set[-1, 0:2]]
        father_index = self.close_set[-1, 2]
        while father_index >= 0:
            father_node = self.close_set[father_index]
            father_index = father_node[2]
            path_grid.append(father_node[0:2])
        self.path_posi = np.flip(np.array(path_grid)[::-1] * self.map_rslt, axis=1) + self.map_init

    def CostCalculate(self, node):
        gcost = abs(node[0]-self.target_point[0]) + abs(node[1]-self.target_point[1])
        node[-1] = 0.4*node[-2] + 0.6*gcost + 20 * math.exp(-self.disf.RMDTO_Search(node[0:2]) + 1)
        return node

    def Clear(self):
        self.path_posi = []


























