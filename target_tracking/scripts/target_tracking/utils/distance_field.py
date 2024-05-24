#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import matplotlib.pyplot as plt

class df:
    def __init__(self):
        self.df_map = None
        self.grid_map = None
        self.extend_map = None

        self.search_scope = None
        self.flatten_index = None


    def SetMap(self, grid_map, search_scope=None):
        self.grid_map = grid_map
        if search_scope is not None:
            self.ResetScope(search_scope)


    def ResetScope(self, search_scope):
        self.search_scope = search_scope

        extend_size = (self.grid_map.shape + 2 * np.array([search_scope, search_scope])).astype(int)
        self.extend_map = np.zeros(extend_size)
        self.extend_map[search_scope:search_scope + self.grid_map.shape[0], search_scope:search_scope + self.grid_map.shape[1]] = self.grid_map

        index = np.array([[i, j, np.sqrt(i ** 2 + j ** 2)] for i in range(-search_scope, search_scope + 1) for j in range(-search_scope, search_scope + 1)])
        sorted_indices = np.argsort(index[:, 2])
        self.flatten_index = (index[sorted_indices]).astype(int)
        self.flatten_index[:, 0:2] += search_scope


    def sedt_8points(self):
        rows, cols = self.grid_map.shape
        sedt = np.full((rows, cols), np.inf)

        for i in range(rows):                                           # Forward pass
            for j in range(cols):
                if self.grid_map[i, j] == 1:
                    sedt[i, j] = 0
                else:
                    if i > 0:
                        sedt[i, j] = min(sedt[i, j], sedt[i - 1, j] + 1)
                    if j > 0:
                        sedt[i, j] = min(sedt[i, j], sedt[i, j - 1] + 1)
                    if i > 0 and j > 0:
                        sedt[i, j] = min(sedt[i, j], sedt[i - 1, j - 1] + np.sqrt(2))

        for i in range(rows - 1, -1, -1):                              # Backward pass
            for j in range(cols - 1, -1, -1):
                if i < rows - 1:
                    sedt[i, j] = min(sedt[i, j], sedt[i + 1, j] + 1)
                if j < cols - 1:
                    sedt[i, j] = min(sedt[i, j], sedt[i, j + 1] + 1)
                if i < rows - 1 and j < cols - 1:
                    sedt[i, j] = min(sedt[i, j], sedt[i + 1, j + 1] + np.sqrt(2))

        self.df_map = sedt


    def RMDTO_Search(self, center_point):
        if self.grid_map[center_point[0], center_point[1]] == 0:
            index_min = np.array(center_point)
            index_max = np.array(center_point) + 2*self.search_scope + 1
            points_part = self.extend_map[index_min[0]:index_max[0], index_min[1]:index_max[1]]

            region_flatten = points_part[self.flatten_index[:, 0], self.flatten_index[:, 1]]
            k = np.where(region_flatten == 1)[0]
            if len(k) > 0:
                return self.flatten_index[k[0], 2]
            else:
                return self.search_scope
        else:
            return 0

        
def sedt_8points(grid_map):
    rows, cols = grid_map.shape
    sedt = np.full((rows, cols), np.inf)

    for i in range(rows):                                           # Forward pass
        for j in range(cols):
            if grid_map[i, j] == 1:
                sedt[i, j] = 0
            else:
                if i > 0:
                    sedt[i, j] = min(sedt[i, j], sedt[i - 1, j] + 1)
                if j > 0:
                    sedt[i, j] = min(sedt[i, j], sedt[i, j - 1] + 1)
                if i > 0 and j > 0:
                    sedt[i, j] = min(sedt[i, j], sedt[i - 1, j - 1] + np.sqrt(2))

    for i in range(rows - 1, -1, -1):                              # Backward pass
        for j in range(cols - 1, -1, -1):
            if i < rows - 1:
                sedt[i, j] = min(sedt[i, j], sedt[i + 1, j] + 1)
            if j < cols - 1:
                sedt[i, j] = min(sedt[i, j], sedt[i, j + 1] + 1)
            if i < rows - 1 and j < cols - 1:
                sedt[i, j] = min(sedt[i, j], sedt[i + 1, j + 1] + np.sqrt(2))

    return sedt

if __name__ == "__main__":
    image = np.loadtxt("../Data/obst_map.txt", dtype=int, delimiter='\t')
    grid_map = image

    search_scope = 10
    dis_f = df()
    dis_f.SetMap(grid_map, search_scope)

    t = []
    for r in range(1, image.shape[0]):
        dis_f.ResetScope(r)
        esdf_map = np.zeros(image.shape)
        ts = 0
        for i in range(grid_map.shape[0]):
            for j in range(grid_map.shape[1]):
                center_points = np.array([i,j])
                t0 = time.time_ns()
                esdf_map[i, j] = dis_f.RMDTO_Search(center_points)
                ts += (time.time_ns() - t0)
        t.append(ts/np.prod(grid_map.shape))
    t = np.array(t) / 1e6
    plt.plot(range(1, image.shape[0]), t)
    plt.xlabel("search radius/m")
    plt.ylabel("time consuming/ms")
    plt.show()

    # esdf_map = np.zeros(image.shape)
    # ts = 0
    # for i in range(grid_map.shape[0]):
    #     for j in range(grid_map.shape[1]):
    #         center_points = np.array([i,j])
    #         esdf_map[i, j] = dis_f.RMDTO_Search(center_points)
    #
    # dis_f.sedt_8points()
    #
    # m = max(np.max(dis_f.df_map), np.max(esdf_map))
    # esdf_map1 = esdf_map / m
    # esdf_map2 = dis_f.df_map / m
    # plt.figure(1)
    # plt.imshow(image,  cmap='gray')
    # plt.figure(2)
    # plt.imshow(esdf_map1,  cmap='gray', vmin=0, vmax=1)
    # plt.colorbar()  # 添加颜色条
    # plt.axis('off')
    # plt.figure(3)
    # plt.imshow(esdf_map2,  cmap='gray', vmin=0, vmax=1)
    # plt.colorbar()  # 添加颜色条
    # plt.axis('off')
    # plt.show()


    # grid_map = image
    # extend_size = (np.array(grid_map.shape) + 2 * search_scope).astype(int)
    # extend_map = np.zeros(extend_size)
    # extend_map[search_scope:search_scope + grid_map.shape[0], search_scope:search_scope + grid_map.shape[1]] = grid_map
    #
    # center_points = np.array([50,50])
    #
    # index_origin = np.array([[i, j, np.sqrt(i ** 2 + j ** 2)] for i in range(-search_scope, search_scope + 1) for j in range(-search_scope, search_scope + 1)])
    # sorted_indices = np.argsort(index_origin[:, 2])
    # index_origin = (index_origin[sorted_indices]).astype(int)
    # index_origin[:,0:2] += search_scope
    #
    # esdf_map = np.zeros(image.shape)
    # ts = 0
    # for i in range(grid_map.shape[0]):
    #     for j in range(grid_map.shape[1]):
    #         center_points = np.array([i,j])
    #         t0 = time.time_ns()
    #         if grid_map[center_points[0], center_points[1]] == 0:
    #             index_min = center_points - search_scope + search_scope
    #             index_max = center_points + search_scope + search_scope
    #             points_part = extend_map[index_min[0]:index_max[0]+1, index_min[1]:index_max[1]+1]
    #
    #             points_flatten = points_part[index_origin[:, 0], index_origin[:, 1]]
    #
    #             k = np.where(points_flatten == 1)[0]
    #             if len(k) > 0:
    #                 d = index_origin[k[0], 2]
    #             else:
    #                 d = search_scope
    #         else:
    #             d = 0
    #         esdf_map[i, j] = d
    #         ts += (time.time_ns() - t0)
    # print(ts/np.prod(grid_map.shape))
    #
    #
    # ts = 0
    # for i in range(grid_map.shape[0]):
    #     for j in range(grid_map.shape[1]):
    #         center_points = np.array([i,j])
    #         index_min = center_points - search_scope + search_scope
    #         index_max = center_points + search_scope + search_scope
    #         t0 = time.time_ns()
    #         if grid_map[center_points[0], center_points[1]] == 0:
    #             points_part = extend_map[index_min[0]:index_max[0], index_min[1]:index_max[1]]
    #             k = np.array(np.where(points_part == 1)).T
    #             if len(k)>0:
    #                 d = np.sort(np.linalg.norm(k- search_scope - 1, axis=1))[0]
    #             else:
    #                 d = 0
    #         else:
    #             pass
    #         ts += (time.time_ns() - t0)
    # print(ts/np.prod(grid_map.shape))

