import numpy as np
import matplotlib.pyplot as plt

class Grid:

    FREE = 0
    OCCUPIED = 1
    VISITED = 2

    def __init__(self, grid_size=600):
        """Simple grid for A* pathfinding"""
        self.grid_size = grid_size
        self._occupancy_grid = np.zeros((grid_size, grid_size), dtype=np.uint8)

    def is_occupied(self, x, y):
        """Check if cell is occupied"""
        if not self.is_in_bounds(x, y):
            return True
        return self._occupancy_grid[x, y] == self.OCCUPIED

    def is_visited(self, x, y):
        """Check if cell has been visited by A*"""
        if not self.is_in_bounds(x, y):
            return True
        return self._occupancy_grid[x, y] == self.VISITED

    def is_in_bounds(self, x, y):
        """Check if coordinates are within grid bounds"""
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size

    def mark_visited(self, x, y):
        """Mark cell as visited by A*"""
        if self.is_in_bounds(x, y):
            self._occupancy_grid[x, y] = self.VISITED

    def set_cell(self, x, y, value):
        """Set cell to any value (for LiDAR updates later)"""
        if self.is_in_bounds(x, y):
            self._occupancy_grid[x, y] = value
class Node:
    def __init__(self, pt, dist_from_start, est_to_goal, prev=None):
        self.pt = pt ## (x,y) position
        self.dist_from_start = dist_from_start
        self.est_to_goal = est_to_goal #estime to goal h(n)
        self.prev = prev

class AStar:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        start_node = Node(start, 0, self.dist(start, goal), None)
        self.nodes_to_investigate = [start_node]
    
    def get_most_promising_node(self):
        """
        finds the node with the lowest f(n) = g(n) + h(n) value and adds it to the list of nodes to investigate.
        """
        if not self.nodes_to_investigate:
            raise ValueError('there are no nodes to investigate')

        min_dist = np.inf
        min_idx = -1
        for idx, node in enumerate(self.nodes_to_investigate):
            dist = node.dist_from_start + node.est_to_goal #g(n) + h(n)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx

        return self.nodes_to_investigate.pop(min_idx)
    
    def get_path(self):
        """
        performs the A Star search and returns a path, if one has been found
        :return: list of (x, y) tuples, or None if no path could be found
        """
        where_from = {}

        while self.nodes_to_investigate:
            node = self.get_most_promising_node()
            x, y = node.pt

            if self.grid.is_visited(x, y):
                continue

            self.grid.mark_visited(x, y)
            where_from[(x, y)] = node.prev

            if (x, y) == self.goal:
                path = [(x, y)]
                prev = where_from[(x, y)]
                while prev is not None:
                    path.append(prev)
                    prev = where_from[prev]
                return path
            
            for movement in self.get_movements():
                pass