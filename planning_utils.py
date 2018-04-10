from enum import Enum
from queue import PriorityQueue
import numpy as np
from bresenham import bresenham


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    #print(north_min, north_max)

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    #print(east_min, east_max)
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))
    #print(north_size, east_size)
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min_center),
                int(north + d_north + safety_distance - north_min_center),
                int(east - d_east - safety_distance - east_min_center),
                int(east + d_east + safety_distance - east_min_center),
            ]
            grid[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3]] = 1

    return grid

def walkpath(grid, path, start, goal):
    """
    Walks the action path and returns the waypoint coordinates
    """
    node = (start[0], start[1])
    walked = [np.array(node, dtype=np.int)]
    for i in path:
        node = (node[0] + i[0], node[1] + i[1])
        walked.append(np.array(node, dtype=np.int))
    walked.append(np.array((goal[0], goal[1]), dtype=np.int))
    return walked

def path_free(grid, node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    cells = list(bresenham(x1, y1, x2, y2))
    
    for (x, y) in cells:
        if grid[x, y] > 0:
            return False
    
    return True

def expand_path(grid, start, goal, path, lookahead = 10, epochs = 1):
    """
    Prunes a path by expanding the distance between visible waypoints
    """
    search_path = path
    
    for epoch in range(epochs):
        new_path = [start]
        max_ind = len(search_path) - 1
        
        current_node = start
        index = 0
        
        while (index < max_ind):

            max_dist = 0

            for i in range(lookahead):
                new_index = index + i + 0

                if (new_index > max_ind):
                    break

                node = search_path[new_index]

                if (path_free(grid, current_node, node)):
                    dist = np.linalg.norm(np.array(current_node) - np.array(node))
                    if dist > max_dist:
                        max_dist = dist
                        index = new_index

            if (max_dist == 0):
                index += 1

            new_path.append(search_path[index])

            current_node = search_path[index]
        
        search_path = new_path
    
    return new_path

class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    
    UP_LEFT = (-1, -1, np.sqrt(2))
    UP_RIGHT = (-1, 1, np.sqrt(2))
    DOWN_LEFT = (1, -1, np.sqrt(2))
    DOWN_RIGHT = (1, 1, np.sqrt(2))
    
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
        elif self == self.DOWN_RIGHT:
            return '\\'
        elif self == self.DOWN_LEFT:
            return '/'
        elif self == self.UP_RIGHT:
            return '/'
        elif self == self.UP_LEFT:
            return '\\'
    
    @property
    def cost(self):
        return self.value[2]
    
    @property
    def delta(self):
        return (self.value[0], self.value[1])
    
    def __mul__(self, v):
        return (self.value[0] * v, self.value[1] * v, self.value[2])

