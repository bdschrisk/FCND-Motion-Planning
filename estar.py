import numpy as np
from queue import PriorityQueue

from planning_utils import Action

class E_Star(object):
    def __init__(self, grid):
        self.m = grid.shape[0]
        self.n = grid.shape[1]

        self.map = grid

        self.actions = list(Action)
    
    def within_bounds(self, node):
        """
        Returns True if the node is in the world.
        """
        y, x = node
        return (x >= 0 and x < self.n) and (y >= 0 and y < self.m)
    
    def valid_actions(self, node, collision = True):
        """
        Returns a list of valid actions given a grid and current node.
        """
        x, y = node

        new_actions = list()
        # check if the node is off the grid or it's an obstacle
        for action in self.actions:
            x1, y1 = x + action.delta[0], y + action.delta[1]
            if (self.within_bounds((x1, y1)) and (not collision or self.map[x1, y1] == 0)):
                new_actions.append(action)

        return new_actions

    # Attractive forces towards the goal
    def attractive_force(self, position, goal_position, alpha = 1.0):
        pos = np.array(position)
        goal = np.array(goal_position)
        # euclidean distance + manhattan
        h = np.linalg.norm(pos - goal) + np.sum(np.abs(pos - goal))
        return alpha * h

    # Repulsive forces away from obstacles
    def repulsive_force(self, current_node, boundary = 4, beta = 5.0):
        actions = self.valid_actions(current_node, collision = False)

        force = 0
        for i in range(boundary):
            depth = boundary - i
            for a in actions:
                next_node = (current_node[0] + a.delta[0] * depth, current_node[1] + a.delta[1] * depth)
                if (self.within_bounds(next_node) and self.map[next_node[0], next_node[1]] == 1):
                        distance = np.linalg.norm(np.array(current_node) - np.array(next_node))
                        if (distance <= 0):
                            distance = 1e-06
                        force += beta * ((1.0 / boundary) - (1.0 / distance)) * (1.0 / distance * distance)
        
        return force

    # Energy star search
    def search(self, start, goal, alpha = 1.0, beta = 5.0, boundary = 5, 
        jump_steps = [1, 5, 10], jump_weights = [1.0, 0.5, 0.25], weight = 0.1):

        path = []
        queue = PriorityQueue()
        queue.put((0, 0, start))
        visited = set(start)

        branch = {}
        found = False
        
        while not queue.empty():
            item = queue.get()
            current_cost = item[1]
            current_node = item[2]

            if current_node == goal:        
                print('Found a path.')
                found = True
                break
            else:
                actions = self.valid_actions(current_node)
                
                for (jump_step, jump_weight) in zip(jump_steps, jump_weights):
                    for action in actions:
                        # get the tuple representation
                        da = action.delta
                        cost = action.cost * (jump_step * jump_weight)
                        next_node = (current_node[0] + da[0] * jump_step, \
                                    current_node[1] + da[1] * jump_step)
                        # calculate new cost, c + g()
                        new_cost = current_cost + cost
                        # calculate new heuristic, a() + r()
                        new_heuristic = (self.attractive_force(next_node, goal, alpha=alpha) - 
                                         self.repulsive_force(next_node, beta=beta, boundary=boundary))

                        if next_node not in visited:
                            visited.add(next_node)
                            # switched sort order to use heuristic first for 10x speedup
                            queue.put((new_heuristic, new_cost * weight, next_node))

                            branch[next_node] = (new_cost, current_node, action, jump_step)
                
        path = []
        path_cost = 0
        
        if found:    
            # retrace steps
            path = []
            n = goal
            path_cost = branch[n][0]
            while branch[n][1] != start:
                path.append(branch[n][2] * branch[n][3])
                n = branch[n][1]
            
            path.append(branch[n][2] * branch[n][3])
                
        return path[::-1], path_cost

