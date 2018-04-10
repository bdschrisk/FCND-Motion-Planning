## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

## Rubric Points
### Here I will consider the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points individually and describe how I addressed each point in my implementation.

---

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
The abovementioned scripts contain a basic planning implementation that includes a state based control loop for managing the drones behaviour, aswell as planning a suitable path.  The functionality in `planning_utils.py` allows the world to be discretised into a grid or graph representation, while the `motion_planning.py` code is an implementation of the Drone object for controlling the behaviour and mission of the drone.

An overview of the two solutions provided are as follows:

Feature | backyard_flyer | motion_planning
--- | --- | --- | ---
`Implements a path planner for finding paths` | no | **yes**
`Uses dynamic searching for missions` | no | **yes**
`Includes an internal state of the configuration space` | no | **yes**
`Uses global positioning for following paths` | no | **yes**

### Implementing Your Path Planning Algorithm
#### On the Energy* Search using Potential Fields based algorithm...

#### 1. Set your global home position
To accurately generate a valid path that the quadcopter can follow, it needs to know the relative coordinates of each successive waypoint.  This is accomplished by setting the home location, which in this case is obtained by reading the provided configuration space state in the 'colliders' file.

``` 
(lat0, lon0) = utils.get_latlon(filename)
alt0 = 0 # setting home altitude to zero
```
*In the above lines, a utility function is called to extract the configuration space state from the provided environment file.*

#### 2. Set your current local position
After the global coordinates have been obtained, the quadcopter needs to realise the home location for advancing to waypoints relative to its position in the world.  This is achieved by the following line:

```
self.set_home_position(lon0, lat0, alt0)
```

#### 3. Set grid start position from local position
When constructing the grid representation, the world is discretized to reduce the size of the grid space so that the minimum data points form the upper left of the grid.  This means that the relative center is the offset of each direction in the NED frame.

Here we obtain the offsets for determining our global position relative to the grid world:
```
north_offset = int(np.abs(np.min(data[:, 0])))
east_offset = int(np.abs(np.min(data[:, 1])))
```

#### 4. Set grid goal position from geodetic coords
Once the relative offsets are determined, the actual local position is taken using the global GNSS and home coordinates:

```
current_local = utils.global_to_local(self.global_position, self.global_home)
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
In this project the search algorithm has been replaced by an extended version of A*, which I've called Energy* search.

*Overview:*

Energy* search is an extension of A* which uses potential field based methods to avoid obstacles. Using an online scanning approach, the repulsion forces from neighbouring obstacles of each state opposes the attractive force towards the goal. By adjusting the relative weights of alpha and beta in the energy functions, different obstacle avoidance behaviour can be achieved.
    
Because of the obstacle avoidance behaviour, sacrifices in the path costs can be made by using a greedy best-first search approach. This greatly speeds up time taken to find a valid solution while still maintaining admissibility.

The local energy field for each state is the difference between the two interacting energy fields:

> Attractive force:  
![Attractive force formula](./images/attractive_force.png)

> Repulsive force:   
![Repulsuve force formula](./images/repulsive_force.png)

> Energy\* Heuristic function visualisation   
![Energy* heuristic](./images/energy_star-heuristic2.png)

In addition to the energy field heuristic method described above, a "jump step" mechanism has been added to the search routine to speed up convergence.  This works by adding predetermined jump steps to each successive action-step, with each jump step cost scaled by a relative weight to control jumping behaviour.

This jumping mechanism allows the algorithm to traverse large open environments quickly, while still being able to navigate small passage ways by falling back to single step search.

All six cardinal actions, left, down, up and right, along with their orthogonal counterparts are used to find an optimal path.

#### 6. Cull waypoints 
To remove redundant waypoints, I have implemented a path expansion algorithm which uses the Bresenham algorithm for iteratively finding the maximal visible waypoint from each candidate state.  I found this method to be far superior to the collinearity test, and removed 10 times more waypoints than the collinearity method.  It was also superior in that the new path was corrective and yeilded more optimal results.

The results below compare the results of the original path, using both the collinearity test and the path expansion algorithm.

> Original unpruned path   
![Original Unpruned Path](./images/path-pruning-raw.png)
    
> Path pruning using collinearity tests   
![Original Unpruned Path](./images/path-pruning-collinear.png)
    
> Path pruning by expansion method    
![Original Unpruned Path](./images/path-pruning-expansion.png)

*As you can see from the images above the expansion method results in a far superior path while still being admissable and maintaining optimality.*

### Execute the flight
#### 1. Does it work?
It works!

![In action](./images/in-action.png)


