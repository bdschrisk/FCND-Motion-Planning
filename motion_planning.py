import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import create_grid, expand_path, walkpath
from estar import E_Star
import utils as utils

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], np.deg2rad(0.0))

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    #def send_waypoints(self):
    #    print("Sending waypoints to simulator ...")
    #    data = msgpack.dumps(self.waypoints)
    #    self.connection._master.write(data)

    def send_waypoints(self, waypoints):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING

        TARGET_ALTITUDE = 32 # 30m min altitude req. + 2m buffer
        SAFETY_DISTANCE = 2 # safety margin to account for discrepancies in the data
        GLOBAL_GOAL = (720, 30) # Top left corner
        S_ALPHA = 1.0 # attraction force weight
        S_BETA = 100.0 # repulsion force weight
        WEIGHT = 1.0 # cost weight
        BOUNDARY = 20 # repulsion energy field size
        
        LOOKAHEAD = 10 # optimisation distance size
        EPOCHS = 1 # optimisation steps

        self.target_position[2] = TARGET_ALTITUDE

        filename = 'colliders.csv'

        (lat0, lon0) = utils.get_latlon(filename)
        alt0 = 0

        print("Setting home position to: [{0}, {1}, {2}]".format(lat0, lon0, alt0))
        
        self.set_home_position(lon0, lat0, alt0)
        
        (clat, clon, calt) = self.global_position
        
        current_local = utils.global_to_local(self.global_position, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
        # Determine offsets between grid and map
        north_offset = int(np.abs(np.min(data[:, 0])))
        east_offset = int(np.abs(np.min(data[:, 1])))

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define a grid for a particular altitude and safety margin around obstacles
        grid = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        
        # convert start position to current position
        grid_start = (int(current_local[0]+north_offset), int(current_local[1]+east_offset))
        
        # Set goal to local ECEF goal location
        grid_goal = (GLOBAL_GOAL[0], GLOBAL_GOAL[1])
        # TODO: adapt to set goal as latitude / longitude position and convert
        # ???
        
        # Perform Energy* search
        print("Initialising search...")
        search = E_Star(grid)
        print("Searching for a path from {} to {}".format(grid_start, grid_goal))
        path, _ = search.search(grid_start, grid_goal, alpha=S_ALPHA, beta=S_BETA, boundary=BOUNDARY, weight=WEIGHT)
        print("Pruning path...")
        # prune path to minimize number of waypoints
        waypoints = walkpath(grid, path, grid_start, grid_goal)
        path = expand_path(grid, grid_start, grid_goal, waypoints, lookahead=LOOKAHEAD, epochs=EPOCHS)
        
        # Convert path to waypoints
        waypoints = [[int(p[0] - (current_local[0]+north_offset)), int(p[1] - (current_local[1]+east_offset)), TARGET_ALTITUDE] for p in path]
        # Set self.waypoints

        self.waypoints = waypoints
        print(waypoints)
        # send waypoints to sim
        self.send_waypoints(waypoints)

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        #while self.in_mission:
        #    time.sleep(1)

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
