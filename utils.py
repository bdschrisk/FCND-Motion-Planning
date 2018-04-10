import utm
import numpy as np

def get_latlon(filename):
    """
    Reads the latitude and longitude from the configuration space file
    """
    lat, lon = 0, 0

    with open(filename, 'r') as fd:
        for n, line in enumerate(fd):
            if (n == 1):
                lat, lon = [float(x) for x in line.split(",")]
                break
    
    return lat, lon

def global_to_local(global_position, global_home):
    """
    Returns the local coordinates from the global position and reference position
    """
    (east_home, north_home, _, _) = utm.from_latlon(global_home[1], global_home[0])
    (east, north, _, _) = utm.from_latlon(global_position[1], global_position[0])
    
    local_position = np.array([north - north_home, east - east_home, -global_position[2]])
    
    return local_position

def local_to_global(local_position, global_home):
    """
    Returns the global coordinates from the local position and reference position
    """
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(global_home[1], global_home[0])
    (lat, lon) = utm.to_latlon(east_home + local_position[1], north_home + local_position[0], zone_number, zone_letter)
    
    global_position = np.array([lon, lat, -local_position[2]])
    
    return global_position