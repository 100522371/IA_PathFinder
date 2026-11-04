# Required imports
import numpy as np
from Location import Location
from Boundaries import Boundaries
from Radar import Radar
from tqdm import tqdm

# Constant that avoids setting cells to have an associated cost of zero
EPSILON = 1e-4
MIN_PSI_SCALED = 0
class Map:
    """ Class that models the map for the simulation """
    def __init__(self, 
                 boundaries: Boundaries,
                 height:     np.int32, 
                 width:      np.int32, 
                 radars:     np.array=None):
        self.boundaries = boundaries        # Boundaries of the map
        self.height     = height            # Number of coordinates in the y-axis
        self.width      = width             # Number of coordinates int the x-axis
        self.radars     = radars            # List containing the radars (objects)

    def generate_radars(self, n_radars: np.int32) -> None:
        """ Generates n-radars randomly and inserts them into the radars list """
        # Select random coordinates inside the boundaries of the map
        lat_range = np.linspace(start=self.boundaries.min_lat, stop=self.boundaries.max_lat, num=self.height)
        lon_range = np.linspace(start=self.boundaries.min_lon, stop=self.boundaries.max_lon, num=self.width)
        rand_lats = np.random.choice(a=lat_range, size=n_radars, replace=False) # Randomly picks n_radars unique values from lat_range and lon_range Ensures radars are scattered across the map and don’t share the same coordinate (because replace=False)
        rand_lons = np.random.choice(a=lon_range, size=n_radars, replace=False)
        self.radars = []        # Initialize 'radars' as an empty list

        # Loop for each radar that must be generated
        for i in range(n_radars):
            # Create a new radar
            new_radar = Radar(location=Location(latitude=rand_lats[i], longitude=rand_lons[i]),
                              transmission_power=np.random.uniform(low=1, high=1000000),
                              antenna_gain=np.random.uniform(low=10, high=50),
                              wavelength=np.random.uniform(low=0.001, high=10.0),
                              cross_section=np.random.uniform(low=0.1, high=10.0),
                              minimum_signal=np.random.uniform(low=1e-10, high=1e-15),
                              total_loss=np.random.randint(low=1, high=10),
                              covariance=None)

            # Insert the new radar
            self.radars.append(new_radar)
            print(f"radar {i} in (lat, lon)={new_radar.location.latitude}, {new_radar.location.longitude}\n")
        return
    
    def get_radars_locations_numpy(self) -> np.array:
        """ Returns an array with the coordiantes (lat, lon) of each radar registered in the map """
        locations = np.zeros(shape=(len(self.radars), 2), dtype=np.float32) # create an array with zeros
        for i in range(len(self.radars)):
            locations[i] = self.radars[i].location.to_numpy() # rowi ie (lat,lon)i ie locationi = [lat,lon]
        return locations # [[lat,lon], [lat,lon], [lat,lon], ...]
    
    def compute_detection_map(self) -> np.array:
        """ Computes the detection map for each coordinate in the map (with all the radars) """
        # ...
        # compute a 2D numPy array (detection_map) 
        # where each cell represents the probab of being det at that cell.
        global MIN_PSI_SCALED
        detection_map  = np.zeros(shape=(self.height, self.width), dtype=np.float32)

        latitudes = np.linspace(
            self.boundaries.max_lat,
            self.boundaries.min_lat,
            self.height
        )
        longitudes = np.linspace(
            self.boundaries.min_lon,
            self.boundaries.max_lon,
            self.width
        )
        
        # row by row from top-left (0,0) to bottom-right
        for i, current_lat in enumerate(latitudes):
            for j, current_lon in enumerate(longitudes):
                # by default for the current_cell(i,j) the first radar's detection level is the max
                max_detection_lvl = self.radars[0].compute_detection_level(current_lat, current_lon)
                
                for current_radar in self.radars:
                    curr_detection_lvl = current_radar.compute_detection_level(current_lat, current_lon)
                    
                    # update if necessary
                    if curr_detection_lvl > max_detection_lvl: 
                        max_detection_lvl = curr_detection_lvl

                # now  we save detection value in the detection map
                detection_map[i, j] = max_detection_lvl   
                # print("pos "+ str(j) + "," + str(i) + " has max_detection level " + str(max_detection_lvl) + "\n") # debuggingggg only
                if (i==0 and j==0):
                    psi_star_max = max_detection_lvl 
                    psi_star_min = max_detection_lvl
                else:
                    if max_detection_lvl > psi_star_max:
                        psi_star_max = max_detection_lvl
                    if max_detection_lvl < psi_star_min:
                        psi_star_min = max_detection_lvl
                

        # Now we update the detection_map so it has probability instead of possibility values (psi scaled)
        for i in range(0, self.height):
            for j in range(0, self.width): 
                if psi_star_max == psi_star_min: # else we get a NaN value is the formula of psi*scaled bc we would be dividing by 0
                    probab_detection = EPSILON
                else:
                    probab_detection = ( (detection_map[i, j] - psi_star_min)/(psi_star_max - psi_star_min) ) * (1-EPSILON) + EPSILON
                detection_map[i, j] = probab_detection
                print("pos "+ str(i) + "," + str(j) + " has probab_detection " + str(probab_detection) + "\n")
                if (i==0 and j==0):
                    MIN_PSI_SCALED = probab_detection
                else:
                    if probab_detection < MIN_PSI_SCALED:
                        MIN_PSI_SCALED = probab_detection 
        return detection_map

