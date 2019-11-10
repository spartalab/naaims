import pandas as pd
from Network.Intersection import Intersection


class ArcAim:

    # Makes the assumption that both intersection_traj_file and lanes_file are pandas df
    def __init__(self, intersection_traj_file, lanes_file):
        self.intersection = Intersection.Intersection()
        self.intersection.build_intersection(intersection_traj_file, lanes_file)
    
    











        