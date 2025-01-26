from utils.Point import Point
import numpy as np
from scipy.optimize import linear_sum_assignment

class Coordination():
    def __init__(self):
        self.targets = []
        self.agents = {}
        self.assign = {}

    def allocate(self):
        dists = []
        
        for i in range(len(self.agents)):
            dists.append([])
            for j in range(len(self.agents)):
                dists[i].append(self.agents[i].dist_to(self.targets[j]))
        self.assign = {}



        np_dists = np.array(dists)

        rows, columns = linear_sum_assignment(np_dists)

        for i in rows:
            self.assign.update({i : self.targets[columns[i]]})

        return 

    def declare_position(self, id, pos : Point):
        self.agents.update({id : pos})
        return
    
    def update(self, targets : list[Point]):
        self.targets = targets
        return
    
    def know_target(self, id):
        return self.assign.get(id)
    
