from utils.Point import Point
import numpy as np
from scipy.optimize import linear_sum_assignment

class Coordination():
    def __init__(self):
        self.targets = []
        self.agents = {}
        self.assign = {}

    def max_dist_index(self, dists):
        index = [-1, -1]
        max = -1
        for i in range(len(self.agents)):
            for j in range(len(self.agents)):
                if(dists[i][j] > max):
                    max = dists[i][j]
                    index = [i, j]
        return index

    def cross(self, dists, location):
        i = location[0]
        for j in range(len(self.agents)):
            if(j != location[1]):
                dists[i][j] = -1
        j = location[1]
        for i in range(len(self.agents)):
            if(i != location[0]):
                dists[i][j] = -1
        return dists
    
    def one_option(self, dists, type = 'row'):
        if (type == 'row'):
            for i in range(len(self.agents)):
                count = 0
                for j in range(len(self.agents)):
                    if(dists[i][j] != -1):
                        count += 1
                if(count == 1): return i
            return -1
        if (type == 'column'):
            for j in range(len(self.agents)):
                count = 0
                for i in range(len(self.agents)):
                    if(dists[i][j] != -1):
                        count += 1
                if(count == 1): return j
            return -1
    
    def last_option(self, dists):
        row = self.one_option(dists, 'row')
        if(row != -1 ):
            i = row
            for j in range(len(self.agents)):
                if(dists[i][j] != -1): return [i, j]
        column = self.one_option(dists, 'column')
        if(column != -1 ):
            j = column
            for i in range(len(self.agents)):
                if(dists[i][j] != -1): return [i, j]
        return -1

    def allocate(self):
        dists = []
        
        for i in range(len(self.agents)):
            dists.append([])
            for j in range(len(self.agents)):
                dists[i].append(self.agents[i].dist_to(self.targets[j]))
        self.assign = {}

        index = self.max_dist_index(dists)

        while len(self.assign) < len(self.agents):
            print(dists)
            print(len(self.assign))
            print(len(self.agents))
            repeat = True
            while (repeat == True):
                print(dists)
                print(len(self.assign))
                print(len(self.agents))
                index = self.last_option(dists)
                if(index != -1):
                    self.cross(dists, index)
                    self.assign.update({index[0] : self.targets[index[1]]})
                    dists[index[0]][index[1]] = -1
                else:
                    repeat = False
            
            index = self.max_dist_index(dists)
            dists[index[0]][index[1]] = -1

        return 

    def declare_position(self, id, pos : Point):
        self.agents.update({id : pos})
        return
    
    def update(self, targets : list[Point]):
        self.targets = targets
        return
    
    def know_target(self, id):
        return self.assign.get(id)
    
