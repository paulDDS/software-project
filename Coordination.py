from utils.Point import Point

class Coordination():
    def __init__(self):
        self.targets = []
        self.agents = {}
        self.assign = {}

    def column_min(self, row, dists):
        min = dists[row][0]
        for j in len(self.agents):
            if(dists[row][j] < min):
                min = dists[row][j]
        return min
    
    def search(self, dists, row, visited_columns : list, total_dist, assignments : list, best_min_dist, best_assignments : list):
        if(row == len(self.agents)):
            if(total_dist < best_min_dist): 
                return total_dist, assignments
            else:
                return best_min_dist, best_assignments
        
        for j in range(len(self.agents)):
            if (j not in visited_columns):
                visited_columns.append(j)
                dist = total_dist + dists[row][j]
                assignments.append([row, j])

                best_dist, best_assign = self.search(dists, row + 1, visited_columns, dist, assignments, best_min_dist, best_assignments)

                if(best_dist < best_min_dist):
                    best_min_dist = best_dist
                    best_assignments = best_assign

                visited_columns.remove(j)

        return best_min_dist, best_assignments


    def assignment(self, dists):

        _ , assignments = self.search(dists, 0, [], 0, [], float('inf'), [])

        return assignments



    def allocate(self):
        dists = []
        
        for i in range(len(self.agents)):
            dists.append([])
            for j in range(len(self.agents)):
                dists[i].append(self.agents[i].dist_to(self.targets[j]))
        self.assign = {}

        positions = self.assignment(dists)

        for i, j in positions:
            self.assign.update({i : self.targets[j]})

        return 

    def declare_position(self, id, pos : Point):
        self.agents.update({id : pos})
        return
    
    def update(self, targets : list[Point]):
        self.targets = targets
        return
    
    def know_target(self, id):
        return self.assign.get(id)
    
