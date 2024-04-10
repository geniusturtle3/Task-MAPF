class MAPF:
    def __init__(self, grid, agents):
        self.grid = grid
        self.agents = agents

    def planPaths(self):
        for agent in self.agents:
            agent.setPath(self.grid)
            path = agent.path
            for p in path:
                self.grid[p[0]][p[1]] += 1