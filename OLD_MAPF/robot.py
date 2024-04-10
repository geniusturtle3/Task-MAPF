from astar import astar
class Robot:
    def __init__(self, id, start, goal, prio):
        self.id = id
        self.start = start
        self.goal = goal
        self.prio = prio
        self.path = []
        self.current = start

    def setPath(self, grid):
        self.path = self.astar(grid, self.current, self.goal)

    