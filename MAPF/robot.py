class Robot:
    def __init__(self, x, y, theta, priority):
        self.x = x
        self.y = y
        self.theta = theta
        self.priority = priority
        self.path = []
        self.on_path = False
        self.task = None