import rospy
import math
from robot import Robot
from priority_queue import PriorityQueue

class StaticMAPF:
    def __init__(self):
        rospy.init_node('Static_MAPF')
        self.open_robots = PriorityQueue()
        self.assigned_robots = PriorityQueue()
        self.tasks = []
        pass

    def add_robot(self, x, y, theta, priority):
        self.robots.append(Robot(x, y, theta, priority),priority)
    
    '''
    def set_paths
        assign robots to tasks using whatever setup sam does
        for robot in robots:
            set an initial path with path_planner - pull from node? need to get actual path values not pure pursuit
            check if that path collides with an existing path - loop over robots in assigned_robots
                if collision
                    rerun path planning with collision point and all neighbors set as +x (x = 1, 10, idk) in map copy
                    set new path and retry collision check
            assign path to robot
    '''
    
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Static_MAPF')
    StaticMAPF().run()