#!/usr/bin/env python3
from __future__ import annotations
import rospy
import math
from std_srvs.srv import Empty
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, PointStamped 
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point32, Point, Pose, PoseStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from tf.transformations import euler_from_quaternion
import cv2 as cv
import tf
import math
import random
from copy import deepcopy
import numpy as np
from nav_msgs.srv import GetPlan
from path_planner import PathPlanner
from priority_queue import PriorityQueue
import rospy.timer


class GlobalManager:
    def __init__(self) -> None:
        "Constructor"

        rospy.init_node("global_manager")

        self.odomReqPubs: list[rospy.Publisher]=[]
        self.goals: list[rospy.Publisher]=[]
        self.stoppers: list[rospy.Publisher]=[]
        self.numRobots=6
        self.priorityMultiplier = 6
        self.priorityOffset = 5*self.priorityMultiplier
        
        for i in range(1,self.numRobots+1):
            self.goals.append(rospy.Publisher('/robot_'+str(i)+'/path'+str(i),Path,queue_size=10))
            # rospy.Subscriber('/robot_'+str(i)+'/newGoal',Odometry,self.chooseGoal)
            # rospy.Subscriber("/robot_"+str(i)+'/replan',Odometry,self.sameGoal)
            rospy.Subscriber('/robot_'+str(i)+'/reqedodom',Odometry,self.update_odometry)
            self.odomReqPubs.append(rospy.Publisher('/robot_'+str(i)+'/reqingodom',String,queue_size=10))
            self.stoppers.append(rospy.Publisher('/robot_'+str(i)+'/stopRobot', String, queue_size=10))
            self.goalPublisher=rospy.Publisher('robot_'+str(i)+'/robotGoal', Float64MultiArray, queue_size=20)

        self.a_star_pub = rospy.Publisher("/a_star", GridCells, queue_size=0)
        self.path_pub = rospy.Publisher("/apath", Path, queue_size=0)
        self.cspace_pub = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=10)
        self.cspace_base_pub = rospy.Publisher("/path_planner/basecspace", GridCells, queue_size=10)
        rospy.Timer(rospy.Duration(90), self.timer_callback)

        
        # rospy.Subscriber('/initialpose',
        #                  PoseWithCovarianceStamped, self.chooseGoals)

        # self.goals=[self.pathPub1,self.pathPub2,self.pathPub3,self.pathPub4,self.pathPub5,self.pathPub6,self.pathPub7,self.pathPub8]

        self.px=[0,0,0,0,0,0,0,0]
        self.py=[0,0,0,0,0,0,0,0]
        self.gridPaths = [[], [], [], [], [], [], [], []]
        self.pathMessages = [None, None, None, None, None, None, None, None]
        self.goalPoints={}
        self.isPlanned = [False, False, False, False, False, False, False, False]
        self.unplannedRobots = PriorityQueue()

        for i in range(1,self.numRobots+1):
            self.goalPoints[i]=(i,0,(0,0))

        self.map = PathPlanner.request_map()
        self.cspaced=PathPlanner.calc_cspace(self.map, 1.5)
        self.baseCspace=deepcopy(self.cspaced)
        self.publish_cspace(self.baseCspace,self.cspace_base_pub)
        self.chooseGoals()
        rospy.Timer(rospy.Duration(150), self.timer_callback)
        
        

    def timer_callback(self, event):
        rospy.loginfo("Timer callback")
        self.chooseGoals()
        pass

    def publish_cspace(self,data=None,pub=None):
        if data==None:
            data=self.cspaced
        if pub==None:
            pub=self.cspace_pub
        rospy.loginfo("publishing new cspace")
        cspace = GridCells()
        cspace.header.frame_id = "map"
        cspace.cell_width = self.map.info.resolution
        cspace.cell_height = self.map.info.resolution
        cspace.cells = []

        for i in range(len(data.data)):
            if self.cspaced.data[i] == 100:
                # Publish only the inflated cells
                cspace.cells.append(PathPlanner.grid_to_world(
                    self.cspaced, (i % self.cspaced.info.width, int(i / self.cspaced.info.width))))

        pub.publish(cspace)

    def mapforobot(self,robot=-1):
        if robot==-1:
            return PathPlanner.calc_robots(deepcopy(self.baseCspace),self.px,self.py)
        return PathPlanner.calc_robots(deepcopy(self.baseCspace),self.px,self.py,ignoreRobot=robot)
        


    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # print("updating odom")
        # print(msg.header.frame_id)
        frame_id=msg.header.frame_id
        # rospy.loginfo(str.split(str.split(frame_id,'/')[0],'_')[1])
        number=int(str.split(str.split(frame_id,'/')[0],'_')[1])
        self.odomHeader=msg.header
        self.odomPoint=msg.pose.pose
        self.px[number-1] = msg.pose.pose.position.x
        self.py[number-1] = msg.pose.pose.position.y
        self.quat_orig = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([self.quat_orig.x, self.quat_orig.y, self.quat_orig.z, self.quat_orig.w])
        self.ptheta = yaw
        return number

    def chooseGoals(self):
        self.stopRobots([1,2,3,4,5,6])
        self.callUpdateAllPoses()
        # self.cspaced=self.mapforobot()
        # self.publish_cspace(self.baseCspace,self.cspace_base_pub)
        self.publish_cspace()
        robots_to_plan=self.getUnassignedRobots()
        for i in robots_to_plan:
            self.chooseGoal(i)
        rospy.loginfo("All goals chosen")
        rospy.loginfo(str(self.goalPoints))
        rospy.loginfo(str(self.isPlanned))
        for i in robots_to_plan:
            sending=Float64MultiArray()
            goalpoint=self.goalPoints[i]
            sending.data = [goalpoint[0], goalpoint[1], goalpoint[2][0], goalpoint[2][1]]
            self.goalPublisher.publish(sending)
        self.prioritizeRobots()
        
    
    def chooseGoal(self,robot):

        mapdata=self.baseCspace#self.mapforobot(robot)
        pos=Point()
        pos.x=self.px[robot-1]
        pos.y=self.py[robot-1]
        start=PathPlanner.world_to_grid(mapdata,pos)
        opencells=[]
        for i in range(len(mapdata.data)):
            pnt=mapdata.data[i]
            if pnt<100 and pnt>-1:
                opencells.append(i)
        # print(opencells)
        goalPoint=[]
        while len(goalPoint)==0:
            tempPoint=random.choice(opencells)
            gridPoint=PathPlanner.index_to_grid(mapdata,tempPoint)
            path=PathPlanner.a_star(mapdata,start,gridPoint)
            if len(path)>0:
                goalPoint=gridPoint
                goalTime = random.randint(30,100)+6.9
                self.goalPoints[robot] = (robot, goalTime, goalPoint) 
            # else:
            #     path=PathPlanner.a_star(self.baseCspace,start,gridPoint)
            #     if len(path)>0:
            #         rospy.loginfo("Path found in base cspace Robot "+str(robot))
            #         goalPoint=gridPoint
            #         goalTime = random.randint(50,200)+6.9
            #         self.goalPoints[robot] = (robot, goalTime, goalPoint)
        path_msg=PathPlanner.path_to_message(self.cspaced,path)
        self.gridPaths[robot-1] = path
        self.pathMessages[robot-1] = path_msg
        self.isPlanned[robot-1] = True
        
        rospy.loginfo("Done choosing goal for robot "+str(robot))



    def allGoalsChosen(self):
        return all(self.isPlanned)
    
    def prioritizeRobots(self):
        for key, value in self.goalPoints.items():
            path = self.pathMessages[key-1]
            path_length = PathPlanner.path_length(self.cspaced,path)
            priority = self.priorityMultiplier*path_length
            if priority > value[1]:
                priority += self.priorityOffset
            self.unplannedRobots.put(value, priority)
        
        self.releaseRobots(self.adjustPaths())

    def releaseRobots(self, plannedRobots):
        for robot in plannedRobots:
            path_msg = self.pathMessages[robot-1]
            self.goals[robot-1].publish(path_msg)
            self.path_pub.publish(path_msg)
        self.isPlanned = [False, False, False, False, False, False, False, False]

    def sameGoal(self,msg, publish=True):
        mapdata=self.cspaced
        robot=self.update_odometry(msg)
        pos=Point()
        pos.x=self.px[robot-1]
        pos.y=self.py[robot-1]
        start=PathPlanner.world_to_grid(mapdata,pos)
        path=PathPlanner.a_star(mapdata,start,self.goalPoints[robot-1][2])
        path_msg=PathPlanner.path_to_message(self.cspaced,path)
        self.path_pub.publish(path_msg)
        if publish:
            self.goals[robot-1].publish(path_msg)
            
    def stopRobots(self,robots):
        for i in robots:
            msg=String()
            msg.data="stop"
            self.stoppers[i-1].publish(msg)     

    def callUpdateAllPoses(self):      
        for i in range(1,self.numRobots+1):
            #requesting
            # rospy.loginfo(str((self.px[i-1],self.py[i-1])))
            msg=String()
            msg.data="requesting_"+str(i)
            self.odomReqPubs[i-1].publish(msg)
            rospy.wait_for_message('/robot_'+str(i)+'/reqedodom',Odometry)
            # rospy.loginfo(str((self.px[i-1],self.py[i-1])))
            # rospy.loginfo("received_"+str(i))
    
    def checkLineCollision(self, a, b, c, d):
        def orientation(p, q, r):
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0  # colinear
            return 1 if val > 0 else 2  # clockwise or counterclockwise

        def on_segment(p, q, r):
            return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                    q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

        o1 = orientation(a, b, c)
        o2 = orientation(a, b, d)
        o3 = orientation(c, d, a)
        o4 = orientation(c, d, b)

        if (o1 != o2 and o3 != o4) or \
            (o1 == 0 and on_segment(a, c, b)) or \
            (o2 == 0 and on_segment(a, d, b)) or \
            (o3 == 0 and on_segment(c, a, d)) or \
            (o4 == 0 and on_segment(c, b, d)):
                return True
        return False

    def checkPathCollision(self, path1, path2):
        for i in range(len(path1)-1):
            for j in range(len(path2)-1):
                if self.checkLineCollision(path1[i], path1[i+1], path2[j], path2[j+1]):
                    return j
        return -1
    
    def adjustPath(self, robot, plannedRobots):
        rospy.loginfo("Adjusting path for robot "+str(robot))
        path = self.gridPaths[robot-1]
        self.gridPaths[robot-1] = []
        goal=self.goalPoints[robot][2]
        pos=Point()
        pos.x=self.px[robot-1]
        pos.y=self.py[robot-1]
        start=PathPlanner.world_to_grid(self.baseCspace,pos)
        pathstosend =[[], [], [], [], [], [], [], []]
        for i in plannedRobots:
            pathstosend[i-1] = self.gridPaths[i-1]
        newPath = PathPlanner.a_star(self.baseCspace,start,goal,otherPaths=self.gridPaths)
        if newPath == []:
            rospy.loginfo("No path found for robot "+str(robot))
            newPath=path
        self.gridPaths[robot-1] = newPath
        path_msg = PathPlanner.path_to_message(self.cspaced, newPath)
        self.pathMessages[robot-1] = path_msg
    
    def adjustPaths(self):
        plannedRobots = []
        plannedRobots.append(self.unplannedRobots.get()[0])
        while not self.unplannedRobots.empty():
            robot = self.unplannedRobots.get()
            currentBot = robot[0]
            # self.adjustPath(currentBot, plannedRobots)
            for i in range(len(plannedRobots)):
                plannedRobot = plannedRobots[i]
                collIndex = self.checkPathCollision(self.gridPaths[currentBot-1], self.gridPaths[plannedRobot-1])
                if collIndex > 0:
                    self.adjustPath(currentBot, plannedRobots)
                    i -= 1
                    break
            #     path_msg = PathPlanner.path_to_message(self.cspaced, self.gridPaths[currentBot-1])
            #     self.pathMessages[currentBot-1] = path_msg
            plannedRobots.append(currentBot)
        return plannedRobots
    
    def getUnassignedRobots(self):
        unassigned = []
        for i in range(0, self.numRobots):
            curLocation = [self.px[i], self.py[i]]
            goalLocation = self.goalPoints[i+1][2]
            if not (math.dist(curLocation, goalLocation) < .4) or goalLocation == (0,0):
                unassigned.append(i+1)
        return unassigned

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        # print("starting to choose 1")
        # self.chooseGoal(1,self.cspaced)
        # print("starting to choose 2")
        # self.chooseGoal(2,self.cspaced)
        rospy.spin()


        
if __name__ == '__main__':
    GlobalManager().run()