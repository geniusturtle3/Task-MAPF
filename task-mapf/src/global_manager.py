#!/usr/bin/env python3
from __future__ import annotations
import rospy
import math
from std_srvs.srv import Empty
from std_msgs.msg import String
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
import numpy as np
from nav_msgs.srv import GetPlan
from path_planner import PathPlanner
from priority_queue import PriorityQueue


class GobalManager:
    def __init__(self) -> None:
        "Constructor"

        rospy.init_node("global_manager")

        self.map = PathPlanner.request_map()
        self.cspaced=PathPlanner.calc_cspace(self.map, 1.5)
        self.gradSpace=PathPlanner.calc_gradspace(self.map)

        self.a_star_pub = rospy.Publisher("/a_star", GridCells, queue_size=0)
        self.path_pub = rospy.Publisher("/apath", Path, queue_size=0)
        self.cspace_pub = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=0)

        self.odomReqPubs=[]
        self.goals=[]
        self.numRobots=8
        self.priorityMultiplier = 50
        self.priorityOffset = 5*self.priorityMultiplier
        
        for i in range(1,9):
            self.goals.append(rospy.Publisher('/robot_'+str(i)+'/path'+str(i),Path,queue_size=10))
            rospy.Subscriber('/robot_'+str(i)+'/newGoal',Odometry,self.chooseGoal)
            rospy.Subscriber("/robot_"+str(i)+'/replan',Odometry,self.sameGoal)
            rospy.Subscriber('/robot_'+str(i)+'/reqedodom',Odometry,self.update_odometry)
            self.odomReqPubs.append(rospy.Publisher('/robot_'+str(i)+'/reqingodom',String,queue_size=10))

        

        # self.goals=[self.pathPub1,self.pathPub2,self.pathPub3,self.pathPub4,self.pathPub5,self.pathPub6,self.pathPub7,self.pathPub8]

        self.px=[0,0,0,0,0,0,0,0]
        self.py=[0,0,0,0,0,0,0,0]
        self.paths = [None, None, None, None, None, None, None, None]
        self.goalPoints={}
        self.isPlanned = [False, False, False, False, False, False, False, False]
        self.unplannedRobots = PriorityQueue()

        rospy.loginfo("publishing cspace")
        cspace = GridCells()
        cspace.header.frame_id = "map"
        cspace.cell_width = self.map.info.resolution
        cspace.cell_height = self.map.info.resolution
        cspace.cells = []

        for i in range(len(self.cspaced.data)):
            if self.cspaced.data[i] == 100:
                # Publish only the inflated cells
                cspace.cells.append(PathPlanner.grid_to_world(
                    self.cspaced, (i % self.cspaced.info.width, int(i / self.cspaced.info.width))))

        self.cspace_pub.publish(cspace)

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

    def chooseGoal(self, msg):
        robot=self.update_odometry(msg)
        self.callUpdateAllPoses(robot)
        mapdata=self.cspaced
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
            path=PathPlanner.a_star(mapdata,start,gridPoint,self.gradSpace)
            if len(path)>0:
                goalPoint=gridPoint
                goalTime = random.randint(50,200)
                self.goalPoints[robot] = (robot, goalTime, goalPoint)
        

        # path_msg = PoseStamped()
        # worldpoint=PathPlanner.grid_to_world(mapdata,goalPoint)

        # path_msg.header=self.odomHeader
        # path_msg.header.frame_id="map"
        # path_msg.pose.position=worldpoint
        path_msg=PathPlanner.path_to_message(self.cspaced,path)
        self.paths[robot-1] = path_msg
        self.isPlanned[robot-1] = True
        
        rospy.loginfo("Done choosing goal for robot "+str(robot))

        if self.allGoalsChosen():
            rospy.loginfo("All goals chosen")
            rospy.loginfo(str(self.goalPoints))
            rospy.loginfo(str(self.isPlanned))
            self.prioritizeRobots()

    def allGoalsChosen(self):
        return all(self.isPlanned)
    
    def prioritizeRobots(self):
        for key, value in self.goalPoints.items():
            path = self.paths[key-1]
            path_length = PathPlanner.path_length(self.cspaced,path)
            priority = self.priorityMultiplier*path_length
            if priority > value[1]:
                priority += self.priorityOffset
            self.unplannedRobots.put(value, priority)
        self.releaseRobots()

    def releaseRobots(self):
        while not self.unplannedRobots.empty():
            robotInfo = self.unplannedRobots.get()
            robotNum = robotInfo[0]
            path_msg = self.paths[robotNum-1]
            self.goals[robotNum-1].publish(path_msg)
            self.path_pub.publish(path_msg)
        self.isPlanned = [False, False, False, False, False, False, False, False]

    def sameGoal(self,msg):
        mapdata=self.cspaced
        robot=self.update_odometry(msg)
        pos=Point()
        pos.x=self.px[robot-1]
        pos.y=self.py[robot-1]
        start=PathPlanner.world_to_grid(mapdata,pos)
        path=PathPlanner.a_star(mapdata,start,self.goalPoints[robot-1][2],self.gradSpace)
        path_msg=PathPlanner.path_to_message(self.cspaced,path)
        self.goals[robot-1].publish(path_msg)
        self.path_pub.publish(path_msg)

    def callUpdateAllPoses(self,robot):      
        for i in range(1,self.numRobots+1):
            if i!=robot:
                #requesting
                # rospy.loginfo(str((self.px[i-1],self.py[i-1])))
                msg=String()
                msg.data="requesting_"+str(i)
                self.odomReqPubs[i-1].publish(msg)
                rospy.wait_for_message('/robot_'+str(i)+'/reqedodom',Odometry)
                # rospy.loginfo(str((self.px[i-1],self.py[i-1])))
                # rospy.loginfo("received_"+str(i))
            
            
        



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
    GobalManager().run()