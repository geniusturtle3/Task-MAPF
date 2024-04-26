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


class GobalManager:
    def __init__(self) -> None:
        "Constructor"

        rospy.init_node("global_manager")

        self.map = PathPlanner.request_map()
        self.cspaced=PathPlanner.calc_cspace(self.map, 1.5)
        self.gradSpace=PathPlanner.calc_gradspace(self.map)

        self.a_star_pub = rospy.Publisher("/a_star", GridCells, queue_size=10)
        self.path_pub = rospy.Publisher("/apath", Path, queue_size=10)
        self.cspace_pub = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=10)

        self.pathPub1=rospy.Publisher('/robot_1/path1',Path,queue_size=10)
        self.status1=rospy.Subscriber('/robot_1/newGoal',Odometry,self.chooseGoal)
        self.replan1=rospy.Subscriber('/robot_1/replan',Odometry,self.sameGoal)

        self.pathPub2=rospy.Publisher('/robot_2/path2',Path,queue_size=10)
        self.status2=rospy.Subscriber('/robot_2/newGoal',Odometry,self.chooseGoal)
        self.replan2=rospy.Subscriber('/robot_2/replan',Odometry,self.sameGoal)

        self.pathPub3=rospy.Publisher('/robot_3/path3',Path,queue_size=10)
        self.status3=rospy.Subscriber('/robot_3/newGoal',Odometry,self.chooseGoal)
        self.replan3=rospy.Subscriber('/robot_3/replan',Odometry,self.sameGoal)
        
        self.pathPub4=rospy.Publisher('/robot_4/path4',Path,queue_size=10)
        self.status4=rospy.Subscriber('/robot_4/newGoal',Odometry,self.chooseGoal)
        self.replan4=rospy.Subscriber('/robot_4/replan',Odometry,self.sameGoal)
        
        self.pathPub5=rospy.Publisher('/robot_5/path5',Path,queue_size=10)
        self.status5=rospy.Subscriber('/robot_5/newGoal',Odometry,self.chooseGoal)
        self.replan5=rospy.Subscriber('/robot_5/replan',Odometry,self.sameGoal)


        self.goals=[self.pathPub1,self.pathPub2,self.pathPub3,self.pathPub4,self.pathPub5]

        self.px=[0,0,0,0,0]
        self.py=[0,0,0,0,0]
        self.goalPoints=[[0,0],[0,0],[0,0],[0,0],[0,0]]

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
            # rospy.loginfo(str(robot)+" testing "+str(tempPoint))
            gridPoint=PathPlanner.index_to_grid(mapdata,tempPoint)
            path=PathPlanner.a_star(mapdata,start,gridPoint,self.gradSpace)
            if len(path)>0:
                goalPoint=gridPoint
                self.goalPoints[robot-1]=goalPoint
        

        # path_msg = PoseStamped()
        # worldpoint=PathPlanner.grid_to_world(mapdata,goalPoint)

        # path_msg.header=self.odomHeader
        # path_msg.header.frame_id="map"
        # path_msg.pose.position=worldpoint
        path_msg=PathPlanner.path_to_message(self.cspaced,path)
        self.goals[robot-1].publish(path_msg)
        self.path_pub.publish(path_msg)

        rospy.loginfo("Done choosing goal for robot "+str(robot))

    def sameGoal(self,msg):
        mapdata=self.cspaced
        robot=self.update_odometry(msg)
        pos=Point()
        pos.x=self.px[robot-1]
        pos.y=self.py[robot-1]
        start=PathPlanner.world_to_grid(mapdata,pos)
        path=PathPlanner.a_star(mapdata,start,self.goalPoints[robot-1],self.gradSpace)
        path_msg=PathPlanner.path_to_message(self.cspaced,path)
        self.goals[robot-1].publish(path_msg)
        self.path_pub.publish(path_msg)

    



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