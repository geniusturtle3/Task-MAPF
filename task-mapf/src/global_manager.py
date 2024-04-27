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


class GlobalManager:
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
        self.goalPubs=[]
        self.numRobots = 8
        
        for i in range(1,self.numRobots+1):
            self.goalPubs.append(rospy.Publisher('/robot_'+str(i)+'/path'+str(i),Path,queue_size=10))
            rospy.Subscriber('/robot_'+str(i)+'/newGoal',Odometry,self.chooseGoal)
            rospy.Subscriber("/robot_"+str(i)+'/replan',Odometry,self.sameGoal)
            rospy.Subscriber('/robot_'+str(i)+'/reqedodom',Odometry,self.update_odometry)
            self.odomReqPubs.append(rospy.Publisher('/robot_'+str(i)+'/reqingodom',String,queue_size=10))

        

        # self.goals=[self.pathPub1,self.pathPub2,self.pathPub3,self.pathPub4,self.pathPub5,self.pathPub6,self.pathPub7,self.pathPub8]

        self.px=[0,0,0,0,0,0,0,0]
        self.py=[[0,0,0,0,0,0,0,0]]
        self.goalPoints = [[0,[0,0]],[0,[0,0]],[0,[0,0]],[0,[0,0]],[0,[0,0]],[0,[0,0]],[0,[0,0]],[0,[0,0]]]
        # self.paths = [None, None, None, None, None, None, None, None]
 

        self.opencells=[]
        for i in range(len(self.cspaced.data)):
            pnt=self.cspaced.data[i]
            if pnt<100 and pnt>-1:
                self.opencells.append(i)

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
        pos=Point()
        pos.x=self.px[robot-1]
        pos.y=self.py[robot-1]
        start=PathPlanner.world_to_grid(self.cspaced,pos)
        goalPoint=[]
        while len(goalPoint)==0:
            tempPoint=random.choice(self.opencells)
            # rospy.loginfo(str(robot)+" testing "+str(tempPoint))
            gridPoint=PathPlanner.index_to_grid(self.cspaced,tempPoint)
            path=PathPlanner.a_star(self.cspaced,start,gridPoint,self.gradSpace)
            if len(path)>0:
                goalPoint=gridPoint
                self.goalPoints[robot-1][0]=random.randint(50,100)
                self.goalPoints[robot-1][1]=goalPoint

        path_msg=PathPlanner.path_to_message(self.cspaced,path)
        
        rospy.loginfo("Done choosing goal for robot "+str(robot))
        # self.paths[robot-1] = path_msg
        self.goalPubs[robot-1].publish(path_msg)

    def sameGoal(self,msg):
        robot=self.update_odometry(msg)
        pos=Point()
        pos.x=self.px[robot-1]
        pos.y=self.py[robot-1]
        start=PathPlanner.world_to_grid(self.cspaced,pos)
        path=PathPlanner.a_star(self.cspaced,start,self.goalPoints[robot-1][1],self.gradSpace)
        path_msg=PathPlanner.path_to_message(self.cspaced,path)
        self.goalPubs[robot-1].publish(path_msg)
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
        rospy.spin()


        
if __name__ == '__main__':
    GlobalManager().run()