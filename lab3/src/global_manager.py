#!/usr/bin/env python3
from __future__ import annotations
import rospy
import math
from std_srvs.srv import Empty
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

        self.goal1=rospy.Publisher('/robot_1/goal1',PoseStamped)
        self.pos1=rospy.Subscriber('/robot_1/odom',Odometry,self.update_odometry)

        self.goal2=rospy.Publisher('/robot_2/goal2',PoseStamped)
        self.pos2=rospy.Subscriber('/robot_2/odom',Odometry,self.update_odometry)
        
        self.goals=[self.goal1,self.goal2]
        self.pos=[self.pos1,self.pos2]

        self.px=[0,0]
        self.py=[0,0]
        
        

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

    def chooseGoal(self, robot: int, mapdata: OccupancyGrid):
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
        

        path_msg = PoseStamped()
        worldpoint=PathPlanner.grid_to_world(mapdata,goalPoint)

        path_msg.header=self.odomHeader
        path_msg.header.frame_id="map"
        path_msg.pose.position=worldpoint
        
        
        self.goals[robot-1].publish(path_msg)
        rospy.loginfo("Done calculating frontier path")




    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        self.map = PathPlanner.request_map()
        self.cspaced=PathPlanner.calc_cspace(self.map, 1.5)
        print("starting to choose 1")
        self.chooseGoal(1,self.cspaced)
        print("starting to choose 2")
        self.chooseGoal(2,self.cspaced)
        rospy.spin()


        
if __name__ == '__main__':
    GobalManager().run()