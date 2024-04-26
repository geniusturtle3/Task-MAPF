#!/usr/bin/env python3
from __future__ import annotations
import rospy
import math
from std_srvs.srv import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, PointStamped 
from tf.transformations import euler_from_quaternion
import tf
import math
from nav_msgs.srv import GetPlan
import os
import copy

class PosReqer:
    def __init__(self) -> None:
        "Constructor"
        rospy.init_node("pos_reqer")
        name=rospy.get_name()
        # rospy.loginfo(str.split(str.split(name,'/')[1],'_')[1])
        self.number=int(str.split(str.split(name,'/')[1],'_')[1])
        rospy.Subscriber('/robot_'+str(self.number)+'/odom',Odometry,self.update_odometry)
        rospy.Subscriber('/robot_'+str(self.number)+'/reqingodom',String,self.getOdom)
        self.reqedOdomPub=rospy.Publisher('/robot_'+str(self.number)+'/reqedodom',Odometry,queue_size=0)

        self.goal = PoseStamped()
        self.goal.pose.position.x = 0
        self.goal.pose.position.y = 0
        self.goal.pose.position.z = 0
        self.goal.pose.orientation.x = 0
        self.goal.pose.orientation.y = 0
        self.goal.pose.orientation.z = 0
        self.goal.pose.orientation.w = 1
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.Time.now()
        self.prevOdom=Odometry()
        

    def update_odometry(self, data):
        self.goal.pose=copy.deepcopy(data.pose.pose)
        self.goal.header=data.header
        self.prevOdom=data
        
    def getOdom(self,msg):
        rospy.sleep(.01)
        self.reqedOdomPub.publish(self.prevOdom)
        return self.prevOdom
    
    def run(self):
        # Just needs to spin to keep the node alive - go_to is a callback bound to a Subscriber
        rospy.spin()
        

if __name__ == '__main__':
    PosReqer().run()
