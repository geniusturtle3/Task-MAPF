#!/usr/bin/env python3
from __future__ import annotations
import math
import numpy as np
import cv2
import copy
import rospy
import std_srvs
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path,Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from priority_queue import PriorityQueue
import path_follower

class localizer(path_follower):
    def __init__(self) -> None:
        super().__init__()
        
        # rospy.wait_for_service('global_localization')
        # self.amclinit=rospy.ServiceProxy('global_localization',std_srvs._empty_)
        rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,self.update_AMCL)
        self.uncertainty=0


    def update_AMCL(self,amcl_pose: PoseWithCovarianceStamped):
        if not self.useOdom:
            self.px = amcl_pose.pose.position.x
            self.py = amcl_pose.pose.position.y
            quat_orig = amcl_pose.pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion(
                [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w])
            self.ptheta = yaw

            self.covariance = amcl_pose.pose.covariance

    def run(self):
        # Run the node
        super().smooth_rotate(math.pi,2)
        self.useOdom=False
        rospy.loginfo("Started Localizer")
        rospy.spin()


if __name__ == '__main__':
    localizer().run()



