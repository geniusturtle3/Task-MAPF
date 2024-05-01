#!/usr/bin/env python3
from __future__ import annotations
import rospy
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class ScoreTracker:
    def __init__(self) -> None:
        rospy.init_node("score_tracker")
        self.numRobots = 9
        self.score = 0
        self.goals = {}
        self.px=[0 for i in range(self.numRobots)]
        self.py=[0 for i in range(self.numRobots)]

        for i in range(1, self.numRobots+1):
            rospy.Subscriber('/robot_'+str(i)+'/robotGoal', Float64MultiArray, self.goalChosen)
            rospy.Subscriber('/robot_'+str(i)+'/reqedodom', Odometry,self.update_odometry)

    def goalChosen(self, msg):
        data = msg.data
        self.goals[data[0]] = [data[1], [data[2], data[3]]]
        rospy.Timer(rospy.Duration(data[1]), callback=lambda event, id=data[0], duration=data[1]: self.updateScore(event, id, duration), oneshot=True)

    def updateScore(self, event, robot, duration):
        d = self.robotAtGoal(robot)
        if d < 0.5:
            self.score += ((10 / duration) + d)
        else:
            rospy.loginfo("Robot " + str(robot) + " did not reach goal in time.")
            self.score -= ((10 / duration) + d)
        rospy.loginfo("Score: " + str(self.score))

    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        number=0
        try:
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
        except:
            pass
        return number
    
    def robotAtGoal(self, robot):
        curLocation = [self.px[int(robot-1)], self.py[int(robot-1)]]
        goalLocation = self.goals[robot][1]
        return math.dist(curLocation, goalLocation)
    
    

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    ScoreTracker().run()