#!/usr/bin/env python3
from __future__ import annotations
import rospy
import math
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, PointStamped 
from tf.transformations import euler_from_quaternion
import tf
import math
from nav_msgs.srv import GetPlan
import os
import copy

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        
        # rospy.loginfo(self.number)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        name=rospy.get_name()
        rospy.loginfo(str.split(str.split(name,'/')[1],'_')[1])
        self.number=int(str.split(str.split(name,'/')[1],'_')[1])
       
        self.cmd_vel=rospy.Publisher('/robot_'+str(self.number)+'/cmd_vel',Twist)
        rospy.Subscriber('/robot_'+str(self.number)+'/odom',Odometry,self.update_odometry)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.execute_plan)
        rospy.Subscriber('/robot_'+str(self.number)+'/goal'+str(self.number),PoseStamped,self.execute_plan)
        self.reqCount=0
        # elif rospy.get_name()=="/robot_2/pathfollow":
        #     self.cmd_vel=rospy.Publisher('/robot_2/cmd_vel',Twist)
        #     rospy.Subscriber('/robot_2/odom',Odometry,self.update_odometry)
        #     rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.execute_plan)
        self.px,self.py,self.ptheta=0,0,0

        self.pxStart, self.pyStart, self.pthetaStart = 0, 0, 0

        self.useOdom = True
        # a value of 0.45 meters seem to smooth out things while keeping the robot fairly on track
        self.indexLookahead = 12  # 6 cells lookahead
        self.lookaheadDistance = self.indexLookahead*0.015  # meters
        self.maximumVelocity = 0.4  # meters per second
        self.maximumAngVelocity = 5.5  # rad per second
        self.turnK=1.97
        self.maximumAngAccel= math.pi * 0.8
        self.updateOdom = True
        self.isLocalized = False
        self.prevError = 0

        
        

        # pass # delete this when you implement your code



    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### Make a new Twist message
        msg_cmd_vel = Twist()

        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0

        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed
        ### Publish the message
        self.cmd_vel.publish(msg_cmd_vel)
        
    
        
    def drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        if distance < 0:
            linear_speed = -linear_speed
            distance = -distance
        pose_init = (self.px, self.py)
        drive_tolerance = 0.01
        while self.pose_distance(pose_init) < abs(distance - drive_tolerance):
            self.send_speed(linear_speed, 0.0)
            rospy.sleep(0.05)
        self.send_speed(0.0, 0.0)
    
    def rotate(self, angle: float, angular_speed: float):
        stop_error = 0.005   
        update_time = 0.033 # [s]
        initial_pose = (self.px, self.py, self.ptheta)

        
        # take care of large angle
        while angle >= 2*math.pi:
            angle -= 2*math.pi
        while angle <= -2*math.pi:
            angle += 2*math.pi

        #make all turns smaller than pi
        if angle > math.pi:
            angle = angle - 2*math.pi
        elif angle < -math.pi:
            angle = angle + 2*math.pi
        
        # take care of the sign of the angle in real world
        if angle < 0:
            max_accel = -max_accel  # this effectively swaps the speed sign
            # aspeed = -aspeed

        #print(angle*180/math.pi)
        angle = abs(angle)   

        # take care of the sign of the angle
        if angle < 0:
            aspeed = -aspeed   

        dist_traveled = 0
        while(dist_traveled < (angle - stop_error)):               
            self.send_speed(0.0, aspeed)
            dist_traveled = abs((self.ptheta - initial_pose[2]))
            if(dist_traveled > math.pi):
                dist_traveled = 2*math.pi - dist_traveled
            #print(dist_traveled*180/math.pi)
            rospy.sleep(update_time)
        
        self.send_speed(0.0, 0.0) # stop the robot

    def execute_plan(self, goal: PoseStamped):
        self.reqCount+=1
        print(self.reqCount%2,self.number)
        if True:#self.reqCount%2==self.number%2:
            rospy.loginfo("Requesting the map")
            rospy.wait_for_service("/plan_path")
            # get_plan=rospy.ServiceProxy("/plan_path", GetPlan)

            tolerance = 0.2
            while self.pose_distance((goal.pose.position.x, goal.pose.position.y)) > tolerance * 1.05:
                get_plan = rospy.ServiceProxy("/plan_path", GetPlan)

                # we have to give it start and end in wordl coordinates
                startPose = PoseStamped()
                startPose.header.frame_id = "map"
                startPose.pose.position.x = self.px
                startPose.pose.position.y = self.py

                planToDrive: Path = get_plan(startPose, goal, 0.001).plan

                rospy.loginfo("Path received, executing driving instructions")

                # Execute the path        
                self.pure_pursuit(planToDrive, tolerance=tolerance)
            rospy.loginfo("Finished driving to goal")
        # # we have to give it start and end in wordl coordinates
        # startPose = PoseStamped()
        # startPose.header.frame_id = "/robot"+str(self.number)+'_tf/odom'
        # startPose.pose.position.x = self.px
        # startPose.pose.position.y = self.py
        # # try:
        # #     rospy.loginfo(rospy.get_param('/robot_1/pathfollow/robottf'))
        # # except:
        # #     pass
        # # print(rospy.get_namespace)
        # rospy.loginfo(startPose)
        # planToDrive = get_plan(startPose, goal, 0.001).plan
        
        # rospy.loginfo("Path received, executing driving instructions")
        # i=0
        # for pose in planToDrive.poses:
        #     last=False
        #     if i == len(planToDrive.poses)-1:
        #         last=True
        #     self.go_to(pose,last)
        #     i+=1        

    def pure_pursuit(self, path: Path, tolerance: float = 0.14, earlyExit: bool = False):
        """
        Uses the pure pursuit algorithm to follow a path.
        :param path [Path] The path to follow.
        :param tolerance [float] The range of acceptable error for the robot to 
                         be considered at the final waypoint. Default of 2.5 cm
        """
        prevLookaheadIndex = 0

        # if a path is too short for this algorithm, use simpler algorithm
        if len(path.poses) == 0:
            return
        if len(path.poses) < 2:
            rospy.loginfo(
                "Path too short for pure pursuit, using simple path follow")
            self.go_to(path.poses[0])
            if len(path.poses) > 1:
                self.go_to(path.poses[1])
            return

        self.indexLookahead = min(self.indexLookahead, len(path.poses)-1)

        initialHeading = math.atan2(path.poses[self.indexLookahead].pose.position.y - path.poses[0].pose.position.y,
                                    path.poses[self.indexLookahead].pose.position.x - path.poses[0].pose.position.x)
        print(f"initial heading {initialHeading} and self theta {self.ptheta}")
        diff = initialHeading-self.ptheta
        # only rotate if the difference is large-ish
        if abs(diff) > 0.1:
            self.smooth_rotate(initialHeading-self.ptheta,
                               self.maximumAngVelocity)
            
        isDone = False
    
        while True:
            # Calls findLookaheadPoint function to find the desired lookahead
            #   waypoint and saves it to chosenWaypoint.
            # prevLookaheadIndex = self.findLookaheadPoint(
            #     path, prevLookaheadIndex)
            closestPoint = self.findClosestPoint(path)
            prevLookaheadIndex = min(self.findClosestPoint(
                path) + self.indexLookahead, len(path.poses)-1)
            chosenWaypoint: PoseStamped = path.poses[prevLookaheadIndex]

            # Finds position of waypoint and robot respectively and separates them into X and Y components
            waypointXPos, waypointYPos = chosenWaypoint.pose.position.x, \
                chosenWaypoint.pose.position.y    # Waypoint

            # Calculates the angle in which the robot needs to travel, then finds the distance between this
            #   and the robot heading.
            angleOfLookaheadVector = math.atan2(
                (waypointYPos - self.py), (waypointXPos - self.px))
            angleBetween = angleOfLookaheadVector - self.ptheta

            # Finds the overall distance from the robot to the waypoint, then the horizontal distance between
            #   the lookahead vector and the waypoint.
            distToWaypoint = self.pose_distance(
                (chosenWaypoint.pose.position.x, chosenWaypoint.pose.position.y))
            horizontalDistToWaypoint = math.sin(angleBetween)*distToWaypoint

            if self.pose_distance((path.poses[closestPoint].pose.position.x, path.poses[closestPoint].pose.position.y)) > 2*tolerance:
                rospy.logwarn("ERROR: Robot lost path - Emergency Stopping")
                self.send_speed(0, 0)
                return

            # This represents the case where the robot is either on the lookahead point (last point)
            #   or is already at the right heading and doesn't need to curve.
            if horizontalDistToWaypoint == 0:
                curvature = 0

            # If the robot needs to curve:
            else:
                # Radius of curvature from robot to point.
                radiusOfCurvature = (distToWaypoint)**2 / \
                    (2*horizontalDistToWaypoint)

                # Curvature from robot to point.
                curvature = 1/radiusOfCurvature

            # Constant used to scale overall velocity up or down for tuning.
            kp = .35

            # Slows robot down around curves.
            kd = 0.4*(abs(curvature))

            # Calculates desired overall robot velocity
            error = (distToWaypoint/self.lookaheadDistance) * \
                self.maximumVelocity
            # error keeps jumping a bit too much

            errorDiff = error - self.prevError
            if errorDiff > 0:
                errorDiff = -0.03
                # this happens when we jump to a next lookahead point
                # we just reuse the last error in that case
            if errorDiff < -0.1:
                errorDiff = -0.1

            # print(f"KP: {kp*error :2.4f}\t KD: {kd*errorDiff :2.4f} \t C: {curvature:2.4f}")
            # TODO: Implement smooth acceleration / deceleration
            linearVelocity = kp*error + kd*errorDiff

            # clamp the velocity to the maximum and minimum
            if linearVelocity > self.maximumVelocity:
                linearVelocity = self.maximumVelocity

            if linearVelocity < 0.01:
                linearVelocity = 0.01

            self.prevError = error

            # Curvature to robot linear and angular velocities.
            # for velocity of 0.22 the coeff is 1.35
            angularVelocity = curvature * linearVelocity*self.turnK

            if angularVelocity > self.maximumAngVelocity:
                angularVelocity = self.maximumAngVelocity
                linearVelocity = angularVelocity / curvature

            # print(f"Linear velocity: {linearVelocity}, Angular velocity: {angularVelocity}")
            # print(f"Curvature: {curvature}, Error: {error}")

            self.send_speed(linearVelocity, angularVelocity)

            if earlyExit:
                # early exit w/o stopping anything after 2 cells
                # TODO figure out a good early-exit condition

                lookahead = min(120, len(path.poses)-1)

                isDone = self.pose_distance(
                    (path.poses[lookahead].pose.position.x, path.poses[lookahead].pose.position.y)) < tolerance
            if not isDone:  # and earlyExit:
                isDone = self.pose_distance(
                    (path.poses[-1].pose.position.x, path.poses[-1].pose.position.y)) < tolerance
            # elif not isDone:
            #     # Tells the system that the PP loop is done.
            #     isDone = distToWaypoint < tolerance and chosenWaypoint == path.poses[-1]

            if isDone:
                # self.send_speed(0, 0)
                # Stops the robot and breaks the loop.

                self.send_speed(0, 0)
                return

            self.lastPPseconds = rospy.get_time()
            self.updateOdom = False
            # 20Hz is the frequency
            while not self.updateOdom and (rospy.get_time() - self.lastPPseconds) < 0.25:
                rospy.sleep(0.005)
            rospy.sleep(0.005)

    def findLookaheadPoint(self, path: Path, prevLookaheadIndex: int) -> int:
        poseList = path.poses
        pointList = [(0, 0)]*len(poseList)
        for i in range(len(poseList)):
            pointList[i] = (poseList[i].pose.position.x,
                            poseList[i].pose.position.y)
        # Sets current lookahead point to previous for the
        indexOfLookaheadPoint = prevLookaheadIndex
        #   start of the next loop.
        # Initial value of the lookahead point distance is 0.
        lookaheadPointDist = 0

        # Looks at waypoints from the last lookahead point to the second to last point on the list
        for i in range(prevLookaheadIndex, len(pointList)):
            # Finds position of the waypoint currently being
            pointPosition = pointList[i]
            #   looked at.

            # Finds distance from robot to waypoint
            pointDistance = self.pose_distance(pointPosition)

            if pointDistance > lookaheadPointDist and pointDistance < self.lookaheadDistance:
                # If the distance to the closest waypoint is further than the current lookahead point distance and shorter than the ideal
                #   lookahead distance, it does the following. Basically, we want to find a waypoint as close to the lookahead distance as
                #   possible, but will ALWAYS round down if possible.
                # Sets the index of the lookahead point to i.
                indexOfLookaheadPoint = i
                # Sets the distance to the lookahead point distance so we calculate
                lookaheadPointDist = pointDistance
                # the wheel velocities based on the target waypoint.
            # If the distance of the new closest waypointis further than the lookahead distance, do the following.
            elif pointDistance > lookaheadPointDist:
                # Sets the lookahead index to be one option before the
                indexOfLookaheadPoint = i - 1
                #   current index. (Rounds down.)
                return indexOfLookaheadPoint
        # If we make it through the entire list without finding a point that is closer than the lookahead distance,
        # we set the lookahead point to the last point in the list.
        return indexOfLookaheadPoint

    def findClosestPoint(self, path: Path) -> int:
        """
        Finds closest point for use with Stanley Control
        :param path [Path] The path to follow.
        :returns the index of the closest point
        """
        closestIndex = 0
        previousPointDistance = 69420
        for i in range(len(path.poses)):
            pointPosition = path.poses[i].pose.position.x, path.poses[i].pose.position.y
            # robotPosition = (self.px, self.py)
            pointDistance = self.pose_distance(pointPosition)

            if pointDistance < previousPointDistance:
                # Iterates until the distance increases, then returns
                previousPointDistance = pointDistance
                closestIndex = i
            else:
                # Return the index instead of the coordinates so we can get the heading later
                return closestIndex
        # If we run out of points, use the last valid one
        return len(path.poses)-1


    def go_to(self, msg: PoseStamped, rotEnd=False):
        """
        Attains a given pose by using the three step rotate-drive-rotate method. 
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ###Rot Drv Rot 
        #using the rotate-drive-rotate method here
        turn_speed = 3.3
        wait_time = 0.05
        drive_speed = 0.25
        #extract the final pose(x, y, theta) from msg
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        #calculate the rotation
        angle_to_rotate = math.atan2(goal_y - self.py, goal_x - self.px) - self.ptheta
        self.smooth_rotate(angle_to_rotate, turn_speed)
        rospy.sleep(wait_time)

        #calculate the pythagorean distance to drive
        distance_to_drive = ((goal_x - self.px)**2 + (goal_y - self.py)**2)**0.5
        self.smooth_drive(distance_to_drive, drive_speed)
        
        if rotEnd:
            rospy.sleep(wait_time)
            
            #calculate the angle to rotate to align with the goal
            quat_RAW = msg.pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion([quat_RAW.x, quat_RAW.y, quat_RAW.z, quat_RAW.w])
            goal_theta = yaw
            angle_to_rotate_goal = goal_theta - self.ptheta
            self.smooth_rotate(angle_to_rotate_goal, turn_speed)

    def go_to_ICC(self, msg: PoseStamped):
        """
        Attains a given pose by using arc curvature. 
        Used https://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm for reference.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        init_pose = (self.px, self.py, self.ptheta)
        target_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.z)
        

        # Distance in angle between robot heading and straight line to target
        delta_theta = math.atan2(target_pose[1]-init_pose[1], target_pose[0]-init_pose[0]) - init_pose[2]
        # Radius of arc that intersects current and final position, accounting for initial heading
        arc_radius = math.sqrt((target_pose[0]-init_pose[0])**2 + (target_pose[1]-init_pose[1])**2) / (2 * math.sin(delta_theta))

        tolerance = 0.05
        vel = 0.1
        max_vel = 0.2
        max_accel = 0.05
        update_time = 0.05
        is_decelerating = False
        
        print(f"Going from {init_pose} to {target_pose}; Arc radius: {arc_radius}")
        while self.pose_distance(target_pose) > tolerance:

            # Handle acceleration and deceleration the same way as in smooth_drive
            if not is_decelerating:
                if vel < max_vel:
                    vel += max_accel * update_time
                
                distance_to_switch = (vel)**2/(2*max_accel) * 1.07
                # distance remaining < distance to switch
                if (self.pose_distance(target_pose)) < distance_to_switch:
                    is_decelerating = True
            
            elif is_decelerating and vel > 0.005:
                vel -= max_accel * update_time

            # Angular velocity is the ratio of linear velocity to arc radius
            ang_vel = vel / arc_radius
            # Debug print statement
            # print(f"Vel: {vel}, Ang Vel: {ang_vel}")
            self.send_speed(vel, ang_vel)
            rospy.sleep(0.05)
        self.rotate(target_pose[2]-self.ptheta, 0.1)

    def go_to_2002(self, msg: PoseStamped, rotEnd=False):
        """
        Attains a given pose by using the three step rotate-drive-rotate method. 
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """

        ### 2002 Method Not usued
        kpdis=.3
        kpang=.6
        eightTenPi=.8*math.pi
        twopi=2*math.pi
        
        while(abs(msg.pose.position.x-self.px)+abs(msg.pose.position.y-self.py)>.05):
            print('drive2')
            disError=math.sqrt(math.pow(msg.pose.position.x-self.px,2)+math.pow(msg.pose.position.y-self.py,2))
            angError=math.atan2(msg.pose.position.y-self.py,msg.pose.position.x-self.px)-self.ptheta
            # angError=((yaw-self.pth%twopi)%twopi+math.pi)%(2*math.pi)-math.pi
            if angError>eightTenPi:
                angError-=twopi
            elif angError<-eightTenPi:
                angError+=twopi
            # print(self.px,self.py,disError,angError*180/math.pi)
            dspeed,aspeed=disError*kpdis,angError*kpang
            self.send_speed(dspeed,aspeed)
            if abs (aspeed)>1:
                aspeed=1
            if dspeed>.2:
                dspeed=.2
            rospy.sleep(.01)
        if rotEnd:
            (roll,pitch,yaw)=euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
            self.smooth_rotate(yaw-self.ptheta,.3)   
             
    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # print("updating odom")
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w])
        self.ptheta = yaw


    def smooth_drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance:     [float] [m]   The distance to cover.
        :param linear_speed: [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # it shall be a trapezoidal profile generator
        stop_error = 0.02   # 2cm
        update_time = 0.05  # [s]
        initial_pose = (self.px, self.py, self.ptheta)

        max_accel = 0.05     # [m/s^2]
        current_speed = 0.0 # [m/s]
        is_decelerating = False
        
        # simplified version
        while(self.pose_distance(initial_pose) < (distance - stop_error)):
            #accelerate until max speed is reached or it is time to decelerate
            if not is_decelerating:
                if current_speed < linear_speed:
                    current_speed += max_accel * update_time
                
                distance_to_switch = (current_speed)**2/(2*max_accel) * 1.07
                # distance remaining < distance to switch
                if (distance - self.pose_distance(initial_pose)) < distance_to_switch:
                    is_decelerating = True
            
            elif is_decelerating and current_speed > 0.005:
                current_speed -= max_accel * update_time

            self.send_speed(current_speed, 0.0)
            #print(self.pose_distance(initial_pose))
            rospy.sleep(update_time)
        
        self.send_speed(0.0, 0.0) # stop the robot

    def smooth_rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        stop_error = 0.005   
        update_time = 0.05 # [s]
        initial_pose = (self.px, self.py, self.ptheta)

        max_accel = 0.25    # [rad/s^2]
        current_speed = 0.0 # [rad/s]
        is_decelerating = False


        # take care of large angle
        while angle >= 2*math.pi:
            angle -= 2*math.pi
        while angle <= -2*math.pi:
            angle += 2*math.pi

        #make all turns smaller than pi
        if angle > math.pi:
            angle = angle - 2*math.pi
        elif angle < -math.pi:
            angle = angle + 2*math.pi
        
        # take care of the sign of the angle in real world
        if angle < 0:
            max_accel = -max_accel  # this effectively swaps the speed sign
            # aspeed = -aspeed

        #print(angle*180/math.pi)
        angle = abs(angle)

        # now it is safe to assume the robot will never have to spin more than pi
        dist_traveled = 0
        while(dist_traveled < (angle - stop_error)):                 
            #accelerate until max speed is reached or it is time to decelerate
            if not is_decelerating:
                if abs(current_speed) < abs(aspeed):
                    current_speed += max_accel * update_time
                
                distance_to_switch = abs((current_speed)**2/(2*max_accel)) * 1.05
                # distance traveled > distance remaining
                if dist_traveled > angle - distance_to_switch:
                    is_decelerating = True
            
            elif is_decelerating and abs(current_speed) > 0.05:
                current_speed -= max_accel * update_time
            
             

            self.send_speed(0.0,current_speed)
            dist_traveled = abs((self.ptheta - initial_pose[2]))
            if(dist_traveled > math.pi):
                dist_traveled = 2*math.pi - dist_traveled
            #print(dist_traveled*180/math.pi)
            rospy.sleep(update_time)

        self.send_speed(0.0, 0.0) # stop the robot


    
    def pose_distance(self, position:tuple,):
        """
        :param position [tuple] - A pose(x [m], y [m], theta [rad]) 
        :returns the distance between the two points
        """
        return ((position[0] - self.px)**2 + (position[1] - self.py)**2)**0.5


    def run(self):
        # Just needs to spin to keep the node alive - go_to is a callback bound to a Subscriber
        rospy.spin()
        

if __name__ == '__main__':
    Lab2().run()
