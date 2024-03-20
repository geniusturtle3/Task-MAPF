#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
from nav_msgs.srv import GetPlan

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel=rospy.Publisher('/cmd_vel',Twist)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom',Odometry,self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.execute_plan)

        self.px,self.py,self.ptheta=0,0,0

        
        

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
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service("plan_path")
        get_plan=rospy.ServiceProxy("plan_path", GetPlan)

        # we have to give it start and end in wordl coordinates
        startPose = PoseStamped()
        startPose.header.frame_id = "odom"
        startPose.pose.position.x = self.px
        startPose.pose.position.y = self.py

        planToDrive = get_plan(startPose, goal, 0.001).plan
        
        rospy.loginfo("Path received, executing driving instructions")
        i=0
        for pose in planToDrive.poses:
            last=False
            if i == len(planToDrive.poses)-1:
                last=True
            self.go_to(pose,last)
            i+=1        


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
        ### REQUIRED CREDIT
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
