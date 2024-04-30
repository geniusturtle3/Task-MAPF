from future import annotations
import rospy
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry

class ScoreTracker:
    def __init__(self) -> None:
        rospy.init_node("score_tracker")
        self.numRobots = 8
        self.scores = 0

        for i in range(self.numRobots):
            rospy.Subscriber(f"robot_{i}/newGoal", Odometry, self.goalCallback)

    def goalCallback(self, msg: Odometry) -> None:
        robotId = int(msg.header.frame_id.split("_")[1])
        if self.checkGoal(robotId):
            self.scores += 1
        self.publishScores()

    def checkGoal(self, robotId: int) -> bool:
        return True
    
    

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    ScoreTracker().run()