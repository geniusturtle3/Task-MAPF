from future import annotations
import rospy

class ScoreTracker:
    def __init__(self) -> None:
        rospy.init_node("score_tracker")
        self.numRobots = 8
        

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    ScoreTracker().run()