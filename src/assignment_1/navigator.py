import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


"""
Class used to handle the navigation among the waipoints in order to cover all the map 
"""

class WaypointNavigator:
    def __init__(self, find_april_tags):
        # Initialize the MoveBaseAction client 
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Waypoints definition
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "yaw": -1.57},
            {"x": 7.5, "y": -1.0, "yaw": 0.0},
            {"x": 8.5, "y": -3.5, "yaw": 3.14},
            {"x": 12.5, "y": -3.5, "yaw": -0.8},
            {"x": 12.5, "y": 0.5, "yaw": 1.57},
            {"x": 10, "y": 0.6, "yaw": 3.14}
        ]
        
        self.find_april_tags = find_april_tags


    def navigate_to_waypoint(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        quaternion = self.yaw_to_quaternion(yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(f"Navigating to waypoint: x={x}, y={y}, yaw={yaw}")
        self.client.send_goal(goal)
        self.client.wait_for_result()


    def yaw_to_quaternion(self, yaw):
        return quaternion_from_euler(0, 0, yaw)


    def run(self):
    
        rospy.loginfo("Starting navigation...")
        for waypoint in self.waypoints:
            # Check wheather all target ids are been found
            if set(self.find_april_tags.getDetectedIds()) == set(self.find_april_tags.getTargetIds()):
                rospy.loginfo("All target IDs detected. Stopping navigation.")
                break

            # Naviga al waypoint corrente
            self.navigate_to_waypoint(waypoint["x"], waypoint["y"], waypoint["yaw"])

        # Stop of robot
        self.client.cancel_all_goals()
