#!/usr/bin/env python

import rospy
import actionlib
from assignment_1.search_ids import FindAprilTags
from assignment_1.navigator import WaypointNavigator


if __name__ == '__main__':
    try:
        # Initialize the ROS node_b_server (server node)
        rospy.init_node('node_b_server', anonymous = True)

        # Initialize the find apritags class
        find_april_tags = FindAprilTags('find_apriltags')

        # Initialize the navigator class
        navigator = WaypointNavigator(find_april_tags)
        navigator.run()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error! Node_b stopped!")


