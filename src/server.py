#!/usr/bin/env python

import rospy
import actionlib
from assignment_1.search_ids import FindAprilTags
from assignment_1.navigator import WaypointNavigator
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == '__main__':
    try:
        # Initialize the ROS node_b_server (server node)
        rospy.init_node('node_b_server', anonymous = True)

        # Publisher per il controller del giunto della testa
        pub = rospy.Publisher('/head_trajectory_controller/command', JointTrajectory, queue_size=10)

        # Assicurati che il publisher sia pronto
        rospy.sleep(1)

        # Crea il messaggio JointTrajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['head_2_joint']  # Giunto per l'inclinazione verticale

        # Crea il punto della traiettoria
        point = JointTrajectoryPoint()
        point.positions = [-0.5]  # Angolo in radianti (negativo per inclinare verso il basso)
        point.time_from_start = rospy.Duration(2.0)  # Durata del movimento

        # Aggiungi il punto al messaggio
        trajectory_msg.points.append(point)

        # Pubblica il messaggio
        pub.publish(trajectory_msg)

        # Aspetta che il comando venga eseguito
        rospy.sleep(2)

        # Initialize the find apritags class
        find_april_tags = FindAprilTags('find_apriltags')

        # Initialize the navigator class
        navigator = WaypointNavigator(find_april_tags)
        navigator.run()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error! Node_b stopped!")




def tilt_camera_down():


if __name__ == '__main__':
    try:
        tilt_camera_down()
    except rospy.ROSInterruptException:
        pass

