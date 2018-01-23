#!/usr/bin/env python

import rospy
from ihmc_msgs.msg import *
import time
import sys

if __name__ == '__main__':
    # Variables for neck position
    joint_1 = 0.0  # lowerlimit: 0.0, upperlimit: 1.162
    joint_2 = 0.0  # lowerlimit: -1.0472, upperlimit: 1.0472
    joint_3 = 0.0  # lowerlimit: -0.872, upperlimit: 0.0

    # No args zeros neck orientation
    if len(sys.argv) == 1:
        print "Setting to 0 orientation, use 3 parameters for roll, pitch, yaw"
    # Otherwise, sets user inputs to respective trajectories
    elif len(sys.argv) == 4:
        joint_1 = float(sys.argv[1])
        joint_2 = float(sys.argv[2])
        joint_3 = float(sys.argv[3])
    # End if user did not input right amount of parameters
    elif len(sys.argv) != 9:
        print "Please enter roll, pitch, and yaw coordinates or nothing to zero neck orientation"
        sys.exit()

    # Create a new publisher to publish on the neck_trajectory topic
    neck_trajectory_pub = rospy.Publisher("/ihmc_ros/valkyrie/control/neck_trajectory", NeckTrajectoryRosMessage, queue_size = 1)

    # Create a new node to move Valkyrie's neck
    rospy.init_node("control_neck")

    # Create a new msg to publish onto neck_trajectory
    # Similar to the arms, this msg sets neck joint_1/joint_2/joint_3 position to 0
    msg = NeckTrajectoryRosMessage( joint_trajectory_messages = [
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 0.0, position = joint_1, velocity = 0.3)]),
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 0.0, position = joint_2, velocity = 0.4)]),
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 0.0, position = joint_3, velocity = 0.5)]),
        ], unique_id = 13) # random number; just must not equal zero

    # Publish msg
    neck_trajectory_pub.publish(msg)
    time.sleep(1)
