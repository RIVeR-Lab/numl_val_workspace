#!/usr/bin/env python

import rospy
from ihmc_msgs.msg import *
import time
import sys

if __name__ == '__main__':

    # Determine whether the number of arguments passed is enough
    if len(sys.argv) == 1:
        print "One argument for which side, optional 7 additional arguments for joint control"
        sys.exit()
    # If only one arg passed (for robot side), zero arm orientation
    elif len(sys.argv) == 2:
        print "Sending arms to all 0 positions"
    # End if incorrect number of args passed
    elif len(sys.argv) != 9:
        print "Please start with either 1 (for robot side) or 8 arguments (for robot side, followed by position of each of the 7 arm joints)"
        sys.exit()

    # Get from input which side of the robot to control
    robot_side = int(sys.argv[1]) # 0 refers to the right side of the robot

    # Create a new node to control Valkyrie's arms
    rospy.init_node("control_arms")

    # Create a publisher to publish on the arm_trajectory topic
    pub = rospy.Publisher("/ihmc_ros/valkyrie/control/arm_trajectory", ArmTrajectoryRosMessage, queue_size=1)
    time.sleep(1)

    # Set the arm position using an ArmTrajectoryRosMessage
    # This message sets each arm joint to the desired position in 3 seconds
    msg = ArmTrajectoryRosMessage( joint_trajectory_messages = [
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 3.0, position = 0.0, velocity = 0.1)]),
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 3.0, position = 0.0, velocity = 0.1)]),
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 3.0, position = 0.0, velocity = 0.1)]),
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 3.0, position = 0.0, velocity = 0.1)]),
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 3.0, position = 0.0, velocity = 0.1)]),
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 3.0, position = 0.0, velocity = 0.1)]),
        OneDoFJointTrajectoryRosMessage(trajectory_points = [ TrajectoryPoint1DRosMessage(time= 3.0, position = 0.0, velocity = 0.1)])
        ], execution_mode = 0, unique_id = 13) # random number; just must not be zero
    msg.robot_side = robot_side

    # If all arguments valid, set the position of the respective joint to its respective argument
    if len(sys.argv) == 9:
        for i in range(2,9):
            msg.joint_trajectory_messages[i-2].trajectory_points[0].position = float(sys.argv[i])

    # Publish msg
    pub.publish(msg)
    time.sleep(1)
