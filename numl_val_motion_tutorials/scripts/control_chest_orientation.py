#!/usr/bin/env python

import rospy
from ihmc_msgs.msg import *
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import math
import sys
import tf
import time

if __name__ == '__main__':
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    # No arguments passed sets chest orientation to 0
    if len(sys.argv) == 1:
        print "Setting to 0 orientation, use 3 parameters for roll, pitch, yaw"

    # Otherwise, assign user inputs to respective trajectories
    elif len(sys.argv) == 4:
        roll = math.radians(float(sys.argv[1]))
        pitch = math.radians(float(sys.argv[2]))
        yaw = math.radians(float(sys.argv[3]))
    elif len(sys.argv) != 4:
        print "Please enter roll, pitch, and yaw coordinates or nothing for zero chest orientation"

    # Create a publisher to publish on the chest_trajectory topic
    chest_trajectory_pub = rospy.Publisher("/ihmc_ros/valkyrie/control/chest_trajectory",
                                           ChestTrajectoryRosMessage, queue_size=1)

    # Create a new node to control the chest
    rospy.init_node("control_chest_orientation")

    # Convert Roll/Pitch/Yaw into Quaternion
    tf_quat = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    quat = Quaternion(tf_quat[0], tf_quat[1], tf_quat[2], tf_quat[3])

    # Create new msg to publish onto chest_trajectory
    # This msg sets the chest to the 0 position, assuming no args given
    msg = ChestTrajectoryRosMessage(taskspace_trajectory_points=[
                                 SO3TrajectoryPointRosMessage(orientation = quat, angular_velocity =
                                 Vector3(0.3,0.3,0.3), time = 0.0)],
                                 execution_mode = 0, unique_id = 13) # random number; just must no equal zero

    # Publish msg
    chest_trajectory_pub.publish(msg)
    time.sleep(1)
