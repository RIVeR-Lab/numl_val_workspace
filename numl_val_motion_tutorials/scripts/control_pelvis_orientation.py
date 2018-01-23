#!/usr/bin/env python

import rospy
from ihmc_msgs.msg import *
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import tf
import math
import sys
import time

if __name__ == '__main__':
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    # No args zeros pelvis orientation
    if len(sys.argv) == 1:
        print "Setting to 0 orientation, use 3 parameters for roll, pitch, yaw"
    # Otherwise, set user inputs to respective parameters
    elif len(sys.argv) == 4:
        roll = math.radians(float(sys.argv[1]))
        pitch = math.radians(float(sys.argv[2]))
        yaw = math.radians(float(sys.argv[3]))
        print "Setting pelvis orientation"
    # End if user did not input right amount of parameters
    elif len(sys.argv) != 4:
        print "Please enter roll, pitch, yaw coordinates or nothing to zero pelvis orientation"
        sys.exit()

    # Create a publisher to publish on the pelvis_orientation_trajectory topic
    pelvis_orientation_pub = rospy.Publisher("/ihmc_ros/valkyrie/control/pelvis_orientation_trajectory",
                                             PelvisOrientationTrajectoryRosMessage, queue_size = 1)

    # Create a node to set the orientation of the pelvis
    rospy.init_node("control_pelvis_orientation")

    # Convert Roll/Pitch/Yaw into a quaternion
    tfq = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    quat = Quaternion(tfq[0], tfq[1], tfq[2], tfq[3])

    # Use the quaternion to set the position of the pelvis
    msg = ChestTrajectoryRosMessage(taskspace_trajectory_points = [
    SO3TrajectoryPointRosMessage(orientation = quat, angular_velocity = Vector3(0.3, 0.3, 0.3), time = 0.0)]
    , execution_mode = 0, unique_id = 13) # random number; just must not equal zero

    # Publish msg
    pelvis_orientation_pub.publish(msg)
    time.sleep(1)
