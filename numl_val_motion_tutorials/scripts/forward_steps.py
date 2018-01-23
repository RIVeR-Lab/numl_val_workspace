#!/usr/bin/env python

#
# Python script that has Valkyrie walk forward a given number of steps
#


import time
import rospy
import tf
import tf2_ros
import numpy
import sys

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

TRANSFER_TIME = 1.5
SWING_TIME = 1.5
STEP_SIZE = 0.3
DISTANCE_BETWEEN_FEET = 0.277

# Takes a small sideways step to set feet the required distance apart for walking.
def sidewayStep():

    msg = FootstepDataListRosMessage(transfer_time = TRANSFER_TIME,
                swing_time = SWING_TIME, unique_id = rospy.get_time())

    left_foot_location = createOffsetFootstep(LEFT, [0.0, DISTANCE_BETWEEN_FEET, 0.0], 0)

    # Side step
    msg.footstep_data_list.append(left_foot_location)
    footstep_list_pub.publish(msg)


# Walks forward the desired amount of steps.
def walkForward(steps):

    msg = FootstepDataListRosMessage(transfer_time = TRANSFER_TIME,
                swing_time = SWING_TIME, unique_id = rospy.get_time())

    # Generate the footsteps
    x = STEP_SIZE
    counter = 1
    while counter <= steps/2:
        msg.footstep_data_list.append(createOffsetFootstep(LEFT, [x, DISTANCE_BETWEEN_FEET, 0.0], 0))
        msg.footstep_data_list.append(createOffsetFootstep(RIGHT, [x+STEP_SIZE, -DISTANCE_BETWEEN_FEET, 0.0], 0))
        x += STEP_SIZE*2
        counter += 1

    # Generate one more footstep then bring the feet together if desired
    # number of steps is odd, otherwise just bring the feet together
    if steps % 2 == 1:
        msg.footstep_data_list.append(createOffsetFootstep(LEFT, [x, DISTANCE_BETWEEN_FEET, 0.0], 0))
        msg.footstep_data_list.append(createOffsetFootstep(RIGHT, [x, -DISTANCE_BETWEEN_FEET, 0.0], 0))
    else:
        msg.footstep_data_list.append(createOffsetFootstep(LEFT, [x-STEP_SIZE, DISTANCE_BETWEEN_FEET, 0.0], 0))

    footstep_list_pub.publish(msg)


# Creates footstep for the desired foot that is the given offset and
# angle in the z-axis from the stationary foot. The offset is in foot frame.
def createOffsetFootstep(step_side, offset, angle):

    footstep = createStationaryFootstep(step_side)
    if step_side == LEFT:
        stationary_foot = createStationaryFootstep(RIGHT)
    else:
        stationary_foot = createStationaryFootstep(LEFT)

    # Add in the angle
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [stationary_foot.orientation.x, stationary_foot.orientation.y,
        stationary_foot.orientation.z, stationary_foot.orientation.w])
    y += angle
    rotated_quat = tf.transformations.quaternion_from_euler(r, p, y)
    footstep.orientation.x = rotated_quat[0]
    footstep.orientation.y = rotated_quat[1]
    footstep.orientation.z = rotated_quat[2]
    footstep.orientation.w = rotated_quat[3]

    # Transform the offset to world frame
    quat = stationary_foot.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformed_offset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x = stationary_foot.location.x + transformed_offset[0]
    footstep.location.y = stationary_foot.location.y + transformed_offset[1]
    footstep.location.z = stationary_foot.location.z + transformed_offset[2]

    return footstep


 # Creates footstep with the current position and orientation of the given
 # foot.
def createStationaryFootstep(step_side):

    footstep = FootstepDataRosMessage(robot_side = step_side)

    if step_side == LEFT:
        foot_frame = 'leftFoot'
    else:
        foot_frame = 'rightFoot'

    foot_world = tf_buffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = foot_world.transform.rotation
    footstep.location = foot_world.transform.translation

    return footstep


if __name__ == '__main__':

    try:
        footstep_list_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=1)

        rospy.init_node('valkyrie_forward_walk')

        # Set up TF so we can place footsteps relative to the world frame
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rate = rospy.Rate(10)
        time.sleep(1)

        # Determine amount of steps to take
        if len(sys.argv) == 1:
            print "Taking 7 steps"
            steps = 7
        elif len(sys.argv)== 2:
            print "Taking " + sys.argv[1] + " steps"
            steps = int(sys.argv[1])
        else:
            print "Please start with either 0 arguments (for a default of 7 steps) or 1 argument for desired amount of steps"
            sys.exit()

        # Walk
        if not rospy.is_shutdown():

            sidewayStep()
            time.sleep(10)
            walkForward(steps)

            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
