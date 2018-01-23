#include <ros/ros.h>
#include <ihmc_msgs/PelvisOrientationTrajectoryRosMessage.h>
#include <ihmc_msgs/SO3TrajectoryPointRosMessage.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h> // createQuaternionMsgFromRollPitchYaw()
#include <iostream>

// Will be used to convert user input to rads
// Rads are used in tf euler coordinates roll,pitch,yaw
double degree2rad(double degree)
{
  return (degree * M_PI)/180;
}

int main(int argc, char* argv[])
{
  double roll = 0.0, pitch = 0.0, yaw = 0.0;

  // Create a new node to control Valkyrie's pelvis
  ros::init(argc, argv, "control_pelvis_orientation");

  // New nodehandler for publishing on a topic
  ros::NodeHandle node;

  // New publisher to publish on the chest_trajectory topic
  ros::Publisher pelvis_trajectory_pub = node.advertise<ihmc_msgs::PelvisOrientationTrajectoryRosMessage>
                          ("/ihmc_ros/valkyrie/control/pelvis_orientation_trajectory", 1);

  // Wait until we publisher is setup, Valkyrie may not respond otherwise
  while (pelvis_trajectory_pub.getNumSubscribers() == 0) continue;

  // No args zeros pelvis orientation
  if (argc == 1)
  {
     std::cout << "Setting to 0 orientation, use 3 parameters for roll, pitch, yaw\n";
  }
  // Otherwise, set user inputs to respective parameters
  else if (argc == 4)
  {
    roll = degree2rad(std::atof(argv[1]));
    pitch = degree2rad(std::atof(argv[2]));
    yaw = degree2rad(std::atof(argv[3]));
    std::cout << "Setting pelvis orientation";
  }
  // End if user did not input right amount of parameters
  else if (argc != 4)
  {
    std::cout << "Please enter roll, pitch, yaw coordinates or nothing to zero pelvis orientation\n";
    return 0;
  }

  // Transform euler coordinates into quaternions
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  // Create new SO3 Trajectory msg for the Pelvis trajectory msg
  ihmc_msgs::SO3TrajectoryPointRosMessage SO3_msg;
  SO3_msg.orientation = quat;
  SO3_msg.angular_velocity.x = 0.3;
  SO3_msg.angular_velocity.y = 0.3;
  SO3_msg.angular_velocity.z = 0.3;
  SO3_msg.time = 0.0;

  // Create new Pelvis Trajectory msg to publish
  ihmc_msgs::PelvisOrientationTrajectoryRosMessage pelvis_msg;
  pelvis_msg.taskspace_trajectory_points.push_back(SO3_msg);
  pelvis_msg.execution_mode = 0;
  pelvis_msg.unique_id = 13; // random number; just must not equal zero

  // Publish Pelvis Trajectory msg
  pelvis_trajectory_pub.publish(pelvis_msg);
  sleep(1);

  return 0;
}
