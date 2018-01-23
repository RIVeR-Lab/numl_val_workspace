#include <ros/ros.h>
#include <ihmc_msgs/ChestTrajectoryRosMessage.h>
#include <ihmc_msgs/SO3TrajectoryPointRosMessage.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h> // createQuaternionMsgFromRollPitchYaw()
#include <iostream>
#include <math.h>   // M_PI

// Will be used to convert user input to rads
// Rads are used in tf euler coordinates roll,pitch,yaw
double degree2rad(double degree)
{
  return (degree * M_PI)/180;
}

int main(int argc, char* argv[] )
{
  double roll = 0.0, pitch = 0.0, yaw = 0.0;

  // Create a new node to control Valkyrie's chest
  ros::init(argc, argv, "control_chest_orientation");

  // New nodehandler for publishing on a topic
  ros::NodeHandle node;

  // New publisher to publish on the chest_trajectory topic
  ros::Publisher chest_trajectory_pub = node.advertise<ihmc_msgs::ChestTrajectoryRosMessage>
                                          ("/ihmc_ros/valkyrie/control/chest_trajectory", 1);

  // Wait until the publisher is setup, Valkyrie may not respond otherwise
  while (chest_trajectory_pub.getNumSubscribers() == 0) continue;

  // Based on number of variables passed, set the chest to 0 or move to specified coordinates
  if (argc == 1)
  {
    std::cout << "setting to 0 orientation, use 3 parameters for roll, pitch, yaw\n";
  }
  else if (argc == 4)
  {
    roll = degree2rad(std::atof(argv[1]));
    pitch = degree2rad(std::atof(argv[2]));
    yaw = degree2rad(std::atof(argv[3]));
  }
  else if(argc != 4)
  {
    std::cout << "Please enter roll, pitch, and yaw coordinates or nothing for zero chest orientation\n";
    return 0;
  }

  // Transform euler coordinates into quaternions
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  // Create new SO3 Trajectory msg for the Chest trajectory msg
  ihmc_msgs::SO3TrajectoryPointRosMessage SO3_msg;
  SO3_msg.orientation = quat;
  SO3_msg.angular_velocity.x = 0.3;
  SO3_msg.angular_velocity.y = 0.3;
  SO3_msg.angular_velocity.z = 0.3;
  SO3_msg.time = 0.0;

  // Create new Chest Trajectory msg to publish
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  chest_msg.taskspace_trajectory_points.push_back(SO3_msg);
  chest_msg.execution_mode = 0;
  chest_msg.unique_id = 13; // random number; just must not equal zero

  // Publish Chest Trajectory msg
  chest_trajectory_pub.publish(chest_msg);
  sleep(1);

  return 0;
}
