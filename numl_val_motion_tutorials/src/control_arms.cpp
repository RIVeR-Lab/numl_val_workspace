#include <ros/ros.h>
#include <ihmc_msgs/ArmTrajectoryRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <unistd.h> // sleep()
#include <iostream> // cout


// This program sets Valkyrie's arms to the zero position if only the side of the robot is passed to it.
// It will set the arms to the user specified location if the robot side and 7 positions are passed to it.

int main(int argc, char* argv[])
{

  static int joints = 7; // Static variable referincing the number of joints in the arm
  static int entries = 9; // Static variable referincing the number of entries to empty

  // Create a new node to control Valkyrie's arms
 ros::init(argc, argv, "control_arms");

  // New nodehandler for publishing on a topic
  ros::NodeHandle node;

  // New publisher to publish on the arm_trajectory topic
  ros::Publisher arm_trajectory_pub = node.advertise<ihmc_msgs::ArmTrajectoryRosMessage>
                                        ("/ihmc_ros/valkyrie/control/arm_trajectory", 1);

  // Wait until publisher is setup, Valkyrie may not respond otherwise
  while(arm_trajectory_pub.getNumSubscribers() == 0)
    continue;

  // Determine whether the number of arguments passed is enough
  if (argc == 1)
  {
    std::cout << "One argument for which side, optional 7 additional arguments for joint control\n";
    return 0;
  }
  // If only one arg passed (for robot side), zero arm orientation
  else if (argc == 2)
  {
     std::cout << "Sending arms to all 0 positions\n";
  }
  // End if incorrect number of arguments passed
  else if (argc != 9)
  {
    std::cout << "Please start with either 1 (for robot side) or 8 arguments (for robot side, followed by position of each of the 7 arm joints)\n";
    return 0;
  }

  // Get from input which side of the robot to control
<<<<<<< HEAD:numl_val_motion_tutorials/src/control_arms.cpp
  int robot_side = 0;  // 0 refers to right side of the robot
=======
  int robot_side = 0;  //0 refers to left side of the robot
>>>>>>> 08ecdd270103c7b4c1e48575ebc72394af561799:numl_val_motion_tutorials/src/control_arms.cpp
  robot_side = std::atoi(argv[1]);

  // Create IHMC_msgs objects to publish to arm
  ihmc_msgs::ArmTrajectoryRosMessage arm_msg;     // Main arm msg to be sent to arm
  ihmc_msgs::OneDoFJointTrajectoryRosMessage traj_msg;   // Trajectory msg that consists of trajectory points msgs
  ihmc_msgs::TrajectoryPoint1DRosMessage traj_pt_msg;   // Trajectory pt. msg used to set time to travel, position, and velocity

  // Set the time of travel, position, and velocity for a joint
  traj_pt_msg.time = 3.0; traj_pt_msg.position = 0.0; traj_pt_msg.velocity = 0.1;
  traj_msg.trajectory_points.push_back(traj_pt_msg);

  // Assign trajectory msgs to each arm joint
  for (int i = 0; i < joints; i++)
  {
      arm_msg.joint_trajectory_messages.push_back(traj_msg);
  }

<<<<<<< HEAD:numl_val_motion_tutorials/src/control_arms.cpp
=======
  // Set the trajectory to the arm
  for (int t = 0; t < joints; t++)
  {
    arm_msg.joint_trajectory_messages.push_back(traj_msg);
  }
  
>>>>>>> 08ecdd270103c7b4c1e48575ebc72394af561799:numl_val_motion_tutorials/src/control_arms.cpp
  // Set which side of the robot to control
  arm_msg.robot_side = robot_side;

  // Give msg execution priority and some ID
  arm_msg.execution_mode = 0; arm_msg.unique_id = 13; // random number; just must not equal zero

  // If all arguments valid, set the position of the respective joint to its respective argument
  if (argc == 9)
  {
    // Empty msg from previous entries
    arm_msg.joint_trajectory_messages.clear(); traj_msg.trajectory_points.clear();
    for (int i = 2; i < entries; i++)
    {
      traj_pt_msg.position = std::atof(argv[i]);
      traj_msg.trajectory_points.push_back(traj_pt_msg);
      arm_msg.joint_trajectory_messages.push_back(traj_msg);
    }
<<<<<<< HEAD:numl_val_motion_tutorials/src/control_arms.cpp
=======

    // Set the trajectory to the arm
    for (int t =0; t < joints; t++)
    {
      arm_msg.joint_trajectory_messages.push_back(traj_msg);
    }
>>>>>>> 08ecdd270103c7b4c1e48575ebc72394af561799:numl_val_motion_tutorials/src/control_arms.cpp
  }

  // Publish msg
  arm_trajectory_pub.publish(arm_msg);
  sleep(1);

  return 0;
}
