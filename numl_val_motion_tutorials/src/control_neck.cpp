#include <ros/ros.h>
#include <ihmc_msgs/NeckTrajectoryRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <iostream>

int main(int argc, char* argv[])
{
  // Variables for neck position
  double joint_1 = 0.0; // lowerlimit: 0.0, upperlimit: 1.162
  double joint_2 = 0.0; // lowerlimit: -1.0472, upperlimit: 1.0472
  double joint_3 = 0.0; // lowerlimit: -0.872, upperlimit: 0.0

  // Create a new node to control Valkyrie's neck
  ros::init(argc, argv, "control_neck");

  // New nodehandler for publishing on a topic
  ros::NodeHandle node;

  // New publisher to publish on the neck_trajectory topic
  ros::Publisher neck_trajectory_pub = node.advertise<ihmc_msgs::NeckTrajectoryRosMessage>
                                        ("/ihmc_ros/valkyrie/control/neck_trajectory", 1);

  // Wait until we publisher is setup, Valkyrie may not respond otherwise
  while (neck_trajectory_pub.getNumSubscribers() == 0) continue;

  // No args zeros neck orientation
  if (argc == 1)
    std::cout << "Setting to 0 orientation, use 3 parameters for roll, pitch, yaw\n";

  // Otherwise, sets user inputs to respective trajectories
  else if(argc == 4)
  {
    joint_1 = std::atof(argv[1]);
    joint_2 = std::atof(argv[2]);
    joint_3 = std::atof(argv[3]);
  }
  else if(argc != 4)
  {
    std::cout << "Please enter roll, pich, and yaw coordinates or nothing to zero neck orientation\n";
    return 0;
  }

  // Create new trajectory pt msgs for RYP coordinates
  ihmc_msgs::TrajectoryPoint1DRosMessage traj_pt_msg_r;
  ihmc_msgs::TrajectoryPoint1DRosMessage traj_pt_msg_y;
  ihmc_msgs::TrajectoryPoint1DRosMessage traj_pt_msg_p;

  // Set parameter to each msg
  traj_pt_msg_r.position = joint_1;  traj_pt_msg_r.time = 0.0; traj_pt_msg_r.velocity = 0.3;
  traj_pt_msg_y.position = joint_2;  traj_pt_msg_y.time = 0.0; traj_pt_msg_y.velocity = 0.4;
  traj_pt_msg_p.position = joint_3;  traj_pt_msg_p.time = 0.0; traj_pt_msg_p.velocity = 0.5;

  // Create new 1DoF trajectory msg
  ihmc_msgs::OneDoFJointTrajectoryRosMessage traj_msg_r;
  ihmc_msgs::OneDoFJointTrajectoryRosMessage traj_msg_y;
  ihmc_msgs::OneDoFJointTrajectoryRosMessage traj_msg_p;

  // Assign each RYP coordinate to a trajectory
  traj_msg_r.trajectory_points.push_back(traj_pt_msg_r);
  traj_msg_y.trajectory_points.push_back(traj_pt_msg_y);
  traj_msg_p.trajectory_points.push_back(traj_pt_msg_p);

  // Create new Neck Trajectory msg from 1DoF Trajectory msgs
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.joint_trajectory_messages.push_back(traj_msg_r);
  neck_msg.joint_trajectory_messages.push_back(traj_msg_y);
  neck_msg.joint_trajectory_messages.push_back(traj_msg_p);
  neck_msg.unique_id = 13; // random number; just must not equal zero

  // Publish Neck Trajectory msg
  neck_trajectory_pub.publish(neck_msg);
  sleep(1);

  return 0;
}
