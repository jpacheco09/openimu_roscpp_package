/**
 * @file openimu_tf_brodcaster.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <ros/ros.h>
#include <tf2/LinearMath>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "openimu_tf_broadcast");
  ros::NodeHandle nh("openimu_pose_broadcast");

  // ros::Publisher openimu_tf_pub = nh.adver;

  return 0;
}