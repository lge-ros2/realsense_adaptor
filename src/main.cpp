/**
 *  @file   main.cpp
 *  @date   2020-09-17
 *  @author Hyunseok Yang
 *  @brief  ROS2 adaptor node that controls realsense data
 *  @remark
 *  @copyright
 *      LGE Advanced Robotics Laboratory
 *      Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
 *      All Rights are Reserved.
 *
 *      SPDX-License-Identifier: MIT
 */

#include "realsense_adaptor/realsense_adaptor.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseAdaptor>("realsense_adaptor");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
