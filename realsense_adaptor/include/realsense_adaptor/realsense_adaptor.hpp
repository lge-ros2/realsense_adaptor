/**
 *  @file   realsense_adaptor.hpp
 *  @date   2020-09-17
 *  @author hyunseok Yang
 *  @brief  ROS2 realsense adaptor utility
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#ifndef _REALSENSE_ADAPTOR_H_
#define _REALSENSE_ADAPTOR_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <realsense_adaptor_msgs/msg/synced_image.hpp>

#include <vector>

class RealsenseAdaptor : public rclcpp::Node
{
public:
  explicit RealsenseAdaptor(const std::string node_name);
  ~RealsenseAdaptor();

private:
  struct CameraSet
  {
    void create(rclcpp::Node::SharedPtr node, std::string infoTopic, std::string dataTopic);

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSub;

    sensor_msgs::msg::CameraInfo infoMessage;
    message_filters::Subscriber<sensor_msgs::msg::Image> imageSub;
  };

  struct AdaptorSet
  {
    std::string moduleName;

    CameraSet color;
    CameraSet depth;

    // TODO: To be removed(?)
    // message_filters::Subscriber<realsense_camera_msgs::msg::DeviceInfo> deviceSub;

    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync;

    realsense_adaptor_msgs::msg::SyncedImage pubMessage;
    rclcpp::Publisher<realsense_adaptor_msgs::msg::SyncedImage>::SharedPtr pub;

    void callback(sensor_msgs::msg::Image::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr);
  };

  std::vector<std::shared_ptr<AdaptorSet>> adaptorSetList_;

private:
  static void SetCameraInfo2CameraIntrinsic(const sensor_msgs::msg::CameraInfo* const, realsense_adaptor_msgs::msg::CameraIntrinsic&);
};

#endif