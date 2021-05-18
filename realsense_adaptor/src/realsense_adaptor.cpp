/**
 *  @file   realsense_adaptor.cpp
 *  @date   2020-09-24
 *  @author hyunseok Yang
 *  @brief  ROS2 realsense adaptor utility
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include "realsense_adaptor/realsense_adaptor.hpp"
#include <math.h>

using namespace std;
using placeholders::_1;
using placeholders::_2;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using realsense_adaptor_msgs::msg::SyncedImage;
using realsense_adaptor_msgs::msg::CameraIntrinsic;

RealsenseAdaptor::RealsenseAdaptor(const string node_name)
    : Node(node_name,
           rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
{
  // RCLCPP_INFO(this->get_logger(), "name %s namespace %s", get_name(), get_namespace());

  int queueSize;
  get_parameter_or("queue_size", queueSize, int(10));

  vector<string> deviceList;
  get_parameter("realsense_devices", deviceList);

  string realsense_color_info_topic, realsense_color_topic;
  string realsense_depth_info_topic, realsense_depth_topic;
  string adaptor_ouput_topic;

  get_parameter("realsense_color_info_topic", realsense_color_info_topic);
  get_parameter("realsense_color_topic", realsense_color_topic);
  get_parameter("realsense_depth_info_topic", realsense_depth_info_topic);
  get_parameter("realsense_depth_topic", realsense_depth_topic);
  get_parameter("realsense_adaptor_ouput_topic", adaptor_ouput_topic);

  RCLCPP_INFO(this->get_logger(), "realsenseList count: %d", deviceList.size());
  RCLCPP_INFO(this->get_logger(), "queue_size: %d", queueSize);
  RCLCPP_INFO(this->get_logger(), "realsense_color_topic (info, image): (%s, %s)", realsense_color_info_topic.c_str(), realsense_color_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "realsense_depth_topic (info, image): (%s, %s)", realsense_depth_info_topic.c_str(), realsense_depth_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "realsense_adaptor_ouput_topic: %s", adaptor_ouput_topic.c_str());

  auto nodeHandle = shared_ptr<rclcpp::Node>(this, [](auto) {});

  for (auto realsense : deviceList)
  {
    RCLCPP_INFO(this->get_logger(), "module: %s", realsense.c_str());

    const auto topicPrefix = realsense + "/";
    auto adaptor = std::make_shared<AdaptorSet>();

    adaptor->moduleName = realsense;

    using namespace message_filters;
    {
      adaptor->color.create(nodeHandle, topicPrefix + realsense_color_info_topic, topicPrefix + realsense_color_topic);
      adaptor->depth.create(nodeHandle, topicPrefix + realsense_depth_info_topic, topicPrefix + realsense_depth_topic);

      // TODO: To be removed(?)
      // adaptor->deviceSub = make_unique<Subscriber<realsense_camera_msgs::msg::DeviceInfo>>(this, "/camera/device_info");

      adaptor->sync = std::make_unique<TimeSynchronizer<Image, Image>>(adaptor->color.imageSub, adaptor->depth.imageSub, queueSize);
      adaptor->sync->registerCallback(bind(&AdaptorSet::callback, adaptor.get(), _1, _2));
    }

    adaptor->pub = create_publisher<SyncedImage>(topicPrefix + adaptor_ouput_topic, rclcpp::SensorDataQoS());
    // RCLCPP_INFO(this->get_logger(), "-> Create publihser: %s ", realsense.c_str());

    adaptorSetList_.push_back(adaptor);
  }
}

RealsenseAdaptor::~RealsenseAdaptor()
{
  for (auto adaptor : adaptorSetList_)
  {
    adaptor->color.imageSub.unsubscribe();
    adaptor->depth.imageSub.unsubscribe();

    adaptor->sync.release();
  }
}

void RealsenseAdaptor::CameraSet::create(rclcpp::Node::SharedPtr node, string infoTopic, string dataTopic)
{
  auto depthInfoMessage = &infoMessage;
  auto callback_depth_info = [infoMessage = depthInfoMessage](CameraInfo::ConstSharedPtr msg) -> void {
    if (infoMessage != nullptr)
      *infoMessage = *msg;
  };

  infoSub = node->create_subscription<CameraInfo>(infoTopic, rclcpp::QoS(1), callback_depth_info);
  imageSub.subscribe(node, dataTopic);
}

void RealsenseAdaptor::SetCameraInfo2CameraIntrinsic(const CameraInfo* const src, CameraIntrinsic &dst)
{
  const auto fx = src->k[0];
  const auto fy = src->k[4];
  const auto hfov = 2 * atan((src->width/2) / fx);
  const auto vfov = hfov * ((float)src->height/(float)src->width);

  dst.width = src->width;
  dst.height = src->height;
  dst.cx = src->k[2];
  dst.cy = src->k[5];
  dst.fx = fx;
  dst.fy = fy;
  dst.hfov = hfov;
  dst.vfov = vfov;
  dst.baseline = -src->p[3]/fx;
  dst.coeff[0] = src->d[0];
  dst.coeff[1] = src->d[1];
  dst.coeff[2] = src->d[2];
  dst.coeff[3] = src->d[3];
  dst.coeff[4] = src->d[4];
  dst.depthscale = 1.0f; // ?
}

void RealsenseAdaptor::AdaptorSet::callback(
  Image::ConstSharedPtr colorData,
  Image::ConstSharedPtr depthData)
{
  auto message_time = colorData->header.stamp;

  this->pubMessage.header.stamp = message_time;

  RealsenseAdaptor::SetCameraInfo2CameraIntrinsic(&this->color.infoMessage, this->pubMessage.color.intrinsic);

  this->pubMessage.color.data_name = "color";
  this->pubMessage.color.channels = 3;
  this->pubMessage.color.data_unit_size = 1;
  this->pubMessage.color.data.assign(colorData->data.size(), 0);
  memcpy(&this->pubMessage.color.data[0], &colorData->data[0], colorData->data.size());

  RealsenseAdaptor::SetCameraInfo2CameraIntrinsic(&this->depth.infoMessage, this->pubMessage.depth.intrinsic);

  this->pubMessage.depth.data_name = "depth";
  this->pubMessage.depth.channels = 1;
  this->pubMessage.depth.data_unit_size = 2;
  this->pubMessage.depth.data.assign(depthData->data.size(), 0);
  memcpy(&this->pubMessage.depth.data[0], &depthData->data[0], depthData->data.size());

  // printf("[%s] callback \n", this->moduleName.c_str());
  // printf("[%s] color width: %d height: %d\n", this->moduleName.c_str(), this->pubMessage.color.intrinsic.nwidth, this->pubMessage.color.intrinsic.nheight);

  this->pub->publish(this->pubMessage);
}