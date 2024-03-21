#ifndef ARAVIS_CAMERA_NODE_H
#define ARAVIS_CAMERA_NODE_H

/* add these to includePath
  "/usr/include/glib-2.0",
  "/usr/lib/x86_64-linux-gnu/glib-2.0/include"
*/
#include <arv.h>

#include <array>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>
#include <utility>

class AravisCameraNode : public rclcpp::Node {
 public:
  AravisCameraNode();
  virtual ~AravisCameraNode();

  void spin();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr publishing_timer;

  ArvCamera *camera_;
  ArvStream *stream;
  GError *error = nullptr;
  ArvBuffer *latest_buffer_ = nullptr;
  std::atomic<bool> new_buffer_available{false};

  bool InitializeAravisCamera();
  void CleanupResources();
  void ProcessAndPublish();

  static void StreamCallback(void *user_data, ArvStreamCallbackType type,
                             ArvBuffer *buffer);

  rcl_interfaces::msg::SetParametersResult SetExposure(double exposure_time);
  rcl_interfaces::msg::SetParametersResult SetGain(double gain);

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult onParameterChanged(
      const std::vector<rclcpp::Parameter> &parameters);
};

#endif  // ARAVIS_CAMERA_NODE_H
