#include "aravis_camera/aravis_camera_node.h"

#include <iostream>

AravisCameraNode::AravisCameraNode() : Node("aravis_camera_node") {
  auto exposure_desc = rcl_interfaces::msg::ParameterDescriptor{};
  exposure_desc.description = "Exposure time in microseconds";
  this->declare_parameter<double>("exposure_time", 22222.0, exposure_desc);

  auto gain_desc = rcl_interfaces::msg::ParameterDescriptor{};
  gain_desc.description = "Gain in dB";
  this->declare_parameter<double>("gain", 10.0, gain_desc);

  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &AravisCameraNode::onParameterChanged, this, std::placeholders::_1));

  image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("aravis_image", 10);

  publishing_timer = this->create_wall_timer(
      std::chrono::milliseconds(200),  // 5 Hz
      std::bind(&AravisCameraNode::ProcessAndPublish, this));

  if (!InitializeAravisCamera()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Aravis camera.");
    rclcpp::shutdown();
  }
}

AravisCameraNode::~AravisCameraNode() { CleanupResources(); }

void AravisCameraNode::spin() {
  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
  }
}

rcl_interfaces::msg::SetParametersResult AravisCameraNode::SetExposure(
    double exposure_time) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  result.reason = "Exposure time updated successfully";

  arv_camera_set_exposure_time(camera_, exposure_time, &error);
  if (error != nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set exposure time: %s",
                 error->message);
    result.successful = false;
    result.reason = "Failed to set exposure time";
    g_error_free(error);
  } else {
    double exposure_time = arv_camera_get_exposure_time(camera_, &error);
    if (error != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get exposure time: %s",
                   error->message);
      g_error_free(error);
    }
    RCLCPP_INFO(this->get_logger(), "Exposure time set to %f microseconds.",
                exposure_time);
  }

  return result;
}

rcl_interfaces::msg::SetParametersResult AravisCameraNode::SetGain(
    double gain) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  result.reason = "Gain updated successfully";

  arv_camera_set_gain(camera_, gain, &error);
  if (error != nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set gain: %s", error->message);
    result.successful = false;
    result.reason = "Failed to set gain";
    g_error_free(error);
  } else {
    double gain = arv_camera_get_gain(camera_, &error);
    if (error != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get gain: %s",
                   error->message);
      g_error_free(error);
    }
    RCLCPP_INFO(this->get_logger(), "Gain set to %f dB.", gain);
  }

  return result;
}

rcl_interfaces::msg::SetParametersResult AravisCameraNode::onParameterChanged(
    const std::vector<rclcpp::Parameter>& parameters) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  result.reason = "Parameters updated successfully";

  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "exposure_time") {
      auto exposure_time = parameter.as_double();
      result = SetExposure(exposure_time);

    } else if (parameter.get_name() == "gain") {
      auto gain = parameter.as_double();
      result = SetGain(gain);
    }
  }

  return result;
}

bool AravisCameraNode::InitializeAravisCamera() {
  RCLCPP_INFO(this->get_logger(), "Initializing Aravis camera.");
  error = nullptr;

  camera_ = arv_camera_new(nullptr, &error);
  if (!ARV_IS_CAMERA(camera_)) {
    if (error != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to the camera: %s",
                   error->message);
      g_error_free(error);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to connect to the camera: No specific error.");
    }
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Found camera '%s'",
              arv_camera_get_model_name(camera_, nullptr));

  arv_camera_set_acquisition_mode(camera_, ARV_ACQUISITION_MODE_CONTINUOUS,
                                  &error);
  if (error != nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set acquisition mode: %s",
                 error->message);
    g_error_free(error);
  }

  stream = arv_camera_create_stream(camera_, AravisCameraNode::StreamCallback,
                                    this, &error);

  if (ARV_IS_STREAM(stream)) {
    size_t payload = arv_camera_get_payload(camera_, &error);

    if (error != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get payload: %s",
                   error->message);
      g_error_free(error);
    }

    for (int i = 0; i < 2; i++) {
      arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));
    }

    arv_camera_start_acquisition(camera_, &error);
    if (error != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start acquisition: %s",
                   error->message);
      g_error_free(error);
      return false;
    }

  } else {
    if (error != nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create stream: %s",
                   error->message);
      g_error_free(error);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to create stream: No specific error.");
    }
  }

  SetExposure(this->get_parameter("exposure_time").as_double());
  SetGain(this->get_parameter("gain").as_double());
  RCLCPP_INFO(this->get_logger(),
              "Camera and stream initialization completed successfully");
  return true;
}

void AravisCameraNode::CleanupResources() {
  RCLCPP_INFO(this->get_logger(),
              "Cleaning up resources and destroying camera manager.");

  arv_camera_stop_acquisition(camera_, &error);
  if (error != nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to stop acquisition: %s",
                 error->message);
    g_error_free(error);
  }
}

void AravisCameraNode::ProcessAndPublish() {
  if (new_buffer_available.load()) {
    size_t buffer_size;
    const void* buffer_data = arv_buffer_get_data(latest_buffer_, &buffer_size);

    if (buffer_data == nullptr || buffer_size == 0) {
      RCLCPP_ERROR(this->get_logger(), "Buffer data is empty.");
      return;
    }

    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "aravis_frame";

    msg.encoding = sensor_msgs::image_encodings::MONO8;
    msg.height = arv_buffer_get_image_height(latest_buffer_);
    msg.width = arv_buffer_get_image_width(latest_buffer_);
    msg.step = msg.width;  // For MONO8, step size is equal to width.
    size_t image_size = msg.step * msg.height;
    msg.data.resize(image_size);

    memcpy(&msg.data[0], buffer_data, image_size);

    image_pub_->publish(msg);
    latest_buffer_ = nullptr;

    new_buffer_available.store(false);
  } else {
    RCLCPP_WARN(this->get_logger(), "No new buffer available.");
  }
}

void AravisCameraNode::StreamCallback(void* user_data,
                                      ArvStreamCallbackType type,
                                      ArvBuffer* buffer) {
  auto node = static_cast<AravisCameraNode*>(user_data);

  switch (type) {
    case ARV_STREAM_CALLBACK_TYPE_INIT:
      break;
    case ARV_STREAM_CALLBACK_TYPE_START_BUFFER:
      break;
    case ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE: {
      if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
        node->latest_buffer_ = buffer;
        node->new_buffer_available.store(true);
      }

      arv_stream_push_buffer(node->stream, buffer);
    } break;
    case ARV_STREAM_CALLBACK_TYPE_EXIT:
      break;
    default:
      break;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AravisCameraNode>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
