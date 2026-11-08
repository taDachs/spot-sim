#pragma once

// Feeds Gazebo RGB-D camera sensors on the robot's body into
// spot_sim::BodyCameras: subscribes to each sensor's image, depth and
// camera_info topics in-process and publishes a frame whenever color and
// depth of the same render have arrived.

#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <spot_sim_core/images.hpp>

namespace spot_sim_gz {

class CameraFeed {
 public:
  /// Subscribes to `<topic>/image`, `<topic>/depth_image` and
  /// `<topic>/camera_info` of the rgbd_camera sensor `name`.
  CameraFeed(std::string name, const std::string& topic, spot_sim::BodyCameras& cameras)
      : name_(std::move(name)), cameras_(cameras) {
    ok_ = node_.Subscribe(topic + "/image", &CameraFeed::OnImage, this) &&
          node_.Subscribe(topic + "/depth_image", &CameraFeed::OnDepth, this) &&
          node_.Subscribe(topic + "/camera_info", &CameraFeed::OnInfo, this);
  }

  bool ok() const { return ok_; }

 private:
  static spot_sim::RobotTime Stamp(const gz::msgs::Header& header) {
    return spot_sim::RobotTime{header.stamp().sec() * 1'000'000'000LL + header.stamp().nsec()};
  }

  void OnInfo(const gz::msgs::CameraInfo& msg) {
    if (msg.intrinsics().k_size() < 6) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    intrinsics_ = spot_sim::PinholeIntrinsics{msg.intrinsics().k(0), msg.intrinsics().k(4), msg.intrinsics().k(2),
                                              msg.intrinsics().k(5)};
  }

  void OnImage(const gz::msgs::Image& msg) {
    if (msg.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) {
      return;
    }
    const std::size_t row = static_cast<std::size_t>(msg.width()) * 3;
    std::vector<uint8_t> rgb(row * msg.height());
    for (uint32_t y = 0; y < msg.height(); ++y) {  // drop any row padding
      std::memcpy(rgb.data() + y * row, msg.data().data() + y * msg.step(), row);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    rgb_ = Pending<uint8_t>{Stamp(msg.header()), static_cast<int>(msg.width()), static_cast<int>(msg.height()),
                            std::move(rgb)};
    Publish();
  }

  void OnDepth(const gz::msgs::Image& msg) {
    if (msg.pixel_format_type() != gz::msgs::PixelFormatType::R_FLOAT32) {
      return;
    }
    const std::size_t count = static_cast<std::size_t>(msg.width()) * msg.height();
    std::vector<float> meters(count);
    for (uint32_t y = 0; y < msg.height(); ++y) {
      std::memcpy(meters.data() + y * msg.width(), msg.data().data() + y * msg.step(), msg.width() * sizeof(float));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    depth_ = Pending<uint16_t>{Stamp(msg.header()), static_cast<int>(msg.width()), static_cast<int>(msg.height()),
                               spot_sim::DepthToMillimeters(meters.data(), count)};
    Publish();
  }

  /// Hands over color + depth once both of one render are in (mutex held).
  void Publish() {
    if (!rgb_ || !depth_ || !intrinsics_ || rgb_->stamp != depth_->stamp || rgb_->width != depth_->width ||
        rgb_->height != depth_->height) {
      return;
    }
    spot_sim::CameraFrame frame;
    frame.stamp = rgb_->stamp;
    frame.width = rgb_->width;
    frame.height = rgb_->height;
    frame.intrinsics = *intrinsics_;
    frame.rgb = std::move(rgb_->data);
    frame.depth_mm = std::move(depth_->data);
    rgb_.reset();
    depth_.reset();
    cameras_.SetFrame(name_, std::move(frame));
  }

  template <typename T>
  struct Pending {
    spot_sim::RobotTime stamp;
    int width = 0, height = 0;
    std::vector<T> data;
  };

  std::string name_;
  spot_sim::BodyCameras& cameras_;
  gz::transport::Node node_;
  bool ok_ = false;
  std::mutex mutex_;
  std::optional<spot_sim::PinholeIntrinsics> intrinsics_;
  std::optional<Pending<uint8_t>> rgb_;
  std::optional<Pending<uint16_t>> depth_;
};

}  // namespace spot_sim_gz
