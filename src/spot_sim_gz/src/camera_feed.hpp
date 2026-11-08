#pragma once

// Feeds a Spot camera, a Gazebo visual camera plus a depth camera next to
// it, into spot_sim::CameraSystem: subscribes to both sensors' image topics
// in-process and hands over every new image. For cameras on a moving mount
// (the hand camera on the wrist), each image carries the mount's pose at
// arrival.

#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <spot_sim_core/images.hpp>

namespace spot_sim_gz {

class CameraFeed {
 public:
  struct Sensor {
    std::string topic;
    spot_sim::PinholeIntrinsics intrinsics;
  };

  /// `mount_pose`: body_T_mount now (cameras on a moving mount only).
  using MountPose = std::function<std::optional<spot_sim::Pose>()>;

  CameraFeed(std::string name, Sensor visual, Sensor depth, spot_sim::CameraSystem& cameras, MountPose mount_pose = {})
      : name_(std::move(name)),
        visual_(std::move(visual)),
        depth_(std::move(depth)),
        cameras_(cameras),
        mount_pose_(std::move(mount_pose)) {
    ok_ = node_.Subscribe(visual_.topic, &CameraFeed::OnVisual, this) &&
          node_.Subscribe(depth_.topic, &CameraFeed::OnDepth, this);
  }

  bool ok() const { return ok_; }

 private:
  static spot_sim::RobotTime Stamp(const gz::msgs::Header& header) {
    return spot_sim::RobotTime{header.stamp().sec() * 1'000'000'000LL + header.stamp().nsec()};
  }

  static spot_sim::ImageGeometry Geometry(const gz::msgs::Image& msg, const Sensor& sensor) {
    return spot_sim::ImageGeometry{static_cast<int>(msg.width()), static_cast<int>(msg.height()), sensor.intrinsics};
  }

  void OnVisual(const gz::msgs::Image& msg) {
    if (msg.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) {
      return;
    }
    spot_sim::VisualFrame frame;
    frame.stamp = Stamp(msg.header());
    frame.geometry = Geometry(msg, visual_);
    if (mount_pose_) frame.body_T_mount = mount_pose_();
    const std::size_t row = static_cast<std::size_t>(msg.width()) * 3;
    frame.rgb.resize(row * msg.height());
    for (uint32_t y = 0; y < msg.height(); ++y) {  // drop any row padding
      std::memcpy(frame.rgb.data() + y * row, msg.data().data() + y * msg.step(), row);
    }
    cameras_.SetVisualFrame(name_, std::move(frame));
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
    spot_sim::DepthFrame frame;
    frame.stamp = Stamp(msg.header());
    frame.geometry = Geometry(msg, depth_);
    if (mount_pose_) frame.body_T_mount = mount_pose_();
    frame.depth_mm = spot_sim::DepthToMillimeters(meters.data(), count);
    cameras_.SetDepthFrame(name_, std::move(frame));
  }

  std::string name_;
  Sensor visual_;
  Sensor depth_;
  spot_sim::CameraSystem& cameras_;
  MountPose mount_pose_;
  gz::transport::Node node_;
  bool ok_ = false;
};

}  // namespace spot_sim_gz
