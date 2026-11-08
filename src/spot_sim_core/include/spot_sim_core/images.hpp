#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <bosdyn/api/image.pb.h>

#include "spot_sim_core/geometry.hpp"
#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Pinhole camera intrinsics [px].
struct PinholeIntrinsics {
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;

  /// Square pixels, principal point at the image center: the model of a
  /// Gazebo camera with horizontal field of view `hfov` [rad].
  static PinholeIntrinsics FromHorizontalFov(int width, int height, double hfov);
};

/// The latest image of a visual (color) camera.
struct VisualFrame {
  RobotTime stamp{0};
  int width = 0, height = 0;
  PinholeIntrinsics intrinsics;
  std::vector<uint8_t> rgb;  ///< width * height * 3, row-major, top row first
};

/// The latest image of a depth camera.
struct DepthFrame {
  RobotTime stamp{0};
  int width = 0, height = 0;
  PinholeIntrinsics intrinsics;
  std::vector<uint16_t> depth_mm;  ///< width * height, distance along the optical axis, 0 = no return
};

/// Depth image conversion: meters (float, inf / nan for no return) to the
/// robot's DEPTH_U16 millimeters. Values outside (0, 65.535] m become 0.
std::vector<uint16_t> DepthToMillimeters(const float* meters, std::size_t count);

/// Re-projects `depth` into another camera with the given image size and
/// intrinsics; `target_T_depth` is the depth camera's optical frame in the
/// target's optical frame. Each depth pixel covers the target pixels its
/// footprint projects to; the nearest surface wins; pixels nothing projects
/// to stay 0. (What depth_image_proc's RegisterNode does, and what Spot does
/// for its <cam>_depth_in_visual_frame sources.)
DepthFrame RegisterDepth(const DepthFrame& depth, const Pose& target_T_depth, int width, int height,
                         const PinholeIntrinsics& target);

/// JPEG encoding of 8-bit RGB or greyscale pixels.
std::string EncodeJpeg(const uint8_t* pixels, int width, int height, int channels, int quality);

/// ImageService semantics for Spot's body cameras. Like on the robot, each
/// camera `<cam>` (e.g. "frontleft") pairs a visual camera with a separate
/// depth sensor next to it:
///   <cam>_fisheye_image          visual, RGB_U8 or GREYSCALE_U8, JPEG or RAW,
///                                in frame "<cam>_fisheye"
///   <cam>_depth                  depth, DEPTH_U16 RAW, in frame "<cam>"
///   <cam>_depth_in_visual_frame  the depth registered to the visual image
///                                (its resolution and intrinsics), in frame
///                                "<cam>_fisheye"
/// Thread-safe.
class BodyCameras {
 public:
  /// Registers a camera. Poses are the optical frames (z forward, x right,
  /// y down) of the visual camera and the depth sensor in the body frame.
  void AddCamera(const std::string& name, const Pose& body_T_visual, const Pose& body_T_depth);

  /// Publish new images of `name` (ignored for unknown cameras).
  void SetVisualFrame(const std::string& name, VisualFrame frame);
  void SetDepthFrame(const std::string& name, DepthFrame frame);

  bool HasCamera(const std::string& name) const;

  bosdyn::api::ListImageSourcesResponse ListImageSources() const;

  /// Answers `request`. `body_tree`: the robot's frame tree (body, odom,
  /// vision, ...) at the time of the request; camera frames are added.
  /// Waits up to `first_frame_timeout` for cameras that have not rendered
  /// yet (simulation startup).
  bosdyn::api::GetImageResponse GetImage(const bosdyn::api::GetImageRequest& request,
                                         const bosdyn::api::FrameTreeSnapshot& body_tree,
                                         std::chrono::milliseconds first_frame_timeout = std::chrono::seconds(2)) const;

 private:
  enum class Kind { kVisual, kDepth, kDepthInVisual };
  struct Camera {
    Pose body_T_visual;
    Pose body_T_depth;
    // Shared so requests can encode outside the lock.
    std::shared_ptr<const VisualFrame> visual;
    std::shared_ptr<const DepthFrame> depth;
  };
  using Cameras = std::map<std::string, Camera>;
  struct Source {
    std::string camera;
    Kind kind;
  };

  static std::optional<Source> ParseSource(const Cameras& cameras, const std::string& source_name);
  static bool HasData(const Camera& camera, Kind kind);
  static bosdyn::api::ImageSource DescribeSource(const std::string& source_name, const Source& source,
                                                 const Camera& camera);
  static void AddCameraFrames(bosdyn::api::FrameTreeSnapshot* tree, const std::string& name, const Camera& camera);
  static bosdyn::api::ImageResponse Respond(const Cameras& cameras, const bosdyn::api::ImageRequest& request,
                                            const bosdyn::api::FrameTreeSnapshot& body_tree);

  mutable std::mutex mutex_;
  mutable std::condition_variable frame_arrived_;
  Cameras cameras_;
};

/// Source name helpers.
std::string VisualSourceName(const std::string& camera);
std::string DepthSourceName(const std::string& camera);
std::string DepthInVisualSourceName(const std::string& camera);
/// Frame names: "<cam>_fisheye" (visual) and "<cam>" (depth).
std::string VisualFrameName(const std::string& camera);
std::string DepthFrameName(const std::string& camera);

}  // namespace spot_sim
