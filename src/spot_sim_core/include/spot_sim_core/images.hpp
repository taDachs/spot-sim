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
};

/// The latest frame of one simulated RGB-D body camera.
struct CameraFrame {
  RobotTime stamp{0};
  int width = 0, height = 0;
  PinholeIntrinsics intrinsics;
  std::vector<uint8_t> rgb;        ///< width * height * 3, row-major, top row first
  std::vector<uint16_t> depth_mm;  ///< width * height, 0 = no return
};

/// Depth image conversion: meters (float, inf / nan for no return) to the
/// robot's DEPTH_U16 millimeters. Values outside (0, 65.535] m become 0.
std::vector<uint16_t> DepthToMillimeters(const float* meters, std::size_t count);

/// JPEG encoding of 8-bit RGB or greyscale pixels.
std::string EncodeJpeg(const uint8_t* pixels, int width, int height, int channels, int quality);

/// ImageService semantics for Spot's body cameras: each camera `<cam>` (e.g.
/// "frontleft") provides the sources
///   <cam>_fisheye_image          visual, RGB_U8 or GREYSCALE_U8, JPEG or RAW
///   <cam>_depth                  depth, DEPTH_U16 RAW, in frame "<cam>"
///   <cam>_depth_in_visual_frame  depth registered to the visual image
/// The simulated camera is a pinhole RGB-D sensor, so visual and depth share
/// one viewpoint and resolution; frame "<cam>" coincides with
/// "<cam>_fisheye". Thread-safe.
class BodyCameras {
 public:
  /// Registers a camera. `body_T_sensor`: optical frame (z forward, x right,
  /// y down) in the body frame.
  void AddCamera(const std::string& name, const Pose& body_T_sensor);

  /// Publishes a new frame of `name` (ignored for unknown cameras).
  void SetFrame(const std::string& name, CameraFrame frame);

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
    Pose body_T_sensor;
    std::shared_ptr<const CameraFrame> frame;  ///< shared so requests can encode outside the lock
  };
  using Cameras = std::map<std::string, Camera>;
  struct Source {
    std::string camera;
    Kind kind;
  };

  static std::optional<Source> ParseSource(const Cameras& cameras, const std::string& source_name);
  static bosdyn::api::ImageSource DescribeSource(const std::string& source_name, const Source& source,
                                                 const Camera& camera);
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
