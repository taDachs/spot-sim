#include "spot_sim_core/images.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include <jpeglib.h>

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr const char* kVisualSuffix = "_fisheye_image";
constexpr const char* kDepthSuffix = "_depth";
constexpr const char* kDepthInVisualSuffix = "_depth_in_visual_frame";
constexpr double kDepthScale = 1000.0;  // DEPTH_U16 units per meter
constexpr int kDefaultJpegQuality = 75;

/// The requested pixel format followed by the fallbacks.
std::vector<api::Image::PixelFormat> PixelFormatPreferences(const api::ImageRequest& request) {
  std::vector<api::Image::PixelFormat> out{request.pixel_format()};
  for (const int format : request.fallback_formats()) {
    out.push_back(static_cast<api::Image::PixelFormat>(format));
  }
  return out;
}

void AddEdge(api::FrameTreeSnapshot* tree, const std::string& child, const std::string& parent,
             const Pose& parent_T_child) {
  auto& edge = (*tree->mutable_child_to_parent_edge_map())[child];
  edge.set_parent_frame_name(parent);
  *edge.mutable_parent_tform_child() = ToProto(parent_T_child);
}

std::string GreyscaleJpegOrRaw(const VisualFrame& frame, bool jpeg, int quality) {
  std::vector<uint8_t> grey(frame.geometry.pixels());
  for (std::size_t i = 0; i < grey.size(); ++i) {
    const uint8_t* p = &frame.rgb[3 * i];
    grey[i] = static_cast<uint8_t>(std::lround(0.299 * p[0] + 0.587 * p[1] + 0.114 * p[2]));
  }
  if (jpeg) {
    return EncodeJpeg(grey.data(), frame.geometry.width, frame.geometry.height, 1, quality);
  }
  return std::string(grey.begin(), grey.end());
}
}  // namespace

std::string VisualSourceName(const std::string& camera) { return camera + kVisualSuffix; }
std::string DepthSourceName(const std::string& camera) { return camera + kDepthSuffix; }
std::string DepthInVisualSourceName(const std::string& camera) { return camera + kDepthInVisualSuffix; }
std::string VisualFrameName(const std::string& camera) { return camera + "_fisheye"; }
std::string DepthFrameName(const std::string& camera) { return camera; }

CameraSpec CameraSpec::Body(const std::string& camera, const Pose& body_T_visual, const Pose& body_T_depth) {
  CameraSpec spec;
  spec.visual_source = VisualSourceName(camera);
  spec.depth_source = DepthSourceName(camera);
  spec.depth_in_visual_source = DepthInVisualSourceName(camera);
  spec.visual_frame = VisualFrameName(camera);
  spec.depth_frame = DepthFrameName(camera);
  spec.mount_T_visual = body_T_visual;
  spec.mount_T_depth = body_T_depth;
  return spec;
}

CameraSpec CameraSpec::Hand(const Pose& wrist_T_visual, const Pose& wrist_T_depth) {
  CameraSpec spec;
  spec.visual_source = "hand_color_image";
  spec.depth_source = "hand_depth";
  spec.depth_in_visual_source = "hand_depth_in_hand_color_frame";
  spec.visual_frame = "hand_color_image_sensor";
  spec.depth_frame = "hand_depth_sensor";
  spec.mount_frame = "arm0.link_wr1";
  spec.mount_T_visual = wrist_T_visual;
  spec.mount_T_depth = wrist_T_depth;
  return spec;
}

PinholeIntrinsics PinholeIntrinsics::FromHorizontalFov(int width, int height, double hfov) {
  const double f = 0.5 * width / std::tan(0.5 * hfov);
  return {f, f, 0.5 * width, 0.5 * height};
}

DepthFrame RegisterDepth(const DepthFrame& depth, const Pose& target_T_depth, const ImageGeometry& target) {
  DepthFrame out;
  out.stamp = depth.stamp;
  out.geometry = target;
  out.depth_mm.assign(target.pixels(), 0);
  const int width = target.width, height = target.height;
  const PinholeIntrinsics& k = target.intrinsics;
  const auto& in = depth.geometry.intrinsics;
  // Point of depth pixel (u, v) at range z [m], in the target frame.
  const auto to_target = [&](double u, double v, double z) {
    return target_T_depth * Pose{Vec3{(u - in.cx) * z / in.fx, (v - in.cy) * z / in.fy, z}, Quat{}};
  };
  for (int v = 0; v < depth.geometry.height; ++v) {
    for (int u = 0; u < depth.geometry.width; ++u) {
      const uint16_t mm = depth.depth_mm[static_cast<std::size_t>(v) * depth.geometry.width + u];
      if (mm == 0) {
        continue;
      }
      const double z = mm / 1000.0;
      const Vec3 center = to_target(u, v, z).position;
      if (center.z <= 0.0) {
        continue;
      }
      const double new_mm = std::round(center.z * 1000.0);
      if (new_mm < 1.0 || new_mm > 65535.0) {
        continue;
      }
      // The pixel's footprint: project its corners.
      const Vec3 a = to_target(u - 0.5, v - 0.5, z).position;
      const Vec3 b = to_target(u + 0.5, v + 0.5, z).position;
      if (a.z <= 0.0 || b.z <= 0.0) {
        continue;
      }
      const double ua = k.fx * a.x / a.z + k.cx, va = k.fy * a.y / a.z + k.cy;
      const double ub = k.fx * b.x / b.z + k.cx, vb = k.fy * b.y / b.z + k.cy;
      int u0 = static_cast<int>(std::floor(std::min(ua, ub) + 0.5));
      int u1 = static_cast<int>(std::floor(std::max(ua, ub) - 0.5));
      int v0 = static_cast<int>(std::floor(std::min(va, vb) + 0.5));
      int v1 = static_cast<int>(std::floor(std::max(va, vb) - 0.5));
      u1 = std::max(u1, u0);
      v1 = std::max(v1, v0);
      for (int y = std::max(v0, 0); y <= std::min(v1, height - 1); ++y) {
        for (int x = std::max(u0, 0); x <= std::min(u1, width - 1); ++x) {
          auto& px = out.depth_mm[static_cast<std::size_t>(y) * width + x];
          if (px == 0 || new_mm < px) {
            px = static_cast<uint16_t>(new_mm);
          }
        }
      }
    }
  }
  return out;
}

std::vector<uint16_t> DepthToMillimeters(const float* meters, std::size_t count) {
  std::vector<uint16_t> out(count, 0);
  for (std::size_t i = 0; i < count; ++i) {
    const double mm = std::round(static_cast<double>(meters[i]) * kDepthScale);
    if (std::isfinite(mm) && mm > 0.0 && mm <= 65535.0) {
      out[i] = static_cast<uint16_t>(mm);
    }
  }
  return out;
}

std::string EncodeJpeg(const uint8_t* pixels, int width, int height, int channels, int quality) {
  jpeg_compress_struct cinfo{};
  jpeg_error_mgr jerr{};
  cinfo.err = jpeg_std_error(&jerr);
  jerr.error_exit = [](j_common_ptr info) {
    char message[JMSG_LENGTH_MAX];
    (*info->err->format_message)(info, message);
    throw std::runtime_error(std::string("JPEG encoding failed: ") + message);
  };
  unsigned char* buffer = nullptr;
  unsigned long size = 0;
  try {
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &buffer, &size);
    cinfo.image_width = static_cast<JDIMENSION>(width);
    cinfo.image_height = static_cast<JDIMENSION>(height);
    cinfo.input_components = channels;
    cinfo.in_color_space = channels == 1 ? JCS_GRAYSCALE : JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, std::clamp(quality, 1, 100), TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    const std::size_t stride = static_cast<std::size_t>(width) * channels;
    while (cinfo.next_scanline < cinfo.image_height) {
      JSAMPROW row = const_cast<JSAMPROW>(pixels + cinfo.next_scanline * stride);
      jpeg_write_scanlines(&cinfo, &row, 1);
    }
    jpeg_finish_compress(&cinfo);
  } catch (...) {
    jpeg_destroy_compress(&cinfo);
    std::free(buffer);
    throw;
  }
  std::string out(reinterpret_cast<const char*>(buffer), size);
  jpeg_destroy_compress(&cinfo);
  std::free(buffer);
  return out;
}

void BodyCameras::AddCamera(const std::string& name, const CameraSpec& spec) {
  std::lock_guard<std::mutex> lock(mutex_);
  cameras_[name].spec = spec;
}

void BodyCameras::SetVisualFrame(const std::string& name, VisualFrame frame) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = cameras_.find(name);
    if (it == cameras_.end()) {
      return;
    }
    it->second.visual = std::make_shared<const VisualFrame>(std::move(frame));
  }
  frame_arrived_.notify_all();
}

void BodyCameras::SetDepthFrame(const std::string& name, DepthFrame frame) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = cameras_.find(name);
    if (it == cameras_.end()) {
      return;
    }
    it->second.depth = std::make_shared<const DepthFrame>(std::move(frame));
  }
  frame_arrived_.notify_all();
}

bool BodyCameras::HasCamera(const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cameras_.count(name) > 0;
}

std::optional<BodyCameras::Source> BodyCameras::ParseSource(const Cameras& cameras, const std::string& name) {
  for (const auto& [camera_name, camera] : cameras) {
    const auto& spec = camera.spec;
    if (name == spec.visual_source) return Source{camera_name, Kind::kVisual};
    if (name == spec.depth_source) return Source{camera_name, Kind::kDepth};
    if (name == spec.depth_in_visual_source) return Source{camera_name, Kind::kDepthInVisual};
  }
  return std::nullopt;
}

bool BodyCameras::HasData(const Camera& camera, Kind kind) {
  switch (kind) {
    case Kind::kVisual:
      return camera.visual != nullptr;
    case Kind::kDepth:
      return camera.depth != nullptr;
    case Kind::kDepthInVisual:
      return camera.visual != nullptr && camera.depth != nullptr;
  }
  return false;
}

api::ImageSource BodyCameras::DescribeSource(const std::string& source_name, const Source& source,
                                             const Camera& camera) {
  api::ImageSource out;
  out.set_name(source_name);
  // Visual and registered depth images share the visual camera's geometry.
  std::optional<ImageGeometry> geometry;
  if (source.kind == Kind::kDepth && camera.depth) {
    geometry = camera.depth->geometry;
  } else if (source.kind != Kind::kDepth && camera.visual) {
    geometry = camera.visual->geometry;
  }
  if (geometry) {
    const PinholeIntrinsics& k = geometry->intrinsics;
    out.set_cols(geometry->width);
    out.set_rows(geometry->height);
    auto* intrinsics = out.mutable_pinhole()->mutable_intrinsics();
    intrinsics->mutable_focal_length()->set_x(k.fx);
    intrinsics->mutable_focal_length()->set_y(k.fy);
    intrinsics->mutable_principal_point()->set_x(k.cx);
    intrinsics->mutable_principal_point()->set_y(k.cy);
    intrinsics->mutable_skew();
  }
  if (source.kind == Kind::kVisual) {
    out.set_image_type(api::ImageSource::IMAGE_TYPE_VISUAL);
    out.add_pixel_formats(api::Image::PIXEL_FORMAT_RGB_U8);
    out.add_pixel_formats(api::Image::PIXEL_FORMAT_GREYSCALE_U8);
    out.add_image_formats(api::Image::FORMAT_JPEG);
    out.add_image_formats(api::Image::FORMAT_RAW);
  } else {
    out.set_image_type(api::ImageSource::IMAGE_TYPE_DEPTH);
    out.set_depth_scale(kDepthScale);
    out.add_pixel_formats(api::Image::PIXEL_FORMAT_DEPTH_U16);
    out.add_image_formats(api::Image::FORMAT_RAW);
  }
  return out;
}

void BodyCameras::AddCameraFrames(api::FrameTreeSnapshot* tree, const Camera& camera,
                                  const std::optional<Pose>& body_T_mount) {
  const auto& spec = camera.spec;
  if (spec.mount_frame != kBodyFrame && body_T_mount) {
    AddEdge(tree, spec.mount_frame, kBodyFrame, *body_T_mount);
  }
  AddEdge(tree, spec.visual_frame, spec.mount_frame, spec.mount_T_visual);
  AddEdge(tree, spec.depth_frame, spec.visual_frame, spec.mount_T_visual.Inverse() * spec.mount_T_depth);
}

api::ListImageSourcesResponse BodyCameras::ListImageSources() const {
  std::lock_guard<std::mutex> lock(mutex_);
  api::ListImageSourcesResponse response;
  auto* tree = response.mutable_transforms_snapshot();
  (*tree->mutable_child_to_parent_edge_map())[kBodyFrame];
  for (const auto& [name, camera] : cameras_) {
    const auto& spec = camera.spec;
    for (const auto& [source_name, kind] :
         {std::pair{spec.visual_source, Kind::kVisual}, std::pair{spec.depth_source, Kind::kDepth},
          std::pair{spec.depth_in_visual_source, Kind::kDepthInVisual}}) {
      *response.add_image_sources() = DescribeSource(source_name, Source{name, kind}, camera);
    }
    AddCameraFrames(tree, camera, camera.visual ? camera.visual->body_T_mount : std::nullopt);
  }
  return response;
}

api::ImageResponse BodyCameras::Respond(const Cameras& cameras, const api::ImageRequest& request,
                                        const api::FrameTreeSnapshot& body_tree) {
  api::ImageResponse response;
  const auto source = ParseSource(cameras, request.image_source_name());
  if (!source) {
    response.mutable_source()->set_name(request.image_source_name());
    response.set_status(api::ImageResponse::STATUS_UNKNOWN_CAMERA);
    return response;
  }
  const Camera& camera = cameras.at(source->camera);
  *response.mutable_source() = DescribeSource(request.image_source_name(), *source, camera);
  const auto fail = [&](api::ImageResponse::Status status) {
    response.set_status(status);
    return response;
  };
  if (!HasData(camera, source->kind)) {
    return fail(api::ImageResponse::STATUS_SOURCE_DATA_ERROR);
  }
  if (request.resize_ratio() != 0.0 && request.resize_ratio() != 1.0) {
    return fail(api::ImageResponse::STATUS_UNSUPPORTED_RESIZE_RATIO_REQUESTED);
  }
  auto* shot = response.mutable_shot();
  auto* image = shot->mutable_image();

  if (source->kind == Kind::kVisual) {
    const VisualFrame& frame = *camera.visual;
    auto pixel_format = api::Image::PIXEL_FORMAT_UNKNOWN;
    for (const auto format : PixelFormatPreferences(request)) {
      if (format == api::Image::PIXEL_FORMAT_UNKNOWN || format == api::Image::PIXEL_FORMAT_RGB_U8) {
        pixel_format = api::Image::PIXEL_FORMAT_RGB_U8;
        break;
      }
      if (format == api::Image::PIXEL_FORMAT_GREYSCALE_U8) {
        pixel_format = format;
        break;
      }
    }
    if (pixel_format == api::Image::PIXEL_FORMAT_UNKNOWN) {
      return fail(api::ImageResponse::STATUS_UNSUPPORTED_PIXEL_FORMAT_REQUESTED);
    }
    const auto format = request.image_format() == api::Image::FORMAT_UNKNOWN ? api::Image::FORMAT_JPEG
                                                                              : request.image_format();
    if (format != api::Image::FORMAT_JPEG && format != api::Image::FORMAT_RAW) {
      return fail(api::ImageResponse::STATUS_UNSUPPORTED_IMAGE_FORMAT_REQUESTED);
    }
    const int quality =
        request.quality_percent() > 0.0 ? static_cast<int>(std::lround(request.quality_percent())) : kDefaultJpegQuality;
    const bool jpeg = format == api::Image::FORMAT_JPEG;
    if (pixel_format == api::Image::PIXEL_FORMAT_RGB_U8) {
      image->set_data(jpeg ? EncodeJpeg(frame.rgb.data(), frame.geometry.width, frame.geometry.height, 3, quality)
                           : std::string(frame.rgb.begin(), frame.rgb.end()));
    } else {
      image->set_data(GreyscaleJpegOrRaw(frame, jpeg, quality));
    }
    image->set_cols(frame.geometry.width);
    image->set_rows(frame.geometry.height);
    image->set_format(format);
    image->set_pixel_format(pixel_format);
    *shot->mutable_acquisition_time() = ToTimestamp(frame.stamp);
    shot->set_frame_name_image_sensor(camera.spec.visual_frame);
  } else {
    const auto preferences = PixelFormatPreferences(request);
    const bool depth_ok = std::any_of(preferences.begin(), preferences.end(), [](api::Image::PixelFormat f) {
      return f == api::Image::PIXEL_FORMAT_UNKNOWN || f == api::Image::PIXEL_FORMAT_DEPTH_U16;
    });
    if (!depth_ok) {
      return fail(api::ImageResponse::STATUS_UNSUPPORTED_PIXEL_FORMAT_REQUESTED);
    }
    if (request.image_format() != api::Image::FORMAT_UNKNOWN && request.image_format() != api::Image::FORMAT_RAW) {
      return fail(api::ImageResponse::STATUS_UNSUPPORTED_IMAGE_FORMAT_REQUESTED);
    }
    std::optional<DepthFrame> registered;
    if (source->kind == Kind::kDepthInVisual) {
      const Pose visual_T_depth = camera.spec.mount_T_visual.Inverse() * camera.spec.mount_T_depth;
      registered = RegisterDepth(*camera.depth, visual_T_depth, camera.visual->geometry);
    }
    const DepthFrame& frame = registered ? *registered : *camera.depth;
    // Little-endian 16-bit millimeters, as on the robot.
    std::string data(frame.depth_mm.size() * 2, '\0');
    for (std::size_t i = 0; i < frame.depth_mm.size(); ++i) {
      data[2 * i] = static_cast<char>(frame.depth_mm[i] & 0xff);
      data[2 * i + 1] = static_cast<char>(frame.depth_mm[i] >> 8);
    }
    image->set_data(std::move(data));
    image->set_cols(frame.geometry.width);
    image->set_rows(frame.geometry.height);
    image->set_format(api::Image::FORMAT_RAW);
    image->set_pixel_format(api::Image::PIXEL_FORMAT_DEPTH_U16);
    *shot->mutable_acquisition_time() = ToTimestamp(frame.stamp);
    shot->set_frame_name_image_sensor(source->kind == Kind::kDepth ? camera.spec.depth_frame
                                                                   : camera.spec.visual_frame);
  }

  auto* tree = shot->mutable_transforms_snapshot();
  *tree = body_tree;
  // The mount where it was when the returned image was taken.
  const auto& body_T_mount = source->kind == Kind::kVisual ? camera.visual->body_T_mount : camera.depth->body_T_mount;
  AddCameraFrames(tree, camera, body_T_mount);
  response.set_status(api::ImageResponse::STATUS_OK);
  return response;
}

api::GetImageResponse BodyCameras::GetImage(const api::GetImageRequest& request, const api::FrameTreeSnapshot& body_tree,
                                            std::chrono::milliseconds first_frame_timeout) const {
  Cameras cameras;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Right after startup the renderer may not have produced images yet.
    const auto all_rendered = [&] {
      for (const auto& image_request : request.image_requests()) {
        const auto source = ParseSource(cameras_, image_request.image_source_name());
        if (source && !HasData(cameras_.at(source->camera), source->kind)) {
          return false;
        }
      }
      return true;
    };
    frame_arrived_.wait_for(lock, first_frame_timeout, all_rendered);
    cameras = cameras_;
  }
  api::GetImageResponse response;
  for (const auto& image_request : request.image_requests()) {
    *response.add_image_responses() = Respond(cameras, image_request, body_tree);
  }
  return response;
}

}  // namespace spot_sim
