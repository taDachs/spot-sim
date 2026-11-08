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

bool EndsWith(const std::string& s, const std::string& suffix) {
  return s.size() > suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

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

std::string GreyscaleJpegOrRaw(const CameraFrame& frame, bool jpeg, int quality) {
  std::vector<uint8_t> grey(static_cast<std::size_t>(frame.width) * frame.height);
  for (std::size_t i = 0; i < grey.size(); ++i) {
    const uint8_t* p = &frame.rgb[3 * i];
    grey[i] = static_cast<uint8_t>(std::lround(0.299 * p[0] + 0.587 * p[1] + 0.114 * p[2]));
  }
  if (jpeg) {
    return EncodeJpeg(grey.data(), frame.width, frame.height, 1, quality);
  }
  return std::string(grey.begin(), grey.end());
}
}  // namespace

std::string VisualSourceName(const std::string& camera) { return camera + kVisualSuffix; }
std::string DepthSourceName(const std::string& camera) { return camera + kDepthSuffix; }
std::string DepthInVisualSourceName(const std::string& camera) { return camera + kDepthInVisualSuffix; }
std::string VisualFrameName(const std::string& camera) { return camera + "_fisheye"; }
std::string DepthFrameName(const std::string& camera) { return camera; }

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

void BodyCameras::AddCamera(const std::string& name, const Pose& body_T_sensor) {
  std::lock_guard<std::mutex> lock(mutex_);
  cameras_[name].body_T_sensor = body_T_sensor;
}

void BodyCameras::SetFrame(const std::string& name, CameraFrame frame) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = cameras_.find(name);
    if (it == cameras_.end()) {
      return;
    }
    it->second.frame = std::make_shared<const CameraFrame>(std::move(frame));
  }
  frame_arrived_.notify_all();
}

bool BodyCameras::HasCamera(const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cameras_.count(name) > 0;
}

std::optional<BodyCameras::Source> BodyCameras::ParseSource(const Cameras& cameras, const std::string& name) {
  for (const auto& [suffix, kind] : {std::pair{kDepthInVisualSuffix, Kind::kDepthInVisual},
                                     std::pair{kVisualSuffix, Kind::kVisual}, std::pair{kDepthSuffix, Kind::kDepth}}) {
    if (EndsWith(name, suffix)) {
      const std::string camera = name.substr(0, name.size() - std::string(suffix).size());
      if (cameras.count(camera) > 0) {
        return Source{camera, kind};
      }
      return std::nullopt;
    }
  }
  return std::nullopt;
}

api::ImageSource BodyCameras::DescribeSource(const std::string& source_name, const Source& source,
                                             const Camera& camera) {
  api::ImageSource out;
  out.set_name(source_name);
  if (camera.frame) {
    out.set_cols(camera.frame->width);
    out.set_rows(camera.frame->height);
    auto* intrinsics = out.mutable_pinhole()->mutable_intrinsics();
    intrinsics->mutable_focal_length()->set_x(camera.frame->intrinsics.fx);
    intrinsics->mutable_focal_length()->set_y(camera.frame->intrinsics.fy);
    intrinsics->mutable_principal_point()->set_x(camera.frame->intrinsics.cx);
    intrinsics->mutable_principal_point()->set_y(camera.frame->intrinsics.cy);
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

api::ListImageSourcesResponse BodyCameras::ListImageSources() const {
  std::lock_guard<std::mutex> lock(mutex_);
  api::ListImageSourcesResponse response;
  for (const auto& [name, camera] : cameras_) {
    for (const auto& [source_name, kind] :
         {std::pair{VisualSourceName(name), Kind::kVisual}, std::pair{DepthSourceName(name), Kind::kDepth},
          std::pair{DepthInVisualSourceName(name), Kind::kDepthInVisual}}) {
      *response.add_image_sources() = DescribeSource(source_name, Source{name, kind}, camera);
    }
    auto* tree = response.mutable_transforms_snapshot();
    (*tree->mutable_child_to_parent_edge_map())["body"];
    AddEdge(tree, VisualFrameName(name), "body", camera.body_T_sensor);
    AddEdge(tree, DepthFrameName(name), VisualFrameName(name), Pose{});
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
  if (!camera.frame) {
    return fail(api::ImageResponse::STATUS_SOURCE_DATA_ERROR);
  }
  if (request.resize_ratio() != 0.0 && request.resize_ratio() != 1.0) {
    return fail(api::ImageResponse::STATUS_UNSUPPORTED_RESIZE_RATIO_REQUESTED);
  }
  const CameraFrame& frame = *camera.frame;
  auto* image = response.mutable_shot()->mutable_image();
  image->set_cols(frame.width);
  image->set_rows(frame.height);

  if (source->kind == Kind::kVisual) {
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
      image->set_data(jpeg ? EncodeJpeg(frame.rgb.data(), frame.width, frame.height, 3, quality)
                           : std::string(frame.rgb.begin(), frame.rgb.end()));
    } else {
      image->set_data(GreyscaleJpegOrRaw(frame, jpeg, quality));
    }
    image->set_format(format);
    image->set_pixel_format(pixel_format);
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
    // Little-endian 16-bit millimeters, as on the robot.
    std::string data(frame.depth_mm.size() * 2, '\0');
    for (std::size_t i = 0; i < frame.depth_mm.size(); ++i) {
      data[2 * i] = static_cast<char>(frame.depth_mm[i] & 0xff);
      data[2 * i + 1] = static_cast<char>(frame.depth_mm[i] >> 8);
    }
    image->set_data(std::move(data));
    image->set_format(api::Image::FORMAT_RAW);
    image->set_pixel_format(api::Image::PIXEL_FORMAT_DEPTH_U16);
  }

  auto* shot = response.mutable_shot();
  *shot->mutable_acquisition_time() = ToTimestamp(frame.stamp);
  shot->set_frame_name_image_sensor(source->kind == Kind::kDepth ? DepthFrameName(source->camera)
                                                                 : VisualFrameName(source->camera));
  auto* tree = shot->mutable_transforms_snapshot();
  *tree = body_tree;
  AddEdge(tree, VisualFrameName(source->camera), "body", camera.body_T_sensor);
  AddEdge(tree, DepthFrameName(source->camera), VisualFrameName(source->camera), Pose{});
  response.set_status(api::ImageResponse::STATUS_OK);
  return response;
}

api::GetImageResponse BodyCameras::GetImage(const api::GetImageRequest& request, const api::FrameTreeSnapshot& body_tree,
                                            std::chrono::milliseconds first_frame_timeout) const {
  Cameras cameras;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Right after startup the renderer may not have produced a frame yet.
    const auto all_rendered = [&] {
      for (const auto& image_request : request.image_requests()) {
        const auto source = ParseSource(cameras_, image_request.image_source_name());
        if (source && !cameras_.at(source->camera).frame) {
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
