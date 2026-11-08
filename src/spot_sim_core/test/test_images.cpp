#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <thread>

#include <jpeglib.h>

#include "spot_sim_core/images.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;
using spot_sim::BodyCameras;
using spot_sim::DepthFrame;
using spot_sim::Pose;
using spot_sim::Quat;
using spot_sim::VisualFrame;

namespace {
constexpr int kWidth = 8, kHeight = 6;            // visual
constexpr int kDepthWidth = 4, kDepthHeight = 3;  // depth sensor

VisualFrame TestVisual(spot_sim::RobotTime stamp = 7s) {
  VisualFrame frame;
  frame.stamp = stamp;
  frame.geometry = {kWidth, kHeight, {5.0, 5.5, 4.0, 3.0}};
  for (int i = 0; i < kWidth * kHeight; ++i) {
    frame.rgb.insert(frame.rgb.end(), {200, 100, 50});
  }
  return frame;
}

DepthFrame TestDepth(spot_sim::RobotTime stamp = 7s) {
  DepthFrame frame;
  frame.stamp = stamp;
  frame.geometry = {kDepthWidth, kDepthHeight, {2.5, 2.5, 2.0, 1.5}};
  for (int i = 0; i < kDepthWidth * kDepthHeight; ++i) {
    frame.depth_mm.push_back(static_cast<uint16_t>(1000 + i));
  }
  return frame;
}

const Pose kMount{{0.4, 0.05, -0.02}, Quat::FromRpy(-M_PI / 2, 0.0, -M_PI / 2)};
// Depth sensor 2 cm to the right of the visual camera (optical x).
const Pose kDepthMount = kMount * Pose{{0.02, 0.0, 0.0}, Quat{}};

std::unique_ptr<BodyCameras> Cameras() {
  auto cameras = std::make_unique<BodyCameras>();
  cameras->AddCamera("frontleft", kMount, kDepthMount);
  cameras->SetVisualFrame("frontleft", TestVisual());
  cameras->SetDepthFrame("frontleft", TestDepth());
  return cameras;
}

/// A fronto-parallel plane `distance` in front of a camera.
DepthFrame Plane(int width, int height, const spot_sim::PinholeIntrinsics& k, double distance) {
  DepthFrame frame;
  frame.geometry = {width, height, k};
  frame.depth_mm.assign(frame.geometry.pixels(), static_cast<uint16_t>(distance * 1000));
  return frame;
}

api::GetImageRequest Request(const std::string& source, api::Image::Format format = api::Image::FORMAT_UNKNOWN,
                             api::Image::PixelFormat pixel_format = api::Image::PIXEL_FORMAT_UNKNOWN) {
  api::GetImageRequest request;
  auto* image_request = request.add_image_requests();
  image_request->set_image_source_name(source);
  image_request->set_image_format(format);
  image_request->set_pixel_format(pixel_format);
  image_request->set_quality_percent(90);
  return request;
}

api::FrameTreeSnapshot BodyTree() {
  api::FrameTreeSnapshot tree;
  (*tree.mutable_child_to_parent_edge_map())["body"];
  auto& odom = (*tree.mutable_child_to_parent_edge_map())["odom"];
  odom.set_parent_frame_name("body");
  odom.mutable_parent_tform_child()->mutable_position()->set_z(-0.52);
  odom.mutable_parent_tform_child()->mutable_rotation()->set_w(1.0);
  return tree;
}

api::ImageResponse Get(const BodyCameras& cameras, const api::GetImageRequest& request) {
  const auto response = cameras.GetImage(request, BodyTree(), 10ms);
  EXPECT_EQ(response.image_responses_size(), 1);
  return response.image_responses(0);
}

std::vector<uint8_t> DecodeJpeg(const std::string& data, int& width, int& height, int& channels) {
  jpeg_decompress_struct cinfo{};
  jpeg_error_mgr jerr{};
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, reinterpret_cast<const unsigned char*>(data.data()), data.size());
  jpeg_read_header(&cinfo, TRUE);
  jpeg_start_decompress(&cinfo);
  width = static_cast<int>(cinfo.output_width);
  height = static_cast<int>(cinfo.output_height);
  channels = cinfo.output_components;
  std::vector<uint8_t> pixels(static_cast<std::size_t>(width) * height * channels);
  while (cinfo.output_scanline < cinfo.output_height) {
    JSAMPROW row = pixels.data() + cinfo.output_scanline * width * channels;
    jpeg_read_scanlines(&cinfo, &row, 1);
  }
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return pixels;
}
}  // namespace

TEST(Images, DepthToMillimeters) {
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> meters = {1.2345f, 0.0f, -1.0f, inf, nan, 70.0f, 0.0004f};
  const auto mm = spot_sim::DepthToMillimeters(meters.data(), meters.size());
  EXPECT_EQ(mm, (std::vector<uint16_t>{1235, 0, 0, 0, 0, 0, 0}));
}

TEST(Images, ListsThreeSourcesPerCamera) {
  const auto response = Cameras()->ListImageSources();
  ASSERT_EQ(response.image_sources_size(), 3);
  std::map<std::string, api::ImageSource> sources;
  for (const auto& source : response.image_sources()) sources[source.name()] = source;
  ASSERT_TRUE(sources.count("frontleft_fisheye_image"));
  ASSERT_TRUE(sources.count("frontleft_depth"));
  ASSERT_TRUE(sources.count("frontleft_depth_in_visual_frame"));
  EXPECT_EQ(sources["frontleft_fisheye_image"].image_type(), api::ImageSource::IMAGE_TYPE_VISUAL);
  EXPECT_EQ(sources["frontleft_depth"].image_type(), api::ImageSource::IMAGE_TYPE_DEPTH);
  EXPECT_EQ(sources["frontleft_depth"].depth_scale(), 1000.0);
  EXPECT_EQ(sources["frontleft_fisheye_image"].cols(), kWidth);
  EXPECT_EQ(sources["frontleft_fisheye_image"].pinhole().intrinsics().focal_length().y(), 5.5);
  // The depth sensor has its own, lower resolution; registered depth has the visual one.
  EXPECT_EQ(sources["frontleft_depth"].cols(), kDepthWidth);
  EXPECT_EQ(sources["frontleft_depth"].pinhole().intrinsics().focal_length().x(), 2.5);
  EXPECT_EQ(sources["frontleft_depth_in_visual_frame"].cols(), kWidth);
  EXPECT_TRUE(response.transforms_snapshot().child_to_parent_edge_map().count("frontleft_fisheye"));
  EXPECT_TRUE(response.transforms_snapshot().child_to_parent_edge_map().count("frontleft"));
}

TEST(Images, VisualDefaultsToRgbJpeg) {
  const auto response = Get(*Cameras(), Request("frontleft_fisheye_image"));
  ASSERT_EQ(response.status(), api::ImageResponse::STATUS_OK);
  const auto& image = response.shot().image();
  EXPECT_EQ(image.format(), api::Image::FORMAT_JPEG);
  EXPECT_EQ(image.pixel_format(), api::Image::PIXEL_FORMAT_RGB_U8);
  int width = 0, height = 0, channels = 0;
  const auto pixels = DecodeJpeg(image.data(), width, height, channels);
  EXPECT_EQ(width, kWidth);
  EXPECT_EQ(height, kHeight);
  EXPECT_EQ(channels, 3);
  EXPECT_NEAR(pixels[0], 200, 6);
  EXPECT_NEAR(pixels[1], 100, 6);
  EXPECT_NEAR(pixels[2], 50, 6);

  EXPECT_EQ(response.shot().frame_name_image_sensor(), "frontleft_fisheye");
  EXPECT_EQ(response.shot().acquisition_time().seconds(), 7);
  EXPECT_EQ(response.source().pinhole().intrinsics().principal_point().x(), 4.0);
  // The frame tree links the sensor to the body and keeps the robot's frames.
  const auto& edges = response.shot().transforms_snapshot().child_to_parent_edge_map();
  ASSERT_TRUE(edges.count("frontleft_fisheye"));
  EXPECT_EQ(edges.at("frontleft_fisheye").parent_frame_name(), "body");
  EXPECT_NEAR(edges.at("frontleft_fisheye").parent_tform_child().position().x(), 0.4, 1e-12);
  EXPECT_TRUE(edges.count("odom"));
}

TEST(Images, GreyscaleRaw) {
  const auto response =
      Get(*Cameras(), Request("frontleft_fisheye_image", api::Image::FORMAT_RAW, api::Image::PIXEL_FORMAT_GREYSCALE_U8));
  ASSERT_EQ(response.status(), api::ImageResponse::STATUS_OK);
  const auto& image = response.shot().image();
  ASSERT_EQ(image.data().size(), static_cast<std::size_t>(kWidth * kHeight));
  EXPECT_EQ(static_cast<uint8_t>(image.data()[0]), 124);  // 0.299*200 + 0.587*100 + 0.114*50
}

TEST(Images, DepthIsTheDepthSensorsImage) {
  const auto response = Get(*Cameras(), Request("frontleft_depth", api::Image::FORMAT_RAW));
  ASSERT_EQ(response.status(), api::ImageResponse::STATUS_OK);
  const auto& image = response.shot().image();
  EXPECT_EQ(image.pixel_format(), api::Image::PIXEL_FORMAT_DEPTH_U16);
  EXPECT_EQ(image.cols(), kDepthWidth);
  EXPECT_EQ(image.rows(), kDepthHeight);
  ASSERT_EQ(image.data().size(), static_cast<std::size_t>(2 * kDepthWidth * kDepthHeight));
  const auto value = [&](int i) {
    return static_cast<uint8_t>(image.data()[2 * i]) | (static_cast<uint8_t>(image.data()[2 * i + 1]) << 8);
  };
  EXPECT_EQ(value(0), 1000);
  EXPECT_EQ(value(5), 1005);
  EXPECT_EQ(response.shot().frame_name_image_sensor(), "frontleft");
  // The depth frame hangs off the visual frame, 2 cm to its right.
  const auto& edge = response.shot().transforms_snapshot().child_to_parent_edge_map().at("frontleft");
  EXPECT_EQ(edge.parent_frame_name(), "frontleft_fisheye");
  EXPECT_NEAR(edge.parent_tform_child().position().x(), 0.02, 1e-12);
}

TEST(Images, DepthInVisualFrameIsRegisteredToTheVisualImage) {
  const auto response = Get(*Cameras(), Request("frontleft_depth_in_visual_frame", api::Image::FORMAT_RAW));
  ASSERT_EQ(response.status(), api::ImageResponse::STATUS_OK);
  const auto& image = response.shot().image();
  EXPECT_EQ(image.cols(), kWidth);
  EXPECT_EQ(image.rows(), kHeight);
  EXPECT_EQ(image.pixel_format(), api::Image::PIXEL_FORMAT_DEPTH_U16);
  EXPECT_EQ(response.shot().frame_name_image_sensor(), "frontleft_fisheye");
}

TEST(Images, RegisteringToTheSameCameraIsTheIdentity) {
  const spot_sim::PinholeIntrinsics k{50.0, 50.0, 32.0, 24.0};
  auto depth = Plane(64, 48, k, 2.0);
  depth.depth_mm[10] = 0;     // a hole stays a hole
  depth.depth_mm[100] = 1500;  // a nearer point stays where it is
  const auto registered = spot_sim::RegisterDepth(depth, Pose{}, {64, 48, k});
  EXPECT_EQ(registered.depth_mm, depth.depth_mm);
}

TEST(Images, RegistrationIntoAnOffsetHigherResolutionCamera) {
  // Depth: 424x240 sensor; visual: 640x480, 3 cm to its left; a wall 2 m away.
  const auto depth_k = spot_sim::PinholeIntrinsics::FromHorizontalFov(424, 240, 1.5);
  const auto visual_k = spot_sim::PinholeIntrinsics::FromHorizontalFov(640, 480, 1.6);
  const Pose visual_T_depth{{0.03, 0.0, 0.0}, Quat{}};
  const auto registered = spot_sim::RegisterDepth(Plane(424, 240, depth_k, 2.0), visual_T_depth, {640, 480, visual_k});
  ASSERT_EQ(registered.geometry.width, 640);
  ASSERT_EQ(registered.geometry.height, 480);
  // Where the depth sensor sees the wall, the visual image gets 2 m with no gaps.
  int filled = 0, total = 0;
  for (int v = 180; v < 300; ++v) {
    for (int u = 200; u < 440; ++u) {
      const uint16_t mm = registered.depth_mm[static_cast<std::size_t>(v) * 640 + u];
      ++total;
      if (mm != 0) {
        ++filled;
        EXPECT_EQ(mm, 2000);
      }
    }
  }
  EXPECT_EQ(filled, total);
  // The depth sensor's narrower vertical field of view leaves the visual
  // image's top and bottom rows without depth.
  EXPECT_EQ(registered.depth_mm[320], 0);
}

TEST(Images, RejectsWhatTheCamerasCannotProvide) {
  const auto cameras = Cameras();
  EXPECT_EQ(Get(*cameras, Request("hand_color_image")).status(), api::ImageResponse::STATUS_UNKNOWN_CAMERA);
  EXPECT_EQ(Get(*cameras, Request("frontleft_depth", api::Image::FORMAT_JPEG)).status(),
            api::ImageResponse::STATUS_UNSUPPORTED_IMAGE_FORMAT_REQUESTED);
  EXPECT_EQ(Get(*cameras, Request("frontleft_fisheye_image", api::Image::FORMAT_RLE)).status(),
            api::ImageResponse::STATUS_UNSUPPORTED_IMAGE_FORMAT_REQUESTED);
  EXPECT_EQ(Get(*cameras, Request("frontleft_fisheye_image", api::Image::FORMAT_RAW, api::Image::PIXEL_FORMAT_RGBA_U8))
                .status(),
            api::ImageResponse::STATUS_UNSUPPORTED_PIXEL_FORMAT_REQUESTED);
  auto resized = Request("frontleft_fisheye_image");
  resized.mutable_image_requests(0)->set_resize_ratio(0.5);
  EXPECT_EQ(Get(*cameras, resized).status(), api::ImageResponse::STATUS_UNSUPPORTED_RESIZE_RATIO_REQUESTED);
}

TEST(Images, FallbackPixelFormat) {
  auto request = Request("frontleft_fisheye_image", api::Image::FORMAT_RAW, api::Image::PIXEL_FORMAT_RGBA_U8);
  request.mutable_image_requests(0)->add_fallback_formats(api::Image::PIXEL_FORMAT_GREYSCALE_U8);
  const auto response = Get(*Cameras(), request);
  ASSERT_EQ(response.status(), api::ImageResponse::STATUS_OK);
  EXPECT_EQ(response.shot().image().pixel_format(), api::Image::PIXEL_FORMAT_GREYSCALE_U8);
}

TEST(Images, WaitsForTheFirstRenderedFrame) {
  BodyCameras cameras;
  cameras.AddCamera("back", kMount, kDepthMount);
  EXPECT_EQ(cameras.GetImage(Request("back_fisheye_image"), BodyTree(), 10ms).image_responses(0).status(),
            api::ImageResponse::STATUS_SOURCE_DATA_ERROR);

  std::thread renderer([&] {
    std::this_thread::sleep_for(50ms);
    cameras.SetVisualFrame("back", TestVisual());
  });
  const auto response = cameras.GetImage(Request("back_fisheye_image"), BodyTree(), 5s);
  renderer.join();
  EXPECT_EQ(response.image_responses(0).status(), api::ImageResponse::STATUS_OK);
}

TEST(HandCamera, SourcesFramesAndTheWristPoseOfEachImage) {
  BodyCameras cameras;
  const Pose wrist_T_visual{{0.12, 0.0, 0.0}, Quat::FromRpy(-M_PI / 2, 0.0, -M_PI / 2)};
  const Pose wrist_T_depth = wrist_T_visual * Pose{{0.02, 0.0, 0.0}, Quat{}};
  cameras.AddCamera("hand", spot_sim::CameraSpec::Hand(wrist_T_visual, wrist_T_depth));

  // Visual and depth images, taken with the wrist in different places.
  const Pose wrist_at_visual{{0.8, 0.0, 0.3}, Quat{}};
  const Pose wrist_at_depth{{0.7, 0.1, 0.2}, Quat::FromYaw(0.3)};
  VisualFrame visual = TestVisual();
  visual.body_T_mount = wrist_at_visual;
  DepthFrame depth = TestDepth();
  depth.body_T_mount = wrist_at_depth;
  cameras.SetVisualFrame("hand", visual);
  cameras.SetDepthFrame("hand", depth);

  std::set<std::string> sources;
  const auto listed = cameras.ListImageSources();
  for (const auto& source : listed.image_sources()) {
    sources.insert(source.name());
  }
  EXPECT_EQ(sources, (std::set<std::string>{"hand_color_image", "hand_depth", "hand_depth_in_hand_color_frame"}));

  const auto check = [](const api::ImageResponse& response, const std::string& frame, const Pose& wrist) {
    ASSERT_EQ(response.status(), api::ImageResponse::STATUS_OK);
    EXPECT_EQ(response.shot().frame_name_image_sensor(), frame);
    const auto& edges = response.shot().transforms_snapshot().child_to_parent_edge_map();
    ASSERT_EQ(edges.count("arm0.link_wr1"), 1u);
    EXPECT_EQ(edges.at("arm0.link_wr1").parent_frame_name(), "body");
    EXPECT_DOUBLE_EQ(edges.at("arm0.link_wr1").parent_tform_child().position().x(), wrist.position.x);
    EXPECT_EQ(edges.at("hand_color_image_sensor").parent_frame_name(), "arm0.link_wr1");
    EXPECT_EQ(edges.at("hand_depth_sensor").parent_frame_name(), "hand_color_image_sensor");
    EXPECT_EQ(edges.count("odom"), 1u);  // the robot's frames are kept
  };
  check(Get(cameras, Request("hand_color_image")), "hand_color_image_sensor", wrist_at_visual);
  check(Get(cameras, Request("hand_depth")), "hand_depth_sensor", wrist_at_depth);
  const auto registered = Get(cameras, Request("hand_depth_in_hand_color_frame"));
  check(registered, "hand_color_image_sensor", wrist_at_depth);
  EXPECT_EQ(registered.shot().image().cols(), kWidth);
  EXPECT_EQ(Get(cameras, Request("frontleft_fisheye_image")).status(), api::ImageResponse::STATUS_UNKNOWN_CAMERA);
}
