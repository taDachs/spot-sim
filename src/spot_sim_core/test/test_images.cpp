#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include <jpeglib.h>

#include "spot_sim_core/images.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;
using spot_sim::BodyCameras;
using spot_sim::CameraFrame;

namespace {
constexpr int kWidth = 8, kHeight = 6;

CameraFrame TestFrame(spot_sim::RobotTime stamp = 7s) {
  CameraFrame frame;
  frame.stamp = stamp;
  frame.width = kWidth;
  frame.height = kHeight;
  frame.intrinsics = {5.0, 5.5, 4.0, 3.0};
  for (int i = 0; i < kWidth * kHeight; ++i) {
    frame.rgb.insert(frame.rgb.end(), {200, 100, 50});
    frame.depth_mm.push_back(static_cast<uint16_t>(1000 + i));
  }
  return frame;
}

const spot_sim::Pose kMount{{0.4, 0.05, -0.02}, spot_sim::Quat::FromRpy(-M_PI / 2, 0.0, -M_PI / 2)};

std::unique_ptr<BodyCameras> Cameras() {
  auto cameras = std::make_unique<BodyCameras>();
  cameras->AddCamera("frontleft", kMount);
  cameras->SetFrame("frontleft", TestFrame());
  return cameras;
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
  EXPECT_TRUE(response.transforms_snapshot().child_to_parent_edge_map().count("frontleft_fisheye"));
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

TEST(Images, DepthIsRawMillimeters) {
  for (const std::string source : {"frontleft_depth", "frontleft_depth_in_visual_frame"}) {
    const auto response = Get(*Cameras(), Request(source, api::Image::FORMAT_RAW));
    ASSERT_EQ(response.status(), api::ImageResponse::STATUS_OK) << source;
    const auto& image = response.shot().image();
    EXPECT_EQ(image.pixel_format(), api::Image::PIXEL_FORMAT_DEPTH_U16);
    ASSERT_EQ(image.data().size(), static_cast<std::size_t>(2 * kWidth * kHeight));
    const auto value = [&](int i) {
      return static_cast<uint8_t>(image.data()[2 * i]) | (static_cast<uint8_t>(image.data()[2 * i + 1]) << 8);
    };
    EXPECT_EQ(value(0), 1000);
    EXPECT_EQ(value(5), 1005);
  }
  EXPECT_EQ(Get(*Cameras(), Request("frontleft_depth")).shot().frame_name_image_sensor(), "frontleft");
  EXPECT_EQ(Get(*Cameras(), Request("frontleft_depth_in_visual_frame")).shot().frame_name_image_sensor(),
            "frontleft_fisheye");
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
  cameras.AddCamera("back", kMount);
  EXPECT_EQ(cameras.GetImage(Request("back_fisheye_image"), BodyTree(), 10ms).image_responses(0).status(),
            api::ImageResponse::STATUS_SOURCE_DATA_ERROR);

  std::thread renderer([&] {
    std::this_thread::sleep_for(50ms);
    cameras.SetFrame("back", TestFrame());
  });
  const auto response = cameras.GetImage(Request("back_fisheye_image"), BodyTree(), 5s);
  renderer.join();
  EXPECT_EQ(response.image_responses(0).status(), api::ImageResponse::STATUS_OK);
}
