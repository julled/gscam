// Copyright 2022 Jonathan Bohren, Clyde McQueen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <string>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "gscam/gscam.hpp"

namespace
{

struct SplitMuxContext
{
  GstClockTime base_time;
  int64_t time_offset_ns;
  std::string recording_path;
  std::string recording_suffix;
};

std::string build_segment_path(const SplitMuxContext & ctx, GstBuffer * buffer)
{
  const int64_t absolute_ns =
    static_cast<int64_t>(GST_BUFFER_PTS(buffer)) +
    static_cast<int64_t>(ctx.base_time) +
    ctx.time_offset_ns;

  const auto time_point = std::chrono::system_clock::time_point(std::chrono::nanoseconds(
      absolute_ns));
  std::time_t seconds = std::chrono::system_clock::to_time_t(time_point);
  std::tm tm_time;
#if defined(_WIN32)
  localtime_s(&tm_time, &seconds);
#else
  localtime_r(&seconds, &tm_time);
#endif

  std::ostringstream os;
  os << ctx.recording_path;
  if (!ctx.recording_path.empty() && ctx.recording_path.back() != '/') {
    os << '/';
  }
  os << std::put_time(&tm_time, "%Y-%m-%d-%H-%M-%S") << '_' << absolute_ns;
  if (!ctx.recording_suffix.empty()) {
    os << '_' << ctx.recording_suffix;
  }
  os << ".mp4";
  return os.str();
}

void format_location_full_cb(
  GstElement * splitmux, guint, GstSample * first_sample, gpointer udata)
{
  if (first_sample == nullptr) {
    g_printerr("splitmuxsink format-location-full: no first sample provided\n");
    return;
  }

  auto * ctx = static_cast<SplitMuxContext *>(udata);
  GstBuffer * buffer = gst_sample_get_buffer(first_sample);

  if (buffer == nullptr || !GST_BUFFER_PTS_IS_VALID(buffer)) {
    g_printerr("splitmuxsink format-location-full: invalid buffer timestamp\n");
    return;
  }

  // Read base_time here, not at setup time: the pipeline sets base_time only
  // when transitioning to PLAYING, so any value captured before that is 0.
  ctx->base_time = gst_element_get_base_time(splitmux);

  const auto location = build_segment_path(*ctx, buffer);
  g_object_set(G_OBJECT(splitmux), "location", location.c_str(), NULL);
  g_print("splitmuxsink: writing segment to %s\n", location.c_str());
}

}  // namespace

namespace gscam
{

GSCam::GSCam(const rclcpp::NodeOptions & options)
: rclcpp::Node("gscam_publisher", options),
  gsconfig_(""),
  pipeline_(NULL),
  sink_(NULL),
  camera_info_manager_(this),
  time_offset_(0),
  pipeline_base_time_(0),
  stop_signal_(false)
{
  pipeline_thread_ = std::thread(
    [this]()
    {
      run();
    });
}

GSCam::~GSCam()
{
  stop_signal_ = true;
  pipeline_thread_.join();
}

bool GSCam::configure()
{
  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  bool gsconfig_rosparam_defined = false;
  char * gsconfig_env = NULL;

  const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
  gsconfig_rosparam_defined = !gsconfig_rosparam.empty();
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (!gsconfig_env && !gsconfig_rosparam_defined) {
    RCLCPP_FATAL(
      get_logger(),
      "Problem getting GSCAM_CONFIG environment variable and "
      "'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline.");
    return false;
  } else if (gsconfig_env && gsconfig_rosparam_defined) {
    RCLCPP_FATAL(
      get_logger(),
      "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. "
      "Please only define one.");
    return false;
  } else if (gsconfig_env) {
    gsconfig_ = gsconfig_env;
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Using gstreamer config from env: \"" << gsconfig_env << "\"");
  } else if (gsconfig_rosparam_defined) {
    gsconfig_ = gsconfig_rosparam;
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Using gstreamer config from rosparam: \"" << gsconfig_rosparam << "\"");
  }

  // Get additional gscam configuration
  sync_sink_ = declare_parameter("sync_sink", true);
  preroll_ = declare_parameter("preroll", false);
  use_gst_timestamps_ = declare_parameter("use_gst_timestamps", false);

  reopen_on_eof_ = declare_parameter("reopen_on_eof", false);

  // Get the camera parameters file
  camera_info_url_ = declare_parameter("camera_info_url", "");
  camera_name_ = declare_parameter("camera_name", "");
  recording_path_ = declare_parameter("recording_path", "");
  recording_suffix_ = declare_parameter("recording_suffix", "");
  recording_enabled = declare_parameter("recording_enabled", false);

  // Get the image encoding
  image_encoding_ =
    declare_parameter("image_encoding", std::string(sensor_msgs::image_encodings::RGB8));
  if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
    image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
    image_encoding_ != sensor_msgs::image_encodings::YUV422 &&
    image_encoding_ != "jpeg")
  {
    RCLCPP_FATAL_STREAM(get_logger(), "Unsupported image encoding: " + image_encoding_);
  }

  camera_info_manager_.setCameraName(camera_name_);

  if (camera_info_manager_.validateURL(camera_info_url_)) {
    camera_info_manager_.loadCameraInfo(camera_info_url_);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded camera calibration from " << camera_info_url_);
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Camera info at: " << camera_info_url_ << " not found. Using an uncalibrated config.");
  }

  // Get TF Frame
  frame_id_ = declare_parameter("frame_id", "camera_frame");
  if (frame_id_ == "camera_frame") {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "No camera frame_id set, using frame \"" << frame_id_ << "\".");
  }

  use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", false);

  return true;
}

bool GSCam::init_stream()
{
  if (!gst_is_initialized()) {
    // Initialize gstreamer pipeline
    RCLCPP_DEBUG_STREAM(get_logger(), "Initializing gstreamer...");
    gst_init(0, 0);
  }

  RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer Version: " << gst_version_string() );

  GError * error = 0;  // Assignment to zero is a gst requirement

  pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
  if (pipeline_ == NULL) {
    RCLCPP_FATAL_STREAM(get_logger(), error->message);
    return false;
  }

  // Create RGB sink
  sink_ = gst_element_factory_make("appsink", NULL);
  gst_app_sink_set_max_buffers(GST_APP_SINK(sink_), 1);
  gst_app_sink_set_drop(GST_APP_SINK(sink_), TRUE);
  GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));
  if (caps) {
    gst_caps_unref(caps);
    caps = nullptr;
  }

  // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGB",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "GRAY8",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "UYVY",
      NULL);
  } else if (image_encoding_ == "jpeg") {
    caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }

  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be
  // dropped
  gst_base_sink_set_sync(
    GST_BASE_SINK(sink_),
    (sync_sink_) ? TRUE : FALSE);

  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    g_assert(outpad);

    GstElement * outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
      RCLCPP_FATAL(get_logger(), "gst_bin_add() failed");
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    if (!gst_element_link(outelement, sink_)) {
      RCLCPP_FATAL(
        get_logger(), "GStreamer: cannot link outelement(\"%s\") -> sink\n",
        gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    gst_object_unref(outelement);
  } else {
    GstElement * launchpipe = pipeline_;
    pipeline_ = gst_pipeline_new(NULL);
    g_assert(pipeline_);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

    if (!gst_element_link(launchpipe, sink_)) {
      RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe -> sink");
      gst_object_unref(pipeline_);
      return false;
    }
  }

  // Calibration between ros::Time and gst timestamps
  GstClock * clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ =
    static_cast<int64_t>(now().nanoseconds()) - static_cast<int64_t>(GST_TIME_AS_NSECONDS(ct));
  RCLCPP_INFO(get_logger(), "Time offset: %.6f", static_cast<double>(time_offset_) / 1e9);

  gst_element_set_state(pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
  }

  // Create ROS camera interface
  const auto qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS{1};
  if (image_encoding_ == "jpeg") {
    jpeg_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/image_raw/compressed", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera/camera_info", qos);
  } else {
    camera_pub_ = image_transport::create_camera_publisher(
      this, "camera/image_raw", qos.get_rmw_qos_profile());
  }

  return true;
}

void GSCam::publish_stream()
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing stream...");

  // Pre-roll camera if needed
  if (preroll_) {
    RCLCPP_DEBUG(get_logger(), "Performing preroll...");

    // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    // I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PLAY during preroll.");
      return;
    } else {
      RCLCPP_DEBUG(get_logger(), "Stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PAUSE.");
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Stream is PAUSED in preroll.");
    }
  }

  if (recording_enabled) {
    if (!recording_path_.empty()) {
      setup_splitmux_recording();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "record_to_file is true but no recording_path specified; recording disabled.");
    }
  } else {
    disable_splitmux_recording();
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Could not start stream!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Started stream.");

  // Poll the data as fast a spossible
  while (!stop_signal_ && rclcpp::ok()) {
    // Try to pull a sample with a timeout so transient network issues don't
    // immediately restart the pipeline. Only break on actual EOS.
    GstSample * sample =
      gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 5 * GST_SECOND);
    if (!sample) {
      if (gst_app_sink_is_eos(GST_APP_SINK(sink_))) {
        RCLCPP_INFO(get_logger(), "Could not get gstreamer sample.");
        break;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for gstreamer sample (possible transient network issue)...");
      continue;
    }
    GstBuffer * buf = gst_sample_get_buffer(sample);
    GstMemory * memory = gst_buffer_get_memory(buf, 0);
    GstMapInfo info;

    gst_memory_map(memory, &info, GST_MAP_READ);
    gsize & buf_size = info.size;
    guint8 * & buf_data = info.data;

    GstClockTime bt = gst_element_get_base_time(pipeline_);
    // RCLCPP_INFO(
    //   get_logger(),
    //   "New buffer: timestamp %.6f %lu %lu %.3f",
    //   GST_TIME_AS_USECONDS(buf->timestamp + bt) / 1e6 + time_offset_,
    //   buf->timestamp, bt, time_offset_);


#if 0
    GstFormat fmt = GST_FORMAT_TIME;
    gint64 current = -1;

    Query the current position of the stream
    if (gst_element_query_position(pipeline_, &fmt, &current)) {
      RCLCPP_INFO_STREAM(get_logger(), "Position " << current);
    }
#endif

    // Stop on end of stream
    if (!buf) {
      RCLCPP_INFO(get_logger(), "Stream ended.");
      break;
    }

    // RCLCPP_DEBUG(get_logger(), "Got data.");

    // Get the image width and height
    GstPad * pad = gst_element_get_static_pad(sink_, "sink");
    GstCaps * frame_caps = gst_pad_get_current_caps(pad);
    GstStructure * structure = gst_caps_get_structure(frame_caps, 0);
    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);
    gst_caps_unref(frame_caps);
    gst_object_unref(pad);

    // Update header information
    sensor_msgs::msg::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo;
    cinfo.reset(new sensor_msgs::msg::CameraInfo(cur_cinfo));
    if (use_gst_timestamps_) {
      cinfo->header.stamp = rclcpp::Time(GST_TIME_AS_NSECONDS(buf->pts + bt) + time_offset_);
    } else {
      cinfo->header.stamp = now();
    }
    // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
    cinfo->header.frame_id = frame_id_;
    if (image_encoding_ == "jpeg") {
      sensor_msgs::msg::CompressedImage::SharedPtr img(new sensor_msgs::msg::CompressedImage());
      img->header = cinfo->header;
      img->format = "jpeg";
      img->data.resize(buf_size);
      std::copy(
        buf_data, (buf_data) + (buf_size),
        img->data.begin());
      jpeg_pub_->publish(*img);
      cinfo_pub_->publish(*cinfo);
    } else {
      // Complain if the returned buffer is smaller than we expect
      const unsigned int expected_frame_size =
        width_ * height_ * sensor_msgs::image_encodings::numChannels(image_encoding_);

      if (buf_size < expected_frame_size) {
        RCLCPP_WARN_STREAM(
          get_logger(), "GStreamer image buffer underflow: Expected frame to be " <<
            expected_frame_size << " bytes but got only " <<
            buf_size << " bytes. (make sure frames are correctly encoded)");
      }

      // Construct Image message
      sensor_msgs::msg::Image::SharedPtr img(new sensor_msgs::msg::Image());

      img->header = cinfo->header;

      // Image data and metadata
      img->width = width_;
      img->height = height_;
      img->encoding = image_encoding_;
      img->is_bigendian = false;
      img->data.resize(expected_frame_size);

      // Copy only the data we received
      // Since we're publishing shared pointers, we need to copy the image so
      // we can free the buffer allocated by gstreamer
      img->step = width_ * sensor_msgs::image_encodings::numChannels(image_encoding_);

      std::copy(
        buf_data,
        (buf_data) + (buf_size),
        img->data.begin());

      // Publish the image/info
      camera_pub_.publish(img, cinfo);
    }

    // Release the buffer
    if (buf) {
      gst_memory_unmap(memory, &info);
      gst_memory_unref(memory);
      gst_sample_unref(sample);
    }
  }
}

GstElement * GSCam::find_splitmuxsink() const
{
  if (!GST_IS_BIN(pipeline_)) {
    return nullptr;
  }

  const char * candidate_names[] = {"splitmuxsink0", "splitmuxsink"};
  for (const auto * name : candidate_names) {
    GstElement * element = gst_bin_get_by_name(GST_BIN(pipeline_), name);
    if (element != nullptr) {
      return element;
    }
  }

  GstIterator * iterator = gst_bin_iterate_elements(GST_BIN(pipeline_));
  GValue item = G_VALUE_INIT;
  GstElement * found = nullptr;

  while (gst_iterator_next(iterator, &item) == GST_ITERATOR_OK) {
    GstElement * element = GST_ELEMENT(g_value_get_object(&item));
    GstElementFactory * factory = gst_element_get_factory(element);
    if (factory != nullptr) {
      const gchar * factory_name = gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(factory));
      if (factory_name != nullptr && std::string(factory_name) == "splitmuxsink") {
        found = GST_ELEMENT(gst_object_ref(element));
        g_value_unset(&item);
        break;
      }
    }
    g_value_unset(&item);
  }
  gst_iterator_free(iterator);
  return found;
}

void GSCam::setup_splitmux_recording()
{
  namespace fs = std::filesystem;

  const fs::path target_dir(recording_path_);
  std::error_code ec;
  if (!fs::exists(target_dir)) {
    if (!fs::create_directories(target_dir, ec)) {
      RCLCPP_FATAL(
        get_logger(),
        "Unable to create recording directory '%s': %s",
        recording_path_.c_str(), ec.message().c_str());
      stop_signal_ = true;
      rclcpp::shutdown();
      return;
    }
  } else if (!fs::is_directory(target_dir, ec)) {
    RCLCPP_FATAL(
      get_logger(),
      "recording_path '%s' exists but is not a directory",
      recording_path_.c_str());
    stop_signal_ = true;
    rclcpp::shutdown();
    return;
  }

  GstElement * splitmuxsink = find_splitmuxsink();
  if (splitmuxsink == nullptr) {
    RCLCPP_WARN(
      get_logger(),
      "recording_path provided but no splitmuxsink was found in the configured pipeline.");
    return;
  }

  pipeline_base_time_ = gst_element_get_base_time(pipeline_);

  auto * ctx = new SplitMuxContext{pipeline_base_time_, time_offset_, recording_path_,
    recording_suffix_};

  g_signal_connect_data(
    splitmuxsink,
    "format-location-full",
    G_CALLBACK(format_location_full_cb),
    ctx,
    [](gpointer data, GClosure *) {
      delete static_cast<SplitMuxContext *>(data);
    },
    static_cast<GConnectFlags>(0));

  gst_object_unref(splitmuxsink);

  RCLCPP_INFO(
    get_logger(),
    "Recording enabled: splitmuxsink segments will be written to '%s'",
    recording_path_.c_str());
}

void GSCam::disable_splitmux_recording()
{
  GstElement * splitmuxsink = find_splitmuxsink();
  if (splitmuxsink == nullptr) {
    RCLCPP_DEBUG(
      get_logger(),
      "record_to_file disabled but no splitmuxsink present in the pipeline.");
    return;
  }

  GstPad * sink_pad = nullptr;
  GstIterator * sink_iter = gst_element_iterate_sink_pads(splitmuxsink);
  if (sink_iter != nullptr) {
    GValue item = G_VALUE_INIT;
    if (gst_iterator_next(sink_iter, &item) == GST_ITERATOR_OK) {
      sink_pad = GST_PAD(g_value_get_object(&item));
      gst_object_ref(sink_pad);
      g_value_unset(&item);
    }
    gst_iterator_free(sink_iter);
  }

  if (sink_pad == nullptr) {
    RCLCPP_WARN(
      get_logger(),
      "record_to_file disabled but splitmuxsink exposes no sink pads.");
    gst_object_unref(splitmuxsink);
    return;
  }

  GstPad * peer_pad = gst_pad_get_peer(sink_pad);
  if (peer_pad == nullptr) {
    RCLCPP_WARN(
      get_logger(),
      "record_to_file disabled but splitmuxsink had no upstream peer pad.");
    gst_object_unref(sink_pad);
    gst_object_unref(splitmuxsink);
    return;
  }

  GstElement * fakesink = gst_element_factory_make("fakesink", "gscam_splitmux_disabled");
  if (fakesink == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to create fakesink to replace splitmuxsink.");
    gst_object_unref(peer_pad);
    gst_object_unref(sink_pad);
    gst_object_unref(splitmuxsink);
    return;
  }
  g_object_set(G_OBJECT(fakesink), "sync", FALSE, NULL);

  if (!gst_bin_add(GST_BIN(pipeline_), fakesink)) {
    RCLCPP_ERROR(get_logger(), "Failed to add fakesink to pipeline; recording cannot be disabled.");
    gst_object_unref(fakesink);
    gst_object_unref(peer_pad);
    gst_object_unref(sink_pad);
    gst_object_unref(splitmuxsink);
    return;
  }

  GstPad * fake_sink_pad = gst_element_get_static_pad(fakesink, "sink");
  if (fake_sink_pad == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to access fakesink sink pad; recording cannot be disabled.");
    gst_bin_remove(GST_BIN(pipeline_), fakesink);
    gst_object_unref(fakesink);
    gst_object_unref(peer_pad);
    gst_object_unref(sink_pad);
    gst_object_unref(splitmuxsink);
    return;
  }

  if (!gst_pad_unlink(peer_pad, sink_pad)) {
    RCLCPP_WARN(get_logger(), "Unable to unlink splitmuxsink from pipeline; recording remains enabled.");
    gst_object_unref(fake_sink_pad);
    gst_bin_remove(GST_BIN(pipeline_), fakesink);
    gst_object_unref(fakesink);
    gst_object_unref(peer_pad);
    gst_object_unref(sink_pad);
    gst_object_unref(splitmuxsink);
    return;
  }

  if (gst_pad_link(peer_pad, fake_sink_pad) != GST_PAD_LINK_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to link fakesink; restoring original splitmuxsink connection.");
    gst_pad_link(peer_pad, sink_pad);
    gst_object_unref(fake_sink_pad);
    gst_bin_remove(GST_BIN(pipeline_), fakesink);
    gst_object_unref(fakesink);
    gst_object_unref(peer_pad);
    gst_object_unref(sink_pad);
    gst_object_unref(splitmuxsink);
    return;
  }

  gst_element_release_request_pad(splitmuxsink, sink_pad);
  gst_object_unref(fake_sink_pad);
  gst_object_unref(peer_pad);
  gst_object_unref(sink_pad);

  gst_element_set_state(splitmuxsink, GST_STATE_NULL);
  gst_bin_remove(GST_BIN(pipeline_), splitmuxsink);
  gst_object_unref(splitmuxsink);

  RCLCPP_INFO(get_logger(), "record_to_file is false: splitmuxsink replaced with fakesink.");
}

void GSCam::cleanup_stream()
{
  // Clean up
  RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = NULL;
  }
}

void GSCam::run()
{
  if (!this->configure()) {
    RCLCPP_FATAL(get_logger(), "Failed to configure gscam!");
    return;
  }

  while (!stop_signal_ && rclcpp::ok()) {
    if (!this->init_stream()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize gscam stream!");
      break;
    }

    // Block while publishing
    this->publish_stream();

    this->cleanup_stream();

    RCLCPP_INFO(get_logger(), "GStreamer stream stopped!");

    if (reopen_on_eof_) {
      RCLCPP_INFO(get_logger(), "Reopening stream...");
    } else {
      RCLCPP_INFO(get_logger(), "Cleaning up stream and exiting...");
      break;
    }
  }
  rclcpp::shutdown();
}

// Example callbacks for appsink
// TODO(someone): enable callback-based capture
void gst_eos_cb(GstAppSink * appsink, gpointer user_data)
{
}
GstFlowReturn gst_new_preroll_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}
GstFlowReturn gst_new_asample_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}

}  // namespace gscam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam::GSCam)
