
#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <ctime>  // For time formatting

#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <camera_calibration_parsers/parse_ini.h>

#include <gscam/gscam.h>

namespace gscam {

  struct userdata {
    GstClockTime bt;
    double time_offset_seconds;
    std::string recording_path;
    std::string suffix;
  };

  // Callback function to modify the full location (directory + filename) before each split
  static void format_location_full_callback(GstElement *splitmux, guint fragment_id, GstSample *first_sample, gpointer udata) {
      // Extract the struct containing the two double values
      userdata* values = static_cast<userdata*>(udata);
      GstClockTime bt = values->bt;
      double time_offset_seconds = values->time_offset_seconds;
      std::string recording_path = values->recording_path;
      std::string suffix = values->suffix;

      // Check if the sample is valid
      if (first_sample) {
          GstBuffer *buffer = gst_sample_get_buffer(first_sample);  // Get the buffer from the sample

          // Ensure the buffer is valid and has PTS (presentation timestamp)
          if (buffer && GST_BUFFER_PTS_IS_VALID(buffer)) {
              // Extract timestamp of first frame
              double abs_stamp_seconds = GST_TIME_AS_SECONDS(buffer->pts+bt)+time_offset_seconds;
              long unsigned int abs_stamp_nano_seconds = abs_stamp_seconds * 1e9;

              // Convert timestamp to human-readable format
              struct tm *timeinfo;
              char timestamp_str[21];  // Enough for "_YYYY-MM-DD-HH-MM-SS"
              time_t timestamp_sec = static_cast<time_t>(abs_stamp_seconds);
              timeinfo = localtime(&timestamp_sec);  // Convert to local time
              strftime(timestamp_str, sizeof(timestamp_str), "_%Y-%m-%d-%H-%M-%S", timeinfo);  // Format time

              std::ostringstream new_filename;
              new_filename << recording_path << timestamp_str << "_" << std::to_string(abs_stamp_nano_seconds) << "_" << suffix << ".mp4";  // Full path

              // Set the new full path to the location in the splitmuxsink
              g_print("Setting new full location for splitmuxsink: %s\n", new_filename.str().c_str());
              g_object_set(G_OBJECT(splitmux), "location", new_filename.str().c_str(), NULL);
          } else {
              g_printerr("Error: No valid timestamp found in the first sample buffer.\n");
          }
      } else {
          g_printerr("Error: Received an invalid GstSample.\n");
      }
  }


  GSCam::GSCam(ros::NodeHandle nh_camera, ros::NodeHandle nh_private) :
    gsconfig_(""),
    pipeline_(NULL),
    sink_(NULL),
    nh_(nh_camera),
    nh_private_(nh_private),
    image_transport_(nh_camera),
    camera_info_manager_(nh_camera),
    recording_path_(""),
    suffix_("")
  {
  }

  GSCam::~GSCam()
  {
  }

  static guint frame_counter = 0;  // Global frame counter

  // Callback function to count frames
  static GstPadProbeReturn frame_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    frame_counter++;
    printf("Frame number: %u\n", frame_counter);
    return GST_PAD_PROBE_OK;
  }

  bool GSCam::configure()
  {
    // Get gstreamer configuration
    // (either from environment variable or ROS param)
    std::string gsconfig_rosparam = "";
    bool gsconfig_rosparam_defined = false;
    char *gsconfig_env = NULL;

    gsconfig_rosparam_defined = nh_private_.getParam("gscam_config",gsconfig_rosparam);
    gsconfig_env = getenv("GSCAM_CONFIG");

    if (!gsconfig_env && !gsconfig_rosparam_defined) {
      ROS_FATAL( "Problem getting GSCAM_CONFIG environment variable and 'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline." );
      return false;
    } else if(gsconfig_env && gsconfig_rosparam_defined) {
      ROS_FATAL( "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. Please only define one." );
      return false;
    } else if(gsconfig_env) {
      gsconfig_ = gsconfig_env;
      ROS_INFO_STREAM("Using gstreamer config from env: \""<<gsconfig_env<<"\"");
    } else if(gsconfig_rosparam_defined) {
      gsconfig_ = gsconfig_rosparam;
      ROS_INFO_STREAM("Using gstreamer config from rosparam: \""<<gsconfig_rosparam<<"\"");
    }

    // Get additional gscam configuration
    nh_private_.param("sync_sink", sync_sink_, true);
    nh_private_.param("preroll", preroll_, false);
    nh_private_.param("use_gst_timestamps", use_gst_timestamps_, false);

    nh_private_.param("reopen_on_eof", reopen_on_eof_, false);

    // Get the camera parameters file
    nh_private_.getParam("camera_info_url", camera_info_url_);
    nh_private_.getParam("camera_name", camera_name_);

    // Rosbag recording related
    nh_private_.getParam("recording_path", recording_path_);
    nh_private_.getParam("suffix", suffix_);

    // Get the image encoding
    nh_private_.param("image_encoding", image_encoding_, sensor_msgs::image_encodings::RGB8);
    if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
        image_encoding_ != sensor_msgs::image_encodings::MONO8 && 
        image_encoding_ != "jpeg") {
      ROS_FATAL_STREAM("Unsupported image encoding: " + image_encoding_);
    }

    camera_info_manager_.setCameraName(camera_name_);

    if(camera_info_manager_.validateURL(camera_info_url_)) {
      camera_info_manager_.loadCameraInfo(camera_info_url_);
      ROS_INFO_STREAM("Loaded camera calibration from "<<camera_info_url_);
    } else {
      ROS_WARN_STREAM("Camera info at: "<<camera_info_url_<<" not found. Using an uncalibrated config.");
    }

    // Get TF Frame
    if(!nh_private_.getParam("frame_id",frame_id_)){
      frame_id_ = "/camera_frame";
      ROS_WARN_STREAM("No camera frame_id set, using frame \""<<frame_id_<<"\".");
      nh_private_.setParam("frame_id",frame_id_);
    }

    return true;
  }

  bool GSCam::init_stream()
  {
    if(!gst_is_initialized()) {
      // Initialize gstreamer pipeline
      ROS_DEBUG_STREAM( "Initializing gstreamer..." );
      gst_init(0,0);
    }

    ROS_DEBUG_STREAM( "Gstreamer Version: " << gst_version_string() );

    GError *error = 0; // Assignment to zero is a gst requirement

    pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
    if (pipeline_ == NULL) {
      ROS_FATAL_STREAM( error->message );
      return false;
    }

    // Create RGB sink
    sink_ = gst_element_factory_make("appsink",NULL);
    GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

#if (GST_VERSION_MAJOR == 1)
    // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
    if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
        caps = gst_caps_new_simple( "video/x-raw", 
            "format", G_TYPE_STRING, "RGB",
            NULL); 
    } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
        caps = gst_caps_new_simple( "video/x-raw", 
            "format", G_TYPE_STRING, "GRAY8",
            NULL); 
    } else if (image_encoding_ == "jpeg") {
        caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
    }
#else
    if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
        caps = gst_caps_new_simple( "video/x-raw-rgb", NULL,NULL); 
    } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
        caps = gst_caps_new_simple("video/x-raw-gray", NULL, NULL);
    } else if (image_encoding_ == "jpeg") {
        caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
    }
#endif

    gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
    gst_caps_unref(caps);

    // Set whether the sink should sync
    // Sometimes setting this to true can cause a large number of frames to be
    // dropped
    gst_base_sink_set_sync(
        GST_BASE_SINK(sink_),
        (sync_sink_) ? TRUE : FALSE);

    if(GST_IS_PIPELINE(pipeline_)) {
      GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
      g_assert(outpad);

      GstElement *outelement = gst_pad_get_parent_element(outpad);
      g_assert(outelement);
      gst_object_unref(outpad);

      if(!gst_bin_add(GST_BIN(pipeline_), sink_)) {
        ROS_FATAL("gst_bin_add() failed");
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      if(!gst_element_link(outelement, sink_)) {
        ROS_FATAL("GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      gst_object_unref(outelement);



    } else {
      GstElement* launchpipe = pipeline_;
      pipeline_ = gst_pipeline_new(NULL);
      g_assert(pipeline_);

      gst_object_unparent(GST_OBJECT(launchpipe));

      gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

      if(!gst_element_link(launchpipe, sink_)) {
        ROS_FATAL("GStreamer: cannot link launchpipe -> sink");
        gst_object_unref(pipeline_);
        return false;
      }
    }

    // Calibration between ros::Time and gst timestamps
    GstClock * clock = gst_system_clock_obtain();
    ros::Time now = ros::Time::now();
    GstClockTime ct = gst_clock_get_time(clock);
    gst_object_unref(clock);
    time_offset_ = now.toSec() - GST_TIME_AS_USECONDS(ct)/1e6;
    ROS_INFO("Time offset: %.3f",time_offset_);

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
      return false;
    } else {
      ROS_DEBUG_STREAM("Stream is PAUSED.");
    }


    // Create ROS camera interface
    if (image_encoding_ == "jpeg") {
        jpeg_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("camera/image_raw/compressed",1);
        cinfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera/camera_info",1);
    } else {
        if ( camera_name_.compare("default") ) {
            std::string camname_;
            camname_ = camera_name_ + "/image_raw"; 
            camera_pub_ = image_transport_.advertiseCamera(camname_.c_str(), 1);
        }
        else {
            camera_pub_ = image_transport_.advertiseCamera("camera/image_raw", 1);
        }
    }

    return true;
  }

  void GSCam::publish_stream()
  {
    ROS_INFO_STREAM("Publishing stream...");

    // Pre-roll camera if needed
    if (preroll_) {
      ROS_DEBUG("Performing preroll...");

      //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
      //I am told this is needed and am erring on the side of caution.
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        ROS_ERROR("Failed to PLAY during preroll.");
        return;
      } else {
        ROS_DEBUG("Stream is PLAYING in preroll.");
      }

      gst_element_set_state(pipeline_, GST_STATE_PAUSED);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        ROS_ERROR("Failed to PAUSE.");
        return;
      } else {
        ROS_INFO("Stream is PAUSED in preroll.");
      }
    }

    if(gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Could not start stream!");
      return;
    }
    ROS_INFO("Started stream.");

    if (recording_path_ != ""){
      // Get the base time of the pipeline
      GstClockTime bt = gst_element_get_base_time(pipeline_);
      // Get the splitmuxsink element from the pipeline
      GstElement *splitmuxsink = gst_bin_get_by_name(GST_BIN(pipeline_), "splitmuxsink0");
      userdata* udata = new userdata{bt, time_offset_, recording_path_, suffix_};
      // Connect the "format-location-full" signal to the callback function
      g_signal_connect(splitmuxsink, "format-location-full", G_CALLBACK(format_location_full_callback), udata);
    }
    else{
      ROS_INFO("No recording path specified, not using splitmuxsink and recording to raw video.");

    }

    // Poll the data as fast a spossible
    while(ros::ok()) 
    {
      // This should block until a new frame is awake, this way, we'll run at the
      // actual capture framerate of the device.
      // ROS_DEBUG("Getting data...");
#if (GST_VERSION_MAJOR == 1)
      GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
      if(!sample) {
        ROS_ERROR("Could not get gstreamer sample.");
        break;
      }
      GstBuffer* buf = gst_sample_get_buffer(sample);
      GstMemory *memory = gst_buffer_get_memory(buf, 0);
      GstMapInfo info;

      gst_memory_map(memory, &info, GST_MAP_READ);
      gsize &buf_size = info.size;
      guint8* &buf_data = info.data;
#else
      GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink_));
      guint &buf_size = buf->size;
      guint8* &buf_data = buf->data;
#endif
      GstClockTime bt = gst_element_get_base_time(pipeline_);
      //ROS_INFO("New buffer: timestamp %.6f %lu %lu %.3f", GST_TIME_AS_USECONDS(buf->pts+bt)/1e6+time_offset_, buf->pts, bt, time_offset_);

      // Stop on end of stream
      if (!buf) {
        ROS_INFO("Stream ended.");
        break;
      }

      // ROS_DEBUG("Got data.");

      // Get the image width and height
#if (GST_VERSION_MAJOR == 1)
      GstCaps *caps = gst_sample_get_caps(sample);
#else
      GstPad* pad = gst_element_get_static_pad(sink_, "sink");
      const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
#endif
      GstStructure *structure = gst_caps_get_structure(caps,0);
      gst_structure_get_int(structure,"width",&width_);
      gst_structure_get_int(structure,"height",&height_);

      // Update header information
      sensor_msgs::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
      sensor_msgs::CameraInfoPtr cinfo;
      cinfo.reset(new sensor_msgs::CameraInfo(cur_cinfo));
      if (use_gst_timestamps_) {
#if (GST_VERSION_MAJOR == 1)
          cinfo->header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->pts+bt)/1e6+time_offset_);
#else
          cinfo->header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->timestamp+bt)/1e6+time_offset_);
#endif
      } else {
          cinfo->header.stamp = ros::Time::now();
      }
      // ROS_INFO("Image time stamp: %.3f",cinfo->header.stamp.toSec());
      cinfo->header.frame_id = frame_id_;
      if (image_encoding_ == "jpeg") {
          sensor_msgs::CompressedImagePtr img(new sensor_msgs::CompressedImage());
          img->header = cinfo->header;
          img->format = "jpeg";
          img->data.resize(buf_size);
          std::copy(buf_data, (buf_data)+(buf_size),
                  img->data.begin());
          jpeg_pub_.publish(img);
          cinfo_pub_.publish(cinfo);
      } else {
          // Complain if the returned buffer is smaller than we expect
          const unsigned int expected_frame_size =
              image_encoding_ == sensor_msgs::image_encodings::RGB8
              ? width_ * height_ * 3
              : width_ * height_;

          if (buf_size < expected_frame_size) {
              ROS_WARN_STREAM( "GStreamer image buffer underflow: Expected frame to be "
                      << expected_frame_size << " bytes but got only "
                      << (buf_size) << " bytes. (make sure frames are correctly encoded)");
          }

          // Construct Image message
          sensor_msgs::ImagePtr img(new sensor_msgs::Image());

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
          if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
              img->step = width_ * 3;
          } else {
              img->step = width_;
          }
          std::copy(
                  buf_data,
                  (buf_data)+(buf_size),
                  img->data.begin());

          // Publish the image/info
          camera_pub_.publish(img, cinfo);
      }

      // Release the buffer
      if(buf) {
#if (GST_VERSION_MAJOR == 1)
        // Unmap the memory
        gst_memory_unmap(memory, &info);
        gst_memory_unref(memory);
#endif
        // Do not unref the buffer obtained by gst_sample_get_buffer
        // (We do not own the reference to it!)
        // NO : gst_buffer_unref(buf);
        //
         // Release the sample
         if (sample) {
            gst_sample_unref(sample);
         }
      }

      ros::spinOnce();
    }
  }

  void GSCam::cleanup_stream()
  {
    // Clean up
    ROS_INFO("Stopping gstreamer pipeline...");
    if(pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = NULL;
    }
  }

  void GSCam::run() {
    while(ros::ok()) {
      if(!this->configure()) {
        ROS_FATAL("Failed to configure gscam!");
        break;
      }

      if(!this->init_stream()) {
        ROS_FATAL("Failed to initialize gscam stream!");
        break;
      }

      // Block while publishing
      this->publish_stream();

      this->cleanup_stream();

      ROS_INFO("GStreamer stream stopped!");

      if(reopen_on_eof_) {
        ROS_INFO("Reopening stream...");
      } else {
        ROS_INFO("Cleaning up stream and exiting...");
        break;
      }
    }

  }

  // Example callbacks for appsink
  // TODO: enable callback-based capture
  void gst_eos_cb(GstAppSink *appsink, gpointer user_data ) {
  }
  GstFlowReturn gst_new_preroll_cb(GstAppSink *appsink, gpointer user_data ) {
    return GST_FLOW_OK;
  }
  GstFlowReturn gst_new_asample_cb(GstAppSink *appsink, gpointer user_data ) {
    return GST_FLOW_OK;
  }


}


