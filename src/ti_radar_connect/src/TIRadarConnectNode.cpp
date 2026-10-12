#include "TIRadarConnectNode.hpp"

using cpsl::radar::Code;
using cpsl::radar::Status;

TIRadarConnectNode::TIRadarConnectNode():
    Node("ti_radar_connect_node"),
    config_path(""),
    frame_id(""),
    stamp_delay_sec(),
    stamp_delay(rclcpp::Duration::from_seconds(0.0)),
    radar_config_path(),
    radar_(),
    startup_ok_(false),
    pointcloud_enabled_(false),
    adc_enabled_(false),
    steady_to_ros_offset_ns_(0),
    radar_config_path_pub_(),
    detected_points_pub_(),
    adc_data_cube_pub_()
{
    //declare parameters
    this->declare_parameter<std::string>("config_path");
    this->declare_parameter<std::string>("radar_name","radar_0");
    this->declare_parameter<std::string>("tf_prefix","");
    this->declare_parameter<double>("stamp_delay_sec",0.0);

    //load in parameters
    config_path = this->get_parameter("config_path").as_string();
    std::string radar_name = this->get_parameter("radar_name").as_string();
    std::string tf_prefix = this->get_parameter("tf_prefix").as_string();
    // No leading '/' when there is no prefix: tf2 rejects "/front_radar" as a
    // frame id, so it would never match the URDF's "front_radar".
    frame_id = tf_prefix.empty() ? radar_name : tf_prefix + "/" + radar_name;
    stamp_delay_sec = this->get_parameter("stamp_delay_sec").as_double();
    stamp_delay = rclcpp::Duration::from_seconds(stamp_delay_sec);

    //route the driver's log lines through the ROS logger (the driver never prints on its own)
    cpsl::radar::set_log_sink([](cpsl::radar::LogLevel level, const std::string & msg){
        auto logger = rclcpp::get_logger("cpsl_ti_radar");
        switch(level){
            case cpsl::radar::LogLevel::error: RCLCPP_ERROR(logger,"%s",msg.c_str()); break;
            case cpsl::radar::LogLevel::warn:  RCLCPP_WARN(logger,"%s",msg.c_str()); break;
            case cpsl::radar::LogLevel::info:  RCLCPP_INFO(logger,"%s",msg.c_str()); break;
            default:                           RCLCPP_DEBUG(logger,"%s",msg.c_str()); break;
        }
    });

    //load the (schema v2) system config; a v1 file is rejected here, loudly
    auto cfg = cpsl::radar::RadarConfig::load(config_path);
    if(!cfg){
        RCLCPP_ERROR(this->get_logger(),"cannot load radar config '%s': %s: %s",
            config_path.c_str(), cpsl::radar::to_string(cfg.status.code), cfg.status.message.c_str());
        return;
    }

    //open the ports named by the config (cli.port, DCA1000 socket, ...); sends nothing yet
    auto opened = cpsl::radar::Radar::open(*cfg);
    if(!opened){
        RCLCPP_ERROR(this->get_logger(),"cannot open radar (config '%s'): %s: %s",
            config_path.c_str(), cpsl::radar::to_string(opened.status.code), opened.status.message.c_str());
        return;
    }
    radar_ = std::move(*opened);

    pointcloud_enabled_ = radar_->serial_enabled();
    adc_enabled_ = radar_->dca1000_enabled();

    if(!adc_enabled_ && !pointcloud_enabled_){
        RCLCPP_ERROR(this->get_logger(),
            "config '%s' enables neither dca1000 nor serial_stream: nothing to publish",
            config_path.c_str());
        return;
    }

    //define a qos profile for some publishers
    rclcpp::QoS qos_profile = rclcpp::QoS(5);
    qos_profile.transient_local();
    qos_profile.reliable();

    radar_config_path_pub_ =
        this->create_publisher<std_msgs::msg::String>(radar_name + "/radar_config_path",qos_profile);

    if(pointcloud_enabled_){
        std::string detected_points_topic = radar_name + "/detected_points";
        detected_points_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(detected_points_topic,qos_profile);
    }
    else{
        // e.g. the iwr1843_sar_lvds firmware has no on-chip detection and no data UART
        RCLCPP_INFO(this->get_logger(),
            "point cloud disabled: serial_stream.enabled is false in '%s' (board %s, firmware has no point-cloud path or it is switched off); running ADC-only",
            config_path.c_str(), cfg->board().name.c_str());
    }

    if(adc_enabled_){
        std::string adc_cube_topic = radar_name + "/adc_data_cube";
        adc_data_cube_pub_ =
            this->create_publisher<raw_radar_msgs::msg::ADCDataCube>(adc_cube_topic,qos_profile);
    }

    //publish the radar_config_path
    radar_config_path = cfg->system().getRadarConfigPath();
    pub_radar_config_path();

    //configure the radar (DCA1000 first, then the radar cfg)
    Status st = radar_->configure();
    if(st.code == Code::already_configured){
        RCLCPP_WARN(this->get_logger(),"radar already configured in this process, continuing: %s",st.message.c_str());
    }
    else if(!st){
        RCLCPP_ERROR(this->get_logger(),"cannot configure radar (config '%s'): %s: %s",
            config_path.c_str(), cpsl::radar::to_string(st.code), st.message.c_str());
        return;
    }

    startup_ok_ = true;
}

TIRadarConnectNode::~TIRadarConnectNode(){}

rclcpp::Time TIRadarConnectNode::to_ros_time(std::chrono::steady_clock::time_point t) const{
    // Frames carry a steady_clock time (completed_at), not wall time. The
    // steady->ROS offset is measured ONCE at start(), as (ROS now) - (steady
    // now), and applied to every frame, so stamps are monotonic and do not
    // jitter with scheduling. Drift between the two clocks over a long run is
    // not corrected. stamp_delay_sec is subtracted afterwards, as in v1.
    int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    return rclcpp::Time(steady_ns + steady_to_ros_offset_ns_, this->get_clock()->get_clock_type()) - stamp_delay;
}

int TIRadarConnectNode::run(void){

    if(!startup_ok_ || !radar_){
        RCLCPP_ERROR(this->get_logger(),"radar not ready, not starting");
        return 1;
    }

    //take the steady->ROS clock offset once
    {
        int64_t ros_ns = this->now().nanoseconds();
        int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        steady_to_ros_offset_ns_ = ros_ns - steady_ns;
    }

    Status st = radar_->start();
    if(!st){
        RCLCPP_ERROR(this->get_logger(),"cannot start radar: %s: %s",
            cpsl::radar::to_string(st.code), st.message.c_str());
        return 1;
    }
    RCLCPP_INFO(this->get_logger(),"radar started (adc: %s, point cloud: %s)",
        adc_enabled_ ? "on" : "off", pointcloud_enabled_ ? "on" : "off");

    // With one stream, block up to 500 ms in it; with two, alternate short waits
    // so neither starves the other (and shutdown is noticed quickly).
    const std::chrono::milliseconds wait =
        (adc_enabled_ && pointcloud_enabled_) ? std::chrono::milliseconds(50) : std::chrono::milliseconds(500);

    cpsl::radar::AdcFrame adc_frame;      // reused: the driver swaps buffers in, no per-frame allocation
    cpsl::radar::PointCloud point_cloud;
    int exit_code = 0;
    auto last_counters = std::chrono::steady_clock::now();

    while(rclcpp::ok()){
        Status why;

        if(pointcloud_enabled_){
            if(radar_->next_point_cloud(point_cloud, wait, &why)){
                detected_points_pub_->publish(get_pointcloud2_msg(
                    point_cloud.points, to_ros_time(point_cloud.completed_at)));
            }
            else if(why.code == Code::stopped){ break; }
            else if(why.code != Code::timeout && why.code != Code::stalled){
                RCLCPP_ERROR(this->get_logger(),"point cloud stream failed: %s: %s",
                    cpsl::radar::to_string(why.code), why.message.c_str());
                exit_code = 1; break;
            }
        }

        if(adc_enabled_){
            if(radar_->next_adc_frame(adc_frame, wait, &why)){
                adc_data_cube_pub_->publish(get_AdcDataCube_msg(
                    adc_frame, to_ros_time(adc_frame.completed_at)));
            }
            else if(why.code == Code::stopped){ break; }
            // timeout: no frame yet, publish nothing. stalled: the driver already warned.
            else if(why.code != Code::timeout && why.code != Code::stalled){
                RCLCPP_ERROR(this->get_logger(),"ADC stream failed: %s: %s",
                    cpsl::radar::to_string(why.code), why.message.c_str());
                exit_code = 1; break;
            }
        }

        auto now = std::chrono::steady_clock::now();
        if(now - last_counters >= std::chrono::seconds(1)){
            last_counters = now;
            log_counters(false);
        }
    }

    st = radar_->stop();
    if(!st){
        RCLCPP_ERROR(this->get_logger(),"stop reported: %s: %s",
            cpsl::radar::to_string(st.code), st.message.c_str());
        if(exit_code == 0){ exit_code = 1; }
    }
    log_counters(true);
    return exit_code;
}

void TIRadarConnectNode::log_counters(bool final_report){
    cpsl::radar::Stats s = radar_->stats();
    if(final_report){
        RCLCPP_INFO(this->get_logger(),
            "radar stopped: frames=%lu frames_overwritten=%lu incomplete_frames=%lu skipped_frames=%lu dropped_packets=%lu kernel_drops=%lu stalls=%lu",
            (unsigned long)s.frames,(unsigned long)s.frames_overwritten,(unsigned long)s.incomplete_frames,
            (unsigned long)s.skipped_frames,(unsigned long)s.dropped,(unsigned long)s.kernel_drops,(unsigned long)s.stalls);
    }
    else{
        RCLCPP_DEBUG(this->get_logger(),
            "frames=%lu frames_overwritten=%lu incomplete_frames=%lu dropped_packets=%lu",
            (unsigned long)s.frames,(unsigned long)s.frames_overwritten,
            (unsigned long)s.incomplete_frames,(unsigned long)s.dropped);
    }
}

void TIRadarConnectNode::pub_radar_config_path(void){

    //declare a new string message
    std_msgs::msg::String message;

    //specify the message data
    message.data = radar_config_path;

    //publish the message
    radar_config_path_pub_ -> publish(message);

    return;
}

sensor_msgs::msg::PointCloud2 TIRadarConnectNode::get_pointcloud2_msg(
    const std::vector<cpsl::radar::Point> &detected_points,
    rclcpp::Time timestamp){
    
     // Create a PointCloud2 message
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    point_cloud_msg.header.frame_id = frame_id;
    point_cloud_msg.header.stamp = timestamp;
    
    // Set the height and width (height = 1 for unorganized point cloud)
    point_cloud_msg.height = 1;
    point_cloud_msg.width = detected_points.size();

    // Define the fields: x, y, z, vel
    point_cloud_msg.fields.resize(4);
    point_cloud_msg.fields[0].name = "x";
    point_cloud_msg.fields[1].name = "y";
    point_cloud_msg.fields[2].name = "z";
    point_cloud_msg.fields[3].name = "vel";

    int offset = 0;
    for (size_t i = 0; i < point_cloud_msg.fields.size(); ++i) {
        point_cloud_msg.fields[i].offset = offset;
        point_cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_msg.fields[i].count = 1;
        offset += 4;  // Each float is 4 bytes
    }

    // Set point step (total size of one point in bytes) and row step
    point_cloud_msg.point_step = offset;
    point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;
    
    // Resize the data buffer to hold all detected_points
    point_cloud_msg.data.resize(point_cloud_msg.row_step);
    
    // Set endianness and organization
    point_cloud_msg.is_bigendian = (__BYTE_ORDER == __BIG_ENDIAN);
    point_cloud_msg.is_dense = true;  // No invalid detected_points

    // Fill the data buffer with detected_points using a single iterator
    sensor_msgs::PointCloud2Iterator<float> iter(point_cloud_msg, "x");

    for (const auto &point : detected_points) {
        iter[0] = point.x;
        iter[1] = point.y;
        iter[2] = point.z;
        iter[3] = point.v;  // vel (snr_db / noise_db are not published: message layout unchanged)
        ++iter;  // Move to the next point
    }

    return point_cloud_msg;
}

raw_radar_msgs::msg::ADCDataCube TIRadarConnectNode::get_AdcDataCube_msg(
        const cpsl::radar::AdcFrame & frame,
        rclcpp::Time timestamp
    )
{
    const auto & adc_cube = frame.data;
    raw_radar_msgs::msg::ADCDataCube msg = raw_radar_msgs::msg::ADCDataCube();
  if(adc_cube.size() > 0){
    //stamp the header
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;

    //save the real and imaginary data
    save_adc_cube_to_msg(adc_cube,msg);

    //define the layout
    msg.layout.dim.push_back(raw_radar_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].label = "rx_channel";
    msg.layout.dim[0].size = adc_cube.size();
    msg.layout.dim[0].stride = adc_cube[0].size() * adc_cube[0][0].size();
    msg.layout.dim.push_back(raw_radar_msgs::msg::MultiArrayDimension());
    msg.layout.dim[1].label = "sample";
    msg.layout.dim[1].size = adc_cube[0].size();
    msg.layout.dim[1].stride = adc_cube[0][0].size();
    msg.layout.dim.push_back(raw_radar_msgs::msg::MultiArrayDimension());
    msg.layout.dim[2].label = "chirp";
    msg.layout.dim[2].size = adc_cube[0][0].size();
    msg.layout.dim[2].stride = 1;
  }

  return msg;
}

void TIRadarConnectNode::save_adc_cube_to_msg(
    const std::vector<std::vector<std::vector<std::complex<std::int16_t>>>> & adc_cube,
    raw_radar_msgs::msg::ADCDataCube & msg
){

  size_t samples_per_frame = adc_cube.size() * adc_cube[0].size() * adc_cube[0][0].size();
  size_t chirps_per_frame = adc_cube[0][0].size();
  size_t samples_per_chirp = adc_cube[0].size();
  size_t num_rxs = adc_cube.size();

  //pre-allocate the buffer data
  msg.imag_data = std::vector<std::int16_t>(samples_per_frame,0);
  msg.real_data = std::vector<std::int16_t>(samples_per_frame,0);

  size_t out_idx = 0;

  for (size_t rx_idx = 0; rx_idx < num_rxs; rx_idx++)
  {
    for (size_t sample_idx = 0; sample_idx < samples_per_chirp; sample_idx ++)
    {
      for (size_t chirp_idx = 0; chirp_idx < chirps_per_frame; chirp_idx ++){

        msg.real_data[out_idx] = adc_cube[rx_idx][sample_idx][chirp_idx].real();
        msg.imag_data[out_idx] = adc_cube[rx_idx][sample_idx][chirp_idx].imag();

        //increment the out index
        out_idx += 1;
      }
    }
  }
}