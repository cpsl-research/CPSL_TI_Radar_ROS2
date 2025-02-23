#include "TIRadarConnectNode.hpp"

TIRadarConnectNode::TIRadarConnectNode():
    Node("ti_radar_connect_node"),
    config_path(""),
    frame_id(""),
    stamp_delay_sec(),
    stamp_delay(rclcpp::Duration::from_seconds(0.0)),
    radar_config_path(),
    runner(),
    radar_config_path_pub_(),
    detected_points_pub_()
{
    //declare parameters
    this->declare_parameter<std::string>("config_path");
    this->declare_parameter<std::string>("frame_id","radar_0");
    this->declare_parameter<double>("stamp_delay_sec",0.0);

    //load in parameters
    config_path = this->get_parameter("config_path").as_string();
    frame_id = this->get_parameter("frame_id").as_string();
    stamp_delay_sec = this->get_parameter("stamp_delay_sec").as_double();

    //initialize the stamp delay
    stamp_delay = rclcpp::Duration::from_seconds(stamp_delay_sec);
    //initialize initialize the runner
    runner.initialize(config_path);

    //stop if initialization failed
    if(!runner.initialized){
        return;
    }
    
    //define a qos profile for some publishers
    rclcpp::QoS qos_profile = rclcpp::QoS(5); //queue size of 1
    qos_profile.transient_local();
    qos_profile.reliable();


    radar_config_path_pub_ = 
        this->create_publisher<std_msgs::msg::String>(frame_id + "/radar_config_path",qos_profile);
    
    if(runner.get_serial_streaming_enabled()){
        std::string detected_points_topic = frame_id + "/detected_points";
        detected_points_pub_ = 
            this->create_publisher<sensor_msgs::msg::PointCloud2>(detected_points_topic,qos_profile);
    }

    if(runner.get_dca1000_streaming_enabled()){
        std::string adc_cube_topic = frame_id + "/adc_data_cube";
        adc_data_cube_pub_ = 
            this->create_publisher<radar_msgs::msg::ADCDataCube>(adc_cube_topic,qos_profile);
    }
    

    //publish the radar_config_path
    radar_config_path = runner.get_radar_config_path();
    pub_radar_config_path();

    run_ti_radar();
}

TIRadarConnectNode::~TIRadarConnectNode(){}

void TIRadarConnectNode::run_ti_radar(void){

    //define the timeout in ms
    int timeout_ms = 2000;

    //enter the run loop
    if(runner.initialized){
        
        //start the runner
        runner.start();

        while(rclcpp::ok()){
            //handle point clouds
            if(runner.get_serial_streaming_enabled()){
                std::vector<std::vector<float>> detected_points = 
                    runner.get_next_tlv_detected_points(timeout_ms);
                
                //get the point cloud 2 message
                sensor_msgs::msg::PointCloud2 pc2_msg = get_pointcloud2_msg(
                    detected_points,
                    this -> now() - stamp_delay
                );

                //publish the point cloud 2 message
                detected_points_pub_ -> publish(pc2_msg);
            }

            //handle ethernet packets
            if(runner.get_dca1000_streaming_enabled()){
                std::vector<std::vector<std::vector<std::complex<std::int16_t>>>> adc_cube =
                    runner.get_next_adc_cube(timeout_ms);
                
                //get the adc_data_cube message
                radar_msgs::msg::ADCDataCube adc_cube_msg = get_AdcDataCube_msg(
                    adc_cube,
                    this -> now() - stamp_delay
                );

                adc_data_cube_pub_ -> publish(adc_cube_msg);
            }
        }

        //stop the runner
        runner.stop();
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
    std::vector<std::vector<float>> &detected_points,
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
        iter[0] = point[0];  // x
        iter[1] = point[1];  // y
        iter[2] = point[2];  // z
        iter[3] = point[3];  // vel
        ++iter;  // Move to the next point
    }

    return point_cloud_msg;
}

radar_msgs::msg::ADCDataCube TIRadarConnectNode::get_AdcDataCube_msg(
        std::vector<std::vector<std::vector<std::complex<std::int16_t>>>> & adc_cube,
        rclcpp::Time timestamp
    )
{
    radar_msgs::msg::ADCDataCube msg = radar_msgs::msg::ADCDataCube();
  if(adc_cube.size() > 0){
    //stamp the header
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;

    //save the real and imaginary data
    save_adc_cube_to_msg(adc_cube,msg);

    //define the layout
    msg.layout.dim.push_back(radar_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].label = "rx_channel";
    msg.layout.dim[0].size = adc_cube.size();
    msg.layout.dim[0].stride = adc_cube[0].size() * adc_cube[0][0].size();
    msg.layout.dim.push_back(radar_msgs::msg::MultiArrayDimension());
    msg.layout.dim[1].label = "sample";
    msg.layout.dim[1].size = adc_cube[0].size();
    msg.layout.dim[1].stride = adc_cube[0][0].size();
    msg.layout.dim.push_back(radar_msgs::msg::MultiArrayDimension());
    msg.layout.dim[2].label = "chirp";
    msg.layout.dim[2].size = adc_cube[0][0].size();
    msg.layout.dim[2].stride = 1;
  }

  return msg;
}

void TIRadarConnectNode::save_adc_cube_to_msg(
    std::vector<std::vector<std::vector<std::complex<std::int16_t>>>> & adc_cube,
    radar_msgs::msg::ADCDataCube & msg
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
      for (size_t chirp_idx = 0; chirps_per_frame; chirp_idx ++){

        msg.real_data[out_idx] = adc_cube[rx_idx][sample_idx][chirp_idx].real();
        msg.imag_data[out_idx] = adc_cube[rx_idx][sample_idx][chirp_idx].imag();

        //increment the out index
        out_idx += 1;
      }
    }
  }
}