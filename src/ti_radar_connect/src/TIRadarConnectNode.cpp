#include "TIRadarConnectNode.hpp"

TIRadarConnectNode::TIRadarConnectNode():
    Node("ti_radar_connect_node"),
    config_path(""),
    radar_config_path(),
    runner(),
    radar_config_path_pub_(),
    detected_points_pub_()
{
    //declare parameters
    this->declare_parameter<std::string>("config_path");

    //load in parameters
    config_path = this->get_parameter("config_path").get_value<std::string>();

    //initialize initialize the runner
    runner.initialize(config_path);

    //stop if initialization failed
    if(!runner.initialized){
        return;
    }
    
    //define a qos profile for some publishers
    rclcpp::QoS qos_profile = rclcpp::QoS(1); //queue size of 1
    qos_profile.transient_local();
    qos_profile.reliable();


    radar_config_path_pub_ = 
        this->create_publisher<std_msgs::msg::String>("radar_config_path",qos_profile);
    detected_points_pub_ = 
        this->create_publisher<sensor_msgs::msg::PointCloud2>("detected_points",qos_profile);

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
                    this -> get_namespace(),
                    this -> now()
                );

                //publish the point cloud 2 message
                detected_points_pub_ -> publish(pc2_msg);
            }

            //handle ethernet packets
            if(runner.get_dca1000_streaming_enabled()){
                std::vector<std::vector<std::vector<std::complex<std::int16_t>>>> adc_cube =
                    runner.get_next_adc_cube(timeout_ms);
                //TODO: add code to get the next adc cube
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
    const std::string &frame_id,
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