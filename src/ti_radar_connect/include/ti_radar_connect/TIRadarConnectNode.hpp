#ifndef TI_RADAR_CONNECT_NODE
#define TI_RADAR_CONNECT_NODE

#include <string>
#include <cstdlib>
#include <endian.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "radar_msgs/msg/multi_array_dimension.hpp"
#include "radar_msgs/msg/adc_data_cube.hpp"

// #include "Runner.hpp"
#include "Runner.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class TIRadarConnectNode : public rclcpp::Node
{
//functions
public:
    TIRadarConnectNode();
    ~TIRadarConnectNode();
private:
    //running the radar
    void run_ti_radar(void);

    //publishing data
    void pub_radar_config_path(void);

    //generating messages
    sensor_msgs::msg::PointCloud2 get_pointcloud2_msg(
        std::vector<std::vector<float>> &detected_points,
        rclcpp::Time timestamp);
    
    radar_msgs::msg::ADCDataCube get_AdcDataCube_msg(
        std::vector<std::vector<std::vector<std::complex<std::int16_t>>>> & adc_cube,
        rclcpp::Time timestamp
    );
    void save_adc_cube_to_msg(
        std::vector<std::vector<std::vector<std::complex<std::int16_t>>>> & adc_cube,
        radar_msgs::msg::ADCDataCube & msg
    );
//variables
public:

private:

    //config paths
    std::string config_path;
    std::string frame_id;
    double stamp_delay_sec;
    rclcpp::Duration stamp_delay; 
    std::string radar_config_path;
    Runner runner;

    //publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radar_config_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detected_points_pub_;
    rclcpp::Publisher<radar_msgs::msg::ADCDataCube>::SharedPtr adc_data_cube_pub_;
};

#endif