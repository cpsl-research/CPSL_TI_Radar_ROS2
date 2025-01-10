#ifndef TI_RADAR_CONNECT_NODE
#define TI_RADAR_CONNECT_NODE

#include <string>
#include <cstdlib>
#include <endian.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

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
        const std::string &frame_id,
        rclcpp::Time timestamp);
//variables
public:

private:

    //config paths
    std::string config_path;
    std::string radar_config_path;
    Runner runner;

    //publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radar_config_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detected_points_pub_;
};

#endif