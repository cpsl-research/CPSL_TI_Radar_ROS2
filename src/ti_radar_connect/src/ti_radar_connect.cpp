#include <string>
#include <cstdlib>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "TIRadarConnectNode.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TIRadarConnectNode>());
    rclcpp::shutdown();
    return 0;
}