#include <string>
#include <cstdlib>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "TIRadarConnectNode.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TIRadarConnectNode>();
    // The node runs its capture loop inside its constructor and only returns
    // once rclcpp::ok() goes false (SIGINT). Spinning a shut-down context then
    // throws RCLError and aborts the process instead of letting it exit
    // cleanly, so only spin while the context is still valid.
    if(rclcpp::ok()){
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}