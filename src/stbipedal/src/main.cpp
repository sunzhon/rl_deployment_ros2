#include "node.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotRosNode>());
	rclcpp::shutdown();
	return 0;
}

