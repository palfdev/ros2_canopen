#include "rclcpp/rclcpp.hpp"
#include "canopen_mock_slave/oem_slave.hpp"


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;
	auto canopen_slave = std::make_shared<ros2_canopen::oemSlave>("oem_slave");
	executor.add_node(canopen_slave->get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();
	return 0;
}