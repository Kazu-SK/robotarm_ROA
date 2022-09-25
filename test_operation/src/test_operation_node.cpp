
#include "test_operation/test_operation.hpp"


int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TestOperation>());
	rclcpp::shutdown();

	return 0;
}
