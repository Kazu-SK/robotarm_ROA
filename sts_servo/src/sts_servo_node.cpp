
#include "sts_servo/sts_servo.hpp"


int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<STSServo>());
	rclcpp::shutdown();

	return 0;
}
