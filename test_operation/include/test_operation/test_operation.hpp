
#ifndef _TEST_OPERATION_HPP_
#define _TEST_OPERATION_HPP_


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


#include "sts_servo_msgs/msg/position_command.hpp"
#include "sts_servo_msgs/msg/current_position.hpp"
#include "sts_servo_msgs/msg/current_velocity.hpp"
#include "sts_servo_msgs/msg/current_load.hpp"


using namespace std::placeholders;
using namespace std::chrono_literals;


enum SERVO_ID{

	ID_0 = 0,
	ID_1,
	ID_2,
	ID_3,
	ID_4,
	ID_5,
	//ID_HAND,
	NUMBER_OF_SERVO

};


class TestOperation : public rclcpp::Node{

private:
	/*
	rclcpp::Subscription<sts_servo_msgs::msg::CurrentPosition>::SharedPtr current_position_sub_;
	rclcpp::Subscription<sts_servo_msgs::msg::CurrentVelocity>::SharedPtr current_velocity_sub_;
	rclcpp::Subscription<sts_servo_msgs::msg::CurrentLoad>::SharedPtr current_load_sub_;
	*/

	rclcpp::Publisher<sts_servo_msgs::msg::PositionCommand>::SharedPtr position_command_pub_;


	rclcpp::TimerBase::SharedPtr position_command_timer_;

	int loop_count_;

public:
	TestOperation() : Node("TestOperation"){

		/*
		current_position_sub_ = this->create_subscription<sts_servo_msgs::msg::CurrentPosition>("current_position_topic", 10, std::bind(&TestOperation::SubscribeCurrentPosition, this, _1));
		current_velocity_sub_ = this->create_subscription<sts_servo_msgs::msg::CurrentPosition>("current_velocity_topic", 10, std::bind(&TestOperation::SubscribeCurrentVelocity, this, _1));
		current_load_sub_ = this->create_subscription<sts_servo_msgs::msg::CurrentLoad>("current_load_topic", 10, std::bind(&TestOperation::SubscribeCurrentLoad, this, _1));

		*/
		position_command_timer_ = this->create_wall_timer(3000ms, std::bind(&TestOperation::PublishPositionCommand, this));
		position_command_pub_ = this->create_publisher<sts_servo_msgs::msg::PositionCommand>("position_command_topic", 10);

		RCLCPP_INFO(this->get_logger(),"TestOperation");

		loop_count_ = 0;
	}


	/*
	void SubscribeCurrentPosition(const sts_servo_msgs::msg::CurrentPosition::SharedPtr msg);
	void SubscribeCurrentVelocity(const sts_servo_msgs::msg::CurrentVelocity::SharedPtr msg);
	void SubscribeCurrentLoad(const sts_servo_msgs::msg::CurrentLoad::SharedPtr msg);
	*/

	void PublishPositionCommand();


};

#endif
