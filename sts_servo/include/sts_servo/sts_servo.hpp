
#ifndef STS_SERVO_HPP_
#define STS_SERVO_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"

#include "sts_servo/SerialCommunication.hpp"
#include "sts_servo/sts_servo_define.hpp"

#include "sts_servo_msgs/msg/position_command.hpp"
#include "sts_servo_msgs/msg/current_position.hpp"
#include "sts_servo_msgs/msg/current_velocity.hpp"
#include "sts_servo_msgs/msg/current_load.hpp"



using namespace std::placeholders;
using namespace std::chrono_literals;



class STSServo : public rclcpp::Node{

private:

	rclcpp::Subscription<sts_servo_msgs::msg::PositionCommand>::SharedPtr position_command_sub_;

	rclcpp::Publisher<sts_servo_msgs::msg::CurrentPosition>::SharedPtr current_position_pub_;
	rclcpp::Publisher<sts_servo_msgs::msg::CurrentVelocity>::SharedPtr current_velocity_pub_;
	rclcpp::Publisher<sts_servo_msgs::msg::CurrentLoad>::SharedPtr current_load_pub_;

	rclcpp::TimerBase::SharedPtr current_position_timer_;
	rclcpp::TimerBase::SharedPtr current_velocity_timer_;
	rclcpp::TimerBase::SharedPtr current_load_timer_;

	SerialCommunication *serial_communication_;


	static const unsigned char HEADER_LOW_BYTE_;
	static const unsigned char HEADER_HIGH_BYTE_;
	static const unsigned char POSITION_COMMAND_DATA_SIZE_;
	static const unsigned char READ_COMMAND_DATA_SIZE_;
	static const unsigned char MULTI_CONTROL_COMMAND_;
	static const unsigned char READ_COMMAND_;

	static const unsigned char ACCELATION_REGISTER_;
	static const unsigned char TARGET_POSITION_REGISTER_;
	static const unsigned char TARGET_VELOCITY_REGISTER_;

	static const unsigned char CURRENT_POSITION_REGISTER_;
	static const unsigned char CURRENT_VELOCITY_REGISTER_;
	static const unsigned char CURRENT_LOAD_REGISTER_;

	static const int REPLY_DATA_SIZE_;


public:

	STSServo() : Node("STSServo"){

		position_command_sub_ = this->create_subscription<sts_servo_msgs::msg::PositionCommand>("position_command_topic", 10, std::bind(&STSServo::OperateSTSServo, this, _1));

		
		current_position_pub_ = this->create_publisher<sts_servo_msgs::msg::CurrentPosition>("current_position_topic", 10);
		current_velocity_pub_ = this->create_publisher<sts_servo_msgs::msg::CurrentVelocity>("current_velocity_topic", 10);
		current_load_pub_ = this->create_publisher<sts_servo_msgs::msg::CurrentLoad>("current_load_topic", 10);

		current_position_timer_ = this->create_wall_timer(120ms, std::bind(&STSServo::PublishCurrentPosition, this));
		current_velocity_timer_ = this->create_wall_timer(120ms, std::bind(&STSServo::PublishCurrentVelocity, this));
		current_load_timer_ = this->create_wall_timer(120ms, std::bind(&STSServo::PublishCurrentLoad, this));
		
		serial_communication_ = new SerialCommunication;
	}


	~STSServo(){

		delete serial_communication_;
	}


	void OperateSTSServo(const sts_servo_msgs::msg::PositionCommand::SharedPtr msg); 

	void PublishCurrentPosition();
	void PublishCurrentVelocity();
	void PublishCurrentLoad();

};

#endif
