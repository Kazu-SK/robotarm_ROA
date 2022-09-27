
#include "test_operation/test_operation.hpp"



void TestOperation::SubscribeCurrentPosition(const sts_servo_msgs::msg::CurrentPosition::SharedPtr msg){

	int servo_id_size = msg->current_position.size();


	for(int servo_id = 0 ; servo_id < servo_id_size ; servo_id++){

		RCLCPP_INFO(this->get_logger(),"current_position[%d] = %d\n", servo_id, msg->current_position[servo_id]); 
	}


}


void TestOperation::SubscribeCurrentVelocity(const sts_servo_msgs::msg::CurrentVelocity::SharedPtr msg){

	int servo_id_size = msg->current_velocity.size();


	for(int servo_id = 0 ; servo_id < servo_id_size ; servo_id++){

		RCLCPP_INFO(this->get_logger(),"current_velocity[%d] = %d\n", servo_id, msg->current_velocity[servo_id]); 
	}


}


void TestOperation::SubscribeCurrentLoad(const sts_servo_msgs::msg::CurrentLoad::SharedPtr msg){

	int servo_id_size = msg->current_load.size();


	for(int servo_id = 0 ; servo_id < servo_id_size ; servo_id++){

		RCLCPP_INFO(this->get_logger(),"current_load[%d] = %d\n", servo_id, msg->current_load[servo_id]); 
	}

}



void TestOperation::PublishPositionCommand(){

	unsigned char servo_id_list[NUMBER_OF_SERVO] = {ID_0, ID_1, ID_2, ID_3, ID_4, ID_5};
	unsigned char acceleration[NUMBER_OF_SERVO] = {100, 100, 100, 100, 100, 100};
	unsigned short velocity[NUMBER_OF_SERVO] = {1500, 1500, 1500, 1500, 1500, 1500}; 
	unsigned short input_position[2] = {1500, 2500};

	sts_servo_msgs::msg::PositionCommand array;

	array.servo_id.resize(NUMBER_OF_SERVO);
	array.acceleration.resize(NUMBER_OF_SERVO);
	array.position.resize(NUMBER_OF_SERVO);
	array.velocity.resize(NUMBER_OF_SERVO);

	RCLCPP_INFO(this->get_logger(),"PublishPositionCommand"); 

	for(int servo_id = 0 ; servo_id < NUMBER_OF_SERVO ; servo_id++){
		
		array.servo_id[servo_id] = servo_id_list[servo_id];
		array.acceleration[servo_id] = acceleration[servo_id];
		array.position[servo_id] = input_position[loop_count_];
		array.velocity[servo_id] = velocity[servo_id];
	}	
	
	if(loop_count_ == 0){

		loop_count_ = 1;
	}
	else{

		loop_count_ = 0;
	}

	position_command_pub_->publish(array);

}

