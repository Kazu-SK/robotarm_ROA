
#include "sts_servo/sts_servo.hpp"

//通信フォーマットのヘッダー、コマンド、返信データのデータサイズの値については
//FEETECH社デジタルサーボ使用方法 参照
//https://akizukidenshi.com/download/ds/akizuki/feetech_digital_servo_20220729.pdf

const unsigned char STSServo::HEADER_LOW_BYTE_ = 0xFF;
const unsigned char STSServo::HEADER_HIGH_BYTE_ = 0xFF;
const unsigned char STSServo::POSITION_COMMAND_DATA_SIZE_ = 0x0A;
const unsigned char STSServo::READ_COMMAND_DATA_SIZE_ = 0x04;
const unsigned char STSServo::MULTI_CONTROL_COMMAND_ = 0x03;
const unsigned char STSServo::READ_COMMAND_ = 0x02;


//レジスタの値については
//サーボモータの取説を参照
//https://akizukidenshi.com/catalog/g/gM-16311/  
//FD1.9.8-EN_200923.zip
const unsigned char STSServo::ACCELATION_REGISTER_ = 0x29;
const unsigned char STSServo::TARGET_POSITION_REGISTER_ = 0x2A;
const unsigned char STSServo::TARGET_VELOCITY_REGISTER_ = 0x2E;

const unsigned char STSServo::CURRENT_POSITION_REGISTER_ = 0x38;
const unsigned char STSServo::CURRENT_VELOCITY_REGISTER_ = 0x3A;
const unsigned char STSServo::CURRENT_LOAD_REGISTER_ = 0x3C;

const int STSServo::REPLY_DATA_SIZE_ = 8;



void STSServo::OperateSTSServo(const sts_servo_msgs::msg::PositionCommand::SharedPtr msg){

	PositionCommandFormat format;

	unsigned short sum = 0; //header_LOW_BYTE〜velocity_HIGH_BYTEの値がすべて0xffでも合計が65535を超えない。

	int servo_id_size = msg->servo_id.size();
	int acceleration_size = msg->acceleration.size();
	int position_size = msg->position.size();
	int velocity_size = msg->velocity.size();

	if(!(servo_id_size == acceleration_size && servo_id_size == position_size && servo_id_size == velocity_size)){

		RCLCPP_INFO(this->get_logger(),"STSServo OperateSTSServo : size mismatch");	
		return;
	}

	for(int id = ID_0 ; id < servo_id_size ; id++){ 

		sum = 0;

		format.header_LOW_BYTE = HEADER_LOW_BYTE_;
		format.header_HIGH_BYTE = HEADER_HIGH_BYTE_;
		sum += format.id = msg->servo_id[id];
		sum += format.packet_data_size = POSITION_COMMAND_DATA_SIZE_;
		sum += format.command = MULTI_CONTROL_COMMAND_;
		sum += format.register_top = ACCELATION_REGISTER_;
		sum += format.acceleration = msg->acceleration[id];
		sum += format.position_LOW_BYTE = (unsigned char)(msg->position[id] & 0xFF);
		sum += format.position_HIGH_BYTE = (unsigned char)((msg->position[id] >> 8) & 0xFF);

		//マルチターンコントロールでは、時間情報は無効となるため、下位,上位バイトともに0
		sum += format.time_LOW_BYTE = 0;
		sum += format.time_HIGH_BYTE = 0;
		sum += format.velocity_LOW_BYTE = (unsigned char)(msg->velocity[id] & 0xFF);
		sum += format.velocity_HIGH_BYTE = (unsigned char)((msg->velocity[id] >> 8) & 0xFF);

		format.check_sum = ~((unsigned char)(sum & 0xFF));

		rclcpp::sleep_for(5ms);

		serial_communication_->SerialWrite((unsigned char *)&format, sizeof(format));

	}

}


void STSServo::PublishCurrentPosition(){

	ReadCommandFormat format;

	unsigned short sum = 0; 
	sts_servo_msgs::msg::CurrentPosition array;
	unsigned char reply_data[REPLY_DATA_SIZE_];

	array.current_position.resize(NUMBER_OF_SERVO);

	RCLCPP_INFO(this->get_logger(),"PublishCurrentPosition");

	for(unsigned char id = ID_0 ; id < NUMBER_OF_SERVO ; id++){

		sum = 0;

		format.header_LOW_BYTE = HEADER_LOW_BYTE_;	
		format.header_HIGH_BYTE = HEADER_HIGH_BYTE_;	
		sum += format.id = id;
		sum += format.packet_data_size = READ_COMMAND_DATA_SIZE_;
		sum += format.command = READ_COMMAND_;
		sum += format.register_top = CURRENT_POSITION_REGISTER_;
		sum += format.request_data_size = sizeof(array.current_position[id]);
		format.check_sum = ~((unsigned char)(sum & 0xFF));
		
		rclcpp::sleep_for(5ms);

		if(serial_communication_->TciFlush() < 0){

			std::cout<<"tcflush(fd, TCIFLUSH) error"<<std::endl;
		}

		serial_communication_->SerialWrite((unsigned char *)&format, sizeof(format));

		//要求データは6,7バイト目に格納される。
		//FEETECH社デジタルサーボ使用方法 参照
		//https://akizukidenshi.com/download/ds/akizuki/feetech_digital_servo_20220729.pdf

		rclcpp::sleep_for(5ms);
		serial_communication_->SerialRead(reply_data, sizeof(reply_data));

		array.current_position[id] = (unsigned short)((reply_data[6] << 8) | reply_data[5]);

	}	

	current_position_pub_->publish(array);
}


void STSServo::PublishCurrentVelocity(){

	ReadCommandFormat format;

	unsigned short sum = 0; 
	sts_servo_msgs::msg::CurrentVelocity array;
	unsigned char reply_data[REPLY_DATA_SIZE_];


	array.current_velocity.resize(NUMBER_OF_SERVO);

	RCLCPP_INFO(this->get_logger(),"PublishCurrentVelocity");

	for(unsigned char id = ID_0 ; id < NUMBER_OF_SERVO ; id++){

		sum = 0;

		format.header_LOW_BYTE = HEADER_LOW_BYTE_;	
		format.header_HIGH_BYTE = HEADER_HIGH_BYTE_;	
		sum += format.id = id;
		sum += format.packet_data_size = READ_COMMAND_DATA_SIZE_;
		sum += format.command = READ_COMMAND_;
		sum += format.register_top = CURRENT_VELOCITY_REGISTER_;
		sum += format.request_data_size = sizeof(array.current_velocity[id]);
		format.check_sum = ~((unsigned char)(sum & 0xFF));

		rclcpp::sleep_for(5ms);

		if(serial_communication_->TciFlush() < 0){

			std::cout<<"tcflush(fd, TCIFLUSH) error"<<std::endl;
		}

		serial_communication_->SerialWrite((unsigned char *)&format, sizeof(format));

		rclcpp::sleep_for(5ms);
		serial_communication_->SerialRead(reply_data, sizeof(reply_data));

		array.current_velocity[id] = (unsigned short)((reply_data[6] << 8) | reply_data[5]);

	}	

	current_velocity_pub_->publish(array);
}


void STSServo::PublishCurrentLoad(){

	ReadCommandFormat format;

	unsigned short sum = 0; 
	sts_servo_msgs::msg::CurrentLoad array;
	unsigned char reply_data[REPLY_DATA_SIZE_];


	array.current_load.resize(NUMBER_OF_SERVO);

	RCLCPP_INFO(this->get_logger(), "PublishCurrentLoad");

	for(unsigned char id = ID_0 ; id < NUMBER_OF_SERVO ; id++){

		sum = 0;

		format.header_LOW_BYTE = HEADER_LOW_BYTE_;	
		format.header_HIGH_BYTE = HEADER_HIGH_BYTE_;	
		sum += format.id = id;
		sum += format.packet_data_size = READ_COMMAND_DATA_SIZE_;
		sum += format.command = READ_COMMAND_;
		sum += format.register_top = CURRENT_LOAD_REGISTER_;
		sum += format.request_data_size = sizeof(array.current_load[id]);
		format.check_sum = ~((unsigned char)(sum & 0xFF));

		rclcpp::sleep_for(5ms);

		if(serial_communication_->TciFlush() < 0){

			std::cout<<"tcflush(fd, TCIFLUSH) error"<<std::endl;
		}

		serial_communication_->SerialWrite((unsigned char *)&format, sizeof(format));

		rclcpp::sleep_for(5ms);
		serial_communication_->SerialRead(reply_data, sizeof(reply_data));

		array.current_load[id] = (unsigned short)((reply_data[6] << 8) | reply_data[5]);

	}	

	current_load_pub_->publish(array);
}
