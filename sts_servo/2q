
#ifndef STS_SERVO_DEFINE_HPP_
#define STS_SERVO_DEFINE_HPP_


enum SERVO_ID{

	//一軸目のサーボモータのIDは0とする。

	ID_0 = 0,
	ID_1,
	ID_2,
	ID_3,
	ID_4,
	ID_5,
	//ID_HAND,
	NUMBER_OF_SERVO

};

//位置指令コマンド、読み出しコマンドのフォーマットについては
//FEETECH社デジタルサーボ使用方法 参照
//https://akizukidenshi.com/download/ds/akizuki/feetech_digital_servo_20220729.pdf

typedef struct PositionCommandFormat{

	unsigned char header_LOW_BYTE;
	unsigned char header_HIGH_BYTE;
	unsigned char id;
	unsigned char packet_data_size;
	unsigned char command;
	unsigned char register_top;
	unsigned char acceleration;
	unsigned char position_LOW_BYTE;
	unsigned char position_HIGH_BYTE;
	unsigned char time_LOW_BYTE;
	unsigned char time_HIGH_BYTE;
	unsigned char velocity_LOW_BYTE;
	unsigned char velocity_HIGH_BYTE;
	unsigned char check_sum;

}PositionCommandFormat;


typedef struct ReadCommandFormat{

	unsigned char header_LOW_BYTE;
	unsigned char header_HIGH_BYTE;
	unsigned char id;
	unsigned char packet_data_size;
	unsigned char command;
	unsigned char register_top;
	unsigned char request_data_size;
	unsigned char check_sum;

}ReadCommandFormat;

#endif
