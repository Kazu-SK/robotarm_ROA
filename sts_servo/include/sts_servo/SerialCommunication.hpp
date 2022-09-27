
#ifndef _SERIAL_COMMAND_HPP_
#define _SERIAL_COMMAND_HPP_


#include<fcntl.h>
#include<sys/stat.h>
#include<sys/types.h>
#include<sys/ioctl.h>

#include<stdlib.h>
#include<stdio.h>
#include<unistd.h>
#include<termios.h>
#include<string.h>

#define DEVICE_FILE "/dev/ttyUSB0"
#define BAUDRATE B1000000

#include <iostream>

class SerialCommunication{

private:
    int fd;

public:
    int OpenDevice();
    int CloseDevice();

    int SerialRead(unsigned char reply_data[], int request_data_size);
    int SerialWrite(unsigned char write_data[],int write_data_size);

    SerialCommunication(){

        if(OpenDevice() < 0){
            perror("OpenDevice error ");
            exit(2);
        }
    }

    ~SerialCommunication(){

        if(CloseDevice() < 0){
            perror("CloseDevice() ");
            exit(2);
        }
    }
};


#endif
