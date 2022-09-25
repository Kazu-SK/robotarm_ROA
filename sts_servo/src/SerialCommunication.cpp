
#include "sts_servo/SerialCommunication.hpp"


int SerialCommunication::OpenDevice(){

    struct termios oldtio, newtio;


    fd = 0;
    fd = open(DEVICE_FILE , O_RDWR | O_NOCTTY | O_SYNC);

    if(fd < 0){
        return -1;
    }

    tcgetattr(fd, &oldtio);

    bzero(&newtio, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD; //通信速度,8ビットデータ通信,フロー制御無効,読み出し可
    newtio.c_iflag = IGNPAR; //パリティエラー無視
    newtio.c_oflag = 0;

    newtio.c_lflag = 0; //非カノニカルモード

    newtio.c_cc[VTIME] = 3;//タイムアウト付き読み出し (0.3s)
    newtio.c_cc[VMIN] = 0;//

 /*
  *
  * */

    tcflush(fd, TCIFLUSH); //受信データ破棄
    tcsetattr(fd, TCSANOW, &newtio); //シリアル通信設定が即時反映

    //ioctl(fd,TCSETS,&newtio);


    return 0;
}


int SerialCommunication::SerialWrite(unsigned char write_data[], int write_data_size){

    int w_rtn;

    w_rtn = write(fd,write_data,write_data_size);

    return w_rtn;
}


int SerialCommunication::SerialRead(unsigned char reply_data[], int request_data_size){

    int r_rtn;
    int read_data_size = 0;


    while(read_data_size < request_data_size){

        r_rtn = read(fd, reply_data + read_data_size, request_data_size - read_data_size);
        
        if(r_rtn < 0)return r_rtn;

        read_data_size += r_rtn;
    }


    return r_rtn;
}


int SerialCommunication::CloseDevice(){

    int rtn;


    rtn = close(fd);
    
    if(rtn == -1){
        
        perror("Error massage ");
        return rtn;
    }

    printf("file closed\n");

    return rtn;
}


