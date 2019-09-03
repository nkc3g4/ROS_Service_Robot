/******************************************************************
功能如下：

dobot 串口通信
 *******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
//#include <iot_modules/IOTnet.h>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/Int32.h>

//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
#include <ctime>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
int module_adderss_S=0;  //发送的模块地址，
int module_adderss_R=0;  //接收到的模块地址
const float INTERVAL = 0.2;
const int DATALEN = 42;

/****************************************************/
serial::Serial my_serial;
//************************通讯参数配置*******************************************************
unsigned char data_terminal0=0x5A;
unsigned char module_command[DATALEN]={0};   //要发给串口的数据
//unsigned char motion_command[DATALEN]={0};

string rec_buffer;  //串口数据接收变量

union floatData 
{
    float d ;
    unsigned char data[4];
}module_data1,module_data2,module_data3,module_data4,module_data5,module_data6,module_data7,module_data8,module_data9,module_data10,command_data1,command_data2,command_data3,command_data4,command_data5,command_data6,command_data7,command_data8,command_data9,command_data10;




//********************************************************************************************/
void   Delay(int   time) {
clock_t   now   =   clock();

while(   clock()   -   now   <   time   );
}

//***********************************格式转换**************************************************************

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}


//***************************************************************************************88

//***********************串口函数**********************************************************
void serial_write(float state,float Axis,float X,float Y,float Z,float RHead,float isGrab,float StartVe,float EndVel,float MaxVe)
{
  /*  
    my_serial.setPort("/dev/Dobot0");
    my_serial.setBaudrate(9600);

    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    my_serial.setTimeout(to);
    my_serial.open();    
 */
    
    command_data1.d = state;
    command_data2.d = Axis;
    command_data3.d = X;
    command_data4.d = Y;
    command_data5.d = Z;
    command_data6.d = RHead;
    command_data7.d = isGrab;
    command_data8.d = StartVe;
    command_data9.d = EndVel;
    command_data10.d = MaxVe;
    
    module_command[0] = 0xA5;


    for(int i = 0;i < 4;i++) {
        module_command[i + 1] = command_data1.data[i];
        module_command[i + 5] = command_data2.data[i];
        module_command[i + 9] = command_data3.data[i];
        module_command[i + 13] = command_data4.data[i];
        
        module_command[i + 17] = command_data5.data[i];
        module_command[i + 21] = command_data6.data[i];
        module_command[i + 25] = command_data7.data[i];
        module_command[i + 29] = command_data8.data[i];
        
        module_command[i + 33] = command_data9.data[i];
        module_command[i + 37] = command_data10.data[i];
        
    //    ROS_INFO("command_data: %x %x %x %x %x %x %x %x %x %x",command_data1.data[i],command_data2.data[i],command_data3.data[i],command_data4.data[i],command_data5.data[i],command_data6.data[i],command_data7.data[i],command_data8.data[i],command_data9.data[i],command_data10.data[i]); 
    }
 
    module_command[41]=data_terminal0;
   
    my_serial.write(module_command,42);
//    ROS_INFO("already send data ");   //debug
}

void callback(const std_msgs::Int32::ConstPtr& msg){
    int zlow[20],zhigh[20];
    zlow[0]=-46;
    zhigh[0]=14;
    zhigh[1]=-11.1;
    zhigh[2]=2.7;
    zhigh[3]=-3.8;
    if(msg->data == 1) {
/*        serial_write(3,0,298,0,96.7,0,1,0,0,0);
        serial_write(3,0,298,0,-41,0,1,0,0,0);
        serial_write(3,0,298,0,20,0,1,0,0,0);
	serial_write(3,0,298,-76.4,20,0,1,0,0,0);
        serial_write(3,0,298,-76.4,-41,0,1,0,0,0);
        serial_write(3,0,298,-76.4,20,0,1,0,0,0);
        serial_write(3,0,298,43,20,0,1,0,0,0);
	serial_write(3,0,298,43,-41,0,1,0,0,0);
        serial_write(3,0,298,43,20,0,1,0,0,0);
        serial_write(3,0,251.3,0,-41,0,1,0,0,0);
        serial_write(3,0,251.3,0,20,0,1,0,0,0);
        serial_write(3,0,251.3,69.1,20,0,1,0,0,0);
        serial_write(3,0,251.3,69.1,-41,0,1,0,0,0);
        serial_write(3,0,251.3,69.1,20,0,1,0,0,0);
	serial_write(3,0,251.3,-76.4,20,0,1,0,0,0);
        serial_write(3,0,251.3,-76.4,-41,0,1,0,0,0);
        serial_write(3,0,251.3,-76.4,20,0,1,0,0,0);
        serial_write(3,0,251.3,0,112.4,0,1,0,0,0);
        serial_write(3,0,200.5,0,105.1,0,1,0,0,0);
        serial_write(3,0,184.8,0,-22.7,0,1,0,0,0);
*/
    serial_write(3,0,286,0,zlow[0],0,1,0,0,INTERVAL);
    serial_write(3,0,286,0,zhigh[0],0,1,0,0,INTERVAL);
//    Delay(10000);
//    serial_write(3,0,286,0,zhigh[0],0,1,0,0,0);
//    serial_write(3,0,286,-79.1,zhigh[0],0,1,0,0,0);
    serial_write(3,0,286,-79.1,zlow[0],0,1,0,0,INTERVAL);
    serial_write(3,0,286,-79.1,zhigh[0],0,1,0,0,INTERVAL);
//    serial_write(3,0,286,-79.1,zhigh[0],0,1,0,0,0);
//    serial_write(3,0,286,79.1,zhigh[0],0,1,0,0,0);
//    serial_write(3,0,286,79.1,zlow[0],0,1,0,0,0);
    serial_write(3,0,286,79.1,zlow[0],0,1,0,0,INTERVAL);
    serial_write(3,0,286,79.1,zhigh[0],0,1,0,0,INTERVAL);
//    serial_write(3,0,286,0,zhigh[0],0,1,0,0,0);
/*    serial_write(3,0,244.3,0,zhigh[0],0,1,0,0,0);
    serial_write(3,0,244.3,0,zlow[0],0,1,0,0,0);
    serial_write(3,0,244.3,0,zhigh[0],0,1,0,0,0);
    
    
    serial_write(3,0,243.8,-70,zhigh[1],0,1,0,0,0);
    serial_write(3,0,243.8,-70,zlow[0],0,1,0,0,0);

     serial_write(3,0,243.8,80.6,zhigh[1],0,1,0,0,0);
      serial_write(3,0,243.8,-0.3,zhigh[2],0,1,0,0,0);
*/
      serial_write(3,0,171.9,-0.3,zhigh[3],0,1,0,0,INTERVAL);


      serial_write(3,0,171.9,-0.3,zhigh[3],0,1,0,0,INTERVAL);




    }
    else if(msg->data == 2) {
        serial_write(3,0,269,-136,69,0,0,0,0,3);
        serial_write(3,0,131.8,0,0,0,0,0,0,0);
    }
//跳舞
    else if(msg->data == 3) {
        serial_write(3,0,286,-79.1,zhigh[0],0,1,0,0,0);
        serial_write(3,0,286,79.1,zhigh[0],0,1,0,0,0);
        serial_write(3,0,286,-79.1,zhigh[0],0,1,0,0,0);
        serial_write(3,0,286,79.1,zhigh[0],0,1,0,0,0);
        serial_write(3,0,286,-79.1,zhigh[0],0,1,0,0,0);
        serial_write(3,0,286,79.1,zhigh[0],0,1,0,0,0);
        
    }
    else if(msg->data == 4) {
	serial_write(7,1,0,0,0,0,0,15,0,0);
    }
    else if(msg->data == 5) {
        serial_write(7,2,0,0,0,0,0,15,0,0);
    }
    else if(msg->data == 6) {
        serial_write(7,3,0,0,0,0,0,15,0,0);
    }
    else if(msg->data == 7) {
        serial_write(7,4,0,0,0,0,0,15,0,0);
    }
    else if(msg->data == 8) {
        serial_write(7,5,0,0,0,0,0,15,0,0);
    }
    else if(msg->data == 9) {
        serial_write(7,6,0,0,0,0,0,15,0,0);
    }
    else if(msg->data == 10) {
        serial_write(7,9,0,0,0,0,0,1,0,0);
    }
    else if(msg->data == 11) {
        serial_write(7,10,0,0,0,0,0,1,0,0);
    }
    else if(msg->data == 99) {
        serial_write(3,0,131.8,0,0,0,0,0,0,0);
    }
    else if(msg->data == 1111) { // test code
	serial_write(2,11,0,0,0,0,0,1,0,0);
    }
    else if(msg->data == 1024) {
	serial_write(3,0,170.7,0,100,0,0,0,70,0.5);
	serial_write(3,0,230.7,0,90,0,0,0,70,0.5);
	serial_write(3,0,230.7,0,100,0,0,0,130,1);
	serial_write(3,0,170.7,0,60,0,0,0,130,2);
	serial_write(3,0,170.7,0,35,0,0,0,130,2);
//	serial_write(3,0,260.7,0,27.0,0,0,0,60,10);

    }
    else if(msg->data == 10241) { 
	serial_write(3,0,260.7,70,10,0,0,0,40,2);
	serial_write(3,0,180,0,27.0,0,0,0,130,2);
    }
/*    else if(msg->data == 3) {
	serial_write(7,10,0,0,0,0,0,1,0,0);
	ROS_INFO("7,10,0,0,0,0,0,1,0 success send message");
    }
    else if(msg->data == 4) {
        serial_write(7,10,0,0,0,0,0,1,0,0);
        ROS_INFO("7,0,0,0,0,0,0,1,0 success send message");
    }  
*/
} 


void motionControlCallback(const std_msgs::String::ConstPtr& motion_msg) {

 //    char motion_data_tmp[42];
 //   for(int i = 0;i < 42;i++) motion_data_tmp[i] = motion_msg->data[i];
 //  unsigned char* motion_command = motion_data_tmp;
 
/*
    string str = motion_msg->data;
    const unsigned char* motion_data = (unsigned char*)str.c_str();
 
*/
//    string tmp = motion_msg->data;
//    for(int i = 0;i < 42;i++) ROS_INFO("MD:%x",tmp[i]);  // debug
    my_serial.write(motion_msg->data);
    ROS_INFO("Already sent motion data");

}



//****************************主函数***********************************************************/
int main(int argc, char **argv) {
    ros::init(argc, argv, "dobot_integration_node");
    ros::NodeHandle n;  
    ROS_INFO("arduino node created successfully");
    

    char* my_retur;
   
//    string port("/dev/Dobot0");
  //  unsigned long baud = 9600;
//     string port("/dev/ttyUSB2");
//    unsigned long baud = 9600;
   ros::Subscriber sub_cmd = n.subscribe("/dobot_cmd",50, callback);
   ros::Subscriber sub_motion = n.subscribe("/dobot_cmd_motion",100, motionControlCallback);   
//    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
//    ros::Subscriber sub_nlu = n.subscribe("/voice_system/room_topic",20,subcallback_nlu);

    //ros::Publisher pub_words= n.advertise<std_msgs::String>("/speak_string", 200); 
//    ros::Publisher pub_words= n.advertise<std_msgs::String>("/voice_system/tts_topic", 200);


//    std_msgs::String words;//要说的话
    
    try{
	my_serial.setPort("/dev/Dobot0");
    	my_serial.setBaudrate(9600);

    	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    	my_serial.setTimeout(to);
    	serial::stopbits_t sb = serial::stopbits_one;
	my_serial.setStopbits(sb);
        my_serial.open();	
    }
    catch(serial::IOException& e){
        ROS_ERROR("Unable to open port");
    }

    if(my_serial.isOpen()){
        ROS_INFO("Serial Port initialized");
    }
  
    ros::Rate loop_rate(50);

    ROS_INFO("Dobot_integration node is running now");
  
    while(ros::ok())
    {     
          rec_buffer =my_serial.readline(60,"\n");
	  ROS_INFO("buffer length: %d",rec_buffer.length());
/*
            const char *receive_data=rec_buffer.data();
            ROS_INFO("receive_data[0]:%x",receive_data[0]); 
           
            ROS_INFO("read data len: %d ",rec_buffer.length());  //显示接收数据的长度

            if(rec_buffer.length()==42)
            {                  
                if(receive_data[0]==0xffffffA5)
                {   
                    module_adderss_R = receive_data[1];
                    ROS_INFO("receive_data[0]:%x",receive_data[0]);  //调试编码器使用，
                    for(int i=0;i<4;i++)
                    {                
                        module_data1.data[i]=receive_data[i+3];
                        module_data2.data[i]=receive_data[i+7];
                        module_data3.data[i]=receive_data[i+11];
                        module_data4.data[i]=receive_data[i+15];
                        ROS_INFO("receive_data1: %x ",receive_data[i+3]);                
                    }  
                    printf("module_data1:%f",module_data1.d); 
                }                   
            }
    */  

	ros::spinOnce();
  	loop_rate.sleep();
 	 
    }

    return 0;
}
