/*****************************************************************
功能如下：

串口通信说明：
1.写入串口
                               
（1）内容：         地址  读/写命令  灯光   窗帘   电源   风扇 
（2）格式：  oxA5    1     1/2     1/2    1/2   1/2   1/2   0x0d 0x0a
2.读取串口                        光照度   温度   湿度   CO2
（1）内容                         煤气     漏水
（2）格式：
*******************************************************************/
#include <ros/ros.h>  //ros需要的头文件
#include <ros/console.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ros_arduino_msgs/Digital.h>
#include <ros_arduino_msgs/Analog.h>
#include <string>        
#include <iostream>
#include <cstdio>
#include <math.h>


using namespace std;

ros::Subscriber arduino_voice_sub;
ros::Subscriber ifnlu_sub;
ros::Subscriber sub_remote;
ros::Subscriber sub_motion;
ros::Publisher  arduino_voice_pub;
ros::Publisher  dobot_pub;

std_msgs::String currentMsg;

int id = -1;
int pre = -1;
int counter = 0;
int flag = 0;



void voice_subCallBack(const ros_arduino_msgs::Digital & msg){
    ROS_INFO("%x",msg.value);
    if(msg.value == 0x3D9AE3F7) {
        flag = 1;
    }
    else {
	flag = 2;
    }


}

void nlu_subCallBack(const std_msgs::Int32::ConstPtr& msg){

    if(msg->data == 10){
    	flag = 1;
    }
    else{
     	flag = 0;
    }
}

void subcallback_remote(const ros_arduino_msgs::Analog & remote_cmd) {
    ROS_INFO("remote_cmd: %d",remote_cmd.value);
    if(remote_cmd.value == 38678) {
	for(int i = 0;i < 10;i++) dobot_pub.publish(4);
    }
    else if (remote_cmd.value == 35874) {
        for(int i = 0;i < 10;i++) dobot_pub.publish(5);
    }
    else if (remote_cmd.value == 15770) {
        for(int i = 0;i < 10;i++) dobot_pub.publish(6);
    }
    else if (remote_cmd.value == 18575) {
        for(int i = 0;i < 10;i++) dobot_pub.publish(7);
    }
    else if (remote_cmd.value == 24962) {
        for(int i = 0;i < 10;i++) dobot_pub.publish(8);
    }
    else if (remote_cmd.value == 1097) {
        for(int i = 0;i < 10;i++) dobot_pub.publish(9);
    }
    else if (remote_cmd.value == 41928) {
        for(int i = 0;i < 10;i++) dobot_pub.publish(10);
    }
    else if (remote_cmd.value == 49409) {
        for(int i = 0;i < 10;i++) dobot_pub.publish(11);
    }
}

/*
void subcallback_motion(const std_msgs::String msg) {
        
    char tmp[42];
    for(int i = 0;i < 42;i++) tmp[i] = msg.data[i];


    ROS_INFO("Motion INFO: %s",tmp);
    cout << msg.data;



}
*/

int main(int argc, char **argv){

    ros::init(argc, argv, "arduino_integration_node");
    ros::NodeHandle n;  

    ros::Rate loop_rate(5);

    ROS_INFO("arduino_integration node is running now");
   

    ifnlu_sub = n.subscribe("/voice_system/room_topic",20,nlu_subCallBack);
    sub_remote = n.subscribe("/my_arduino/sensor/dir_sensor",20,subcallback_remote);
//    sub_motion = n.subscribe("/my_arduino/sensor/dir_sensor",20,subcallback_motion);

    arduino_voice_pub= n.advertise<std_msgs::String>("/voice_system/tts_topic", 20);
    dobot_pub = n.advertise<std_msgs::Int32>("/dobot_cmd",20);

    while(ros::ok()){
	loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;

}
