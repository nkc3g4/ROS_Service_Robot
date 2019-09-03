
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include <geometry_msgs/PoseStamped.h>
#include<stdio.h>

#define BACK_HOME_CMD    18
#define GO_BEDROOM_CMD   19  
#define GO_WINDOW_CMD    20
#define GO_DOOR_CMD      21
#define GO_KITCHEN_CMD   22
#define GO_AWAY_CMD      99
//TODO 1/3


ros::Publisher pub;
/*
float speed_x = 0.2;
float speed_y = 0.2;
float turn_speed = 0.5;
*/
int pub_flag = 0;
int id = -1;
int pre = -1; 
geometry_msgs::PoseStamped mb_msg;


void subCallBack(const std_msgs::Int32::ConstPtr& msg){
    ROS_WARN_STREAM(msg->data);

    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;
    switch(msg->data){
        case BACK_HOME_CMD:

        position.x = -0.223930597305;
	      position.y = -0.460890650749;
   	    position.z = 0.0;

  	    orientation.x = 0.0;
        orientation.y = 0.0;
        orientation.z = 0.885530542644;
        orientation.w = 0.464581164109;
	      mb_msg.pose.position = position;
	      mb_msg.pose.orientation = orientation;
	    
	      mb_msg.header.seq = 0;
        mb_msg.header.stamp;
	      mb_msg.header.frame_id = "map";
	      pre = id;
	      id = 1;
        break;

        case GO_BEDROOM_CMD:
        pre = id;
        id = 2;
        
        break;

        case GO_WINDOW_CMD:
        pre = id;
        id = 3;
        
        break;

        case GO_DOOR_CMD:
        pre = id;
        id = 4;
       
        break;

        case GO_KITCHEN_CMD:
        pre = id;
        id = 5;
        break;
        //TODO: 2/3

        case GO_AWAY_CMD: //move back
	
	      position.x = 1.14593064785;
        position.y = 0.015246629715;
        position.z = 0.00337934494019;

        orientation.x = 0.0;
        orientation.y = 0.0;
        orientation.z = 0.999477535121;
        orientation.w = -0.0323211507813;
            
	      mb_msg.pose.position = position;
        mb_msg.pose.orientation = orientation;
        mb_msg.header.seq = 0;
        mb_msg.header.stamp;
        mb_msg.header.frame_id = "map";
  	    pre = id;
  	    id = 9;
            
	      break;

        default:   
	      id = 10;
          pre = id;
	      ROS_WARN_STREAM(msg->data); 
        break;
    }
   
}


int main(int argc, char **argv){
    ros::init(argc, argv, "voice_nav_node");
    ros::NodeHandle ndHandle;

    std::string sub_nav_topic = "/voice_system/voice_cmd_vel";
    std::string pub_nav_topic = "/move_base_simple/goal";

   // ros::param::get("~sub_nav_topic",     sub_nav_topic);
  //  ros::param::get("~pub_nav_topic",     pub_nav_topic);
  //  ros::param::get("~default_speed_x",   speed_x);
  //  ros::param::get("~default_speed_y",   speed_y);
  //  ros::param::get("~default_turn_speed",turn_speed);
    ros::Rate loop_rate(5);
    while(ros::ok()){
	 ros::Subscriber sub = ndHandle.subscribe(sub_nav_topic, 1, subCallBack);
         pub = ndHandle.advertise<geometry_msgs::PoseStamped>(pub_nav_topic,1);
        if(id != pre){
            printf("%d\n",id);
            pub.publish(mb_msg);
    // commented wzh 
    /*       if(id==2){
                system("/home/sz/scripts/./leave_initial_to_one.sh");

            }else if(id==3){
                system("/home/sz/scripts/./leave32to3.sh");

            }else if(id ==4){
                system("/home/sz/scripts/./leave2todoor.sh");

            }
            else if(id == 5){
                system("/home/sz/scripts/./leave1to2.sh");

            }
	*/
            //TODO:3/3
            //COMPILE BEFORE RUNNING
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

