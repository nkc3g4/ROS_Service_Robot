/* Body Tracker Node 
   Publish data as 3 messages:
   
   Publish data messages:
   
   1. body_tracking_position_pub_ custom message:  <body_tracker_msgs::BodyTracker>
   Includes:
   2D position of person relative to head camera; allows for fast, smooth tracking
     when camera is mounted on pan/tilt servos.
     (x, y from 0.0 to 1.0, z is real)
     Astra Mini FOV: 60 horz, 49.5 vert (degrees)

   3D position of the neck joint in relation to robot (using TF)
     Joint.real: position in real world coordinates
     Useful for tracking person in 3D
   
   2. body_tracking_skeleton_pub_ custom message: <body_tracker_msgs::Skeleton>
   Includes:
   Everyting in BodyTracker message above, plus 3D position of upper body. 
     joints in relation to robot (using TF)
     Joint.real: position in real world coordinates

   3. marker_pub_  message: <visualization_msgs::Marker>
   Publishes 3d markers for selected joints.  Visible as markers in RVIZ

*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include "ros/console.h"
#include "geometry_msgs/PoseStamped.h"
#include <SFML/Graphics.hpp>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <iostream>
#include <astra_core/astra_core.hpp>
#include <astra/astra.hpp>


//For Orbbec Astra SDK
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <key_handler.h>

#include "body_tracker_msgs/BodyTracker.h"  // Publish custom message
#include "body_tracker_msgs/Skeleton.h"     // Publish custom message

#include <visualization_msgs/Marker.h>

#define KEY_JOINT_TO_TRACK    ASTRA_JOINT_SHOULDER_SPINE 
//#define M_PI                  3.1415926535897931
class sfLine : public sf::Drawable
{
public:
	sfLine(const sf::Vector2f& point1, const sf::Vector2f& point2, sf::Color color, float thickness)
		: color_(color)
	{
		const sf::Vector2f direction = point2 - point1;
		const sf::Vector2f unitDirection = direction / std::sqrt(direction.x * direction.x + direction.y * direction.y);
		const sf::Vector2f normal(-unitDirection.y, unitDirection.x);

		const sf::Vector2f offset = (thickness / 2.f) * normal;

		vertices_[0].position = point1 + offset;
		vertices_[1].position = point2 + offset;
		vertices_[2].position = point2 - offset;
		vertices_[3].position = point1 - offset;

		for (int i = 0; i < 4; ++i)
			vertices_[i].color = color;
	}

	void draw(sf::RenderTarget& target, sf::RenderStates states) const
	{
		target.draw(vertices_, 4, sf::Quads, states);
	}

private:
	sf::Vertex vertices_[4];
	sf::Color color_;
};
std::vector<sfLine> boneLines_;
std::vector<sf::CircleShape> circles_;
std::vector<sf::RectangleShape> rectangles_;
long double lastBodyTime_{ 0 };
long double lastMidSpineY_{ 0 };
long double lastRightFootY_{ 0 };
long double lastFallDownTime_{ 0 };
std::clock_t lastTimepoint_{ 0 };
int falldownCount_{ 0 };

class ColorFrameListener : public astra::FrameListener
{
public:
    ColorFrameListener()
    {
        prev_ = ClockType::now();
    }

    void init_texture(int width, int height)
    {
        if (displayBuffer_ == nullptr || width != displayWidth_ || height != displayHeight_)
        {
            displayWidth_ = width;
            displayHeight_ = height;

            // texture is RGBA
            int byteLength = displayWidth_ * displayHeight_ * 4;

            displayBuffer_ = BufferPtr(new uint8_t[byteLength]);
            std::memset(displayBuffer_.get(), 0, byteLength);

            texture_.create(displayWidth_, displayHeight_);
            sprite_.setTexture(texture_, true);
            sprite_.setPosition(0, 0);
        }
    }

    void check_fps()
    {
        const float frameWeight = .2f;

        const ClockType::time_point now = ClockType::now();
        const float elapsedMillis = std::chrono::duration_cast<DurationType>(now - prev_).count();

        elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
        prev_ = now;

        const float fps = 1000.f / elapsedMillis;

        const auto precision = std::cout.precision();

        // std::cout << std::fixed
        //           << std::setprecision(1)
        //           << fps << " fps ("
        //           << std::setprecision(1)
        //           << elapsedMillis_ << " ms)"
        //           << std::setprecision(precision)
        //           << std::endl;
    }

    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
    {
        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

        int width = colorFrame.width();
        int height = colorFrame.height();

        init_texture(width, height);

        const astra::RgbPixel* colorData = colorFrame.data();

        for (int i = 0; i < width * height; i++)
        {
            int rgbaOffset = i * 4;
            displayBuffer_[rgbaOffset] = colorData[i].r;
            displayBuffer_[rgbaOffset + 1] = colorData[i].g;
            displayBuffer_[rgbaOffset + 2] = colorData[i].b;
            displayBuffer_[rgbaOffset + 3] = 255;
        }

        texture_.update(displayBuffer_.get());
        check_fps();
    }

    void drawTo(sf::RenderWindow& window)
    {
        if (displayBuffer_ != nullptr)
        {
            float imageScale = window.getView().getSize().x / displayWidth_;
            sprite_.setScale(imageScale, imageScale);
            window.draw(sprite_);
            const float scaleX = window.getView().getSize().x / displayWidth_;
            const float scaleY = window.getView().getSize().y / displayWidth_;

            sf::RenderStates states;
            sf::Transform transform;
            transform.scale(scaleX, scaleY);
            states.transform *= transform;
        
            
            for (auto& r : rectangles_)
              window.draw(r,states);
        }

    }

private:
    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;

    ClockType::time_point prev_;
    float elapsedMillis_{.0f};

    sf::Texture texture_;
    sf::Sprite sprite_;

    using BufferPtr = std::unique_ptr<uint8_t[]>;
    BufferPtr displayBuffer_{nullptr};

    int displayWidth_{0};
    int displayHeight_{0};
};
class astra_body_tracker_node 
{
public:
  astra_body_tracker_node(std::string name) :
    _name(name)
  {
    ROS_INFO("%s: Initializing", _name.c_str());
    bool initialized = false;
    last_id_ = -1;

    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("myparm1",myparm1_,"mydefault");

    // Subscribers
    //robot_behavior_state_ = nh_.subscribe("/behavior/cmd", 1, &behavior_logic_node::behaviorStateCB, this);

    // PUBLISHERS
    // Publish tracked person in 2D and 3D
    // 2D: x,y in camera frame.   3D: x,y,z in world coordinates
    body_tracking_position_pub_ = nh_.advertise<body_tracker_msgs::BodyTracker>
      ("body_tracker/position", 1); 

    // Publish tracked person upper body skeleton for advanced uses
    body_tracking_skeleton_pub_ = nh_.advertise<body_tracker_msgs::Skeleton>
      ("body_tracker/skeleton", 1);

    // Publish markers to show where robot thinks person is in RViz
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>
      ("body_tracker/marker", 1);

    image_raw_pub_ = nh_.advertise<sensor_msgs::Image>
      ("/camera/rgb/image_raw",1);
    pub_words = nh_.advertise<std_msgs::String>("/voice_system/tts_topic",200);

/*    body_tracking_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
      ("body_tracker/pose", 1); // NOTE: We only provide to POSITION not full pose

    body_tracking_data_pub_ = nh_.advertise<body_tracker_msgs::BodyTracker>
      ("body_tracker/skeleton", 1);
*/
    ROS_INFO("astra_body_tracker: Advertised Publisher: body_tracker/pose, skeleton, marker");

  }

  ~astra_body_tracker_node()
  {
    ROS_INFO("astra_body_tracker_node shutting down");
  }



  //////////////////////////////////////////////////////////
  // Modified Orbec Astra sample code

  void output_floor(astra_bodyframe_t bodyFrame)
  {
    astra_floor_info_t floorInfo;

    astra_status_t rc = astra_bodyframe_floor_info(bodyFrame, &floorInfo);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      printf("Error %d in astra_bodyframe_floor_info()\n", rc);
      return;
    }

    const astra_bool_t floorDetected = floorInfo.floorDetected;
    const astra_plane_t* floorPlane = &floorInfo.floorPlane;
    const astra_floormask_t* floorMask = &floorInfo.floorMask;

    if (floorDetected != ASTRA_FALSE)
    {
      printf("Floor plane: [%f, %f, %f, %f]\n",
             floorPlane->a,
             floorPlane->b,
             floorPlane->c,
             floorPlane->d);

      const int32_t bottomCenterIndex = floorMask->width / 2 + floorMask->width * (floorMask->height - 1);
      printf("Floor mask: width: %d height: %d bottom center value: %d\n",
            floorMask->width,
            floorMask->height,
            floorMask->data[bottomCenterIndex]);
    }
  }

  void output_body_mask(astra_bodyframe_t bodyFrame)
  {
    astra_bodymask_t bodyMask;

    const astra_status_t rc = astra_bodyframe_bodymask(bodyFrame, &bodyMask);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      printf("Error %d in astra_bodyframe_bodymask()\n", rc);
      return;
    }

    /*
    const int32_t centerIndex = bodyMask.width / 2 + bodyMask.width * bodyMask.height / 2;
    printf("Body mask: width: %d height: %d center value: %d\n",
        bodyMask.width,
        bodyMask.height,
        bodyMask.data[centerIndex]);
    */
  }

  void output_bodyframe_info(astra_bodyframe_t bodyFrame)
  {
    astra_bodyframe_info_t info;

    const astra_status_t rc = astra_bodyframe_info(bodyFrame, &info);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      printf("Error %d in astra_bodyframe_info()\n", rc);
      return;
    }

    // width and height of floor mask, body mask, and the size of depth image
    // that joint depth position is relative to.
    const int32_t width = info.width;
    const int32_t height = info.height;

    /*
    printf("BodyFrame info: Width: %d Height: %d\n",
        width,
        height);
   */
  }



  void output_joint(std::string joint_name, const int32_t bodyId, const astra_joint_t* joint)
  {

    printf("%14s:", joint_name.c_str());

    // jointType is one of ASTRA_JOINT_* which exists for each joint type
    const astra_joint_type_t jointType = joint->type;

    // jointStatus is one of:
    // ASTRA_JOINT_STATUS_NOT_TRACKED = 0,
    // ASTRA_JOINT_STATUS_LOW_CONFIDENCE = 1,
    // ASTRA_JOINT_STATUS_TRACKED = 2,
    const astra_joint_status_t jointStatus = joint->status;

    const astra_vector3f_t* worldPos = &joint->worldPosition;

    // depthPosition is in pixels from 0 to width and 0 to height
    // where width and height are member of astra_bodyframe_info_t
    // which is obtained from astra_bodyframe_info().
    const astra_vector2f_t* depthPos = &joint->depthPosition;

    printf("Body %u Joint %d status %d @ world (%.1f, %.1f, %.1f) depth (%.1f, %.1f)\n",
           bodyId,
           jointType,
           jointStatus,
           worldPos->x,
           worldPos->y,
           worldPos->z,
           depthPos->x,
           depthPos->y);

    // orientation is a 3x3 rotation matrix where the column vectors also
    // represent the orthogonal basis vectors for the x, y, and z axes.
    /* Not sure I need orientation for anything yet, or how well this works...
    const astra_matrix3x3_t* orientation = &joint->orientation;
    const astra_vector3f_t* xAxis = &orientation->xAxis; // same as orientation->m00, m10, m20
    const astra_vector3f_t* yAxis = &orientation->yAxis; // same as orientation->m01, m11, m21
    const astra_vector3f_t* zAxis = &orientation->zAxis; // same as orientation->m02, m12, m22

    printf("Head orientation x: [%f %f %f]\n", xAxis->x, xAxis->y, xAxis->z);
    printf("Head orientation y: [%f %f %f]\n", yAxis->x, yAxis->y, yAxis->z);
    printf("Head orientation z: [%f %f %f]\n", zAxis->x, zAxis->y, zAxis->z);
    */
  }

  void output_hand_poses(const astra_body_t* body)
  {
    const astra_handpose_info_t* handPoses = &body->handPoses;

    // astra_handpose_t is one of:
    // ASTRA_HANDPOSE_UNKNOWN = 0
    // ASTRA_HANDPOSE_GRIP = 1
    const astra_handpose_t leftHandPose = handPoses->leftHand;
    const astra_handpose_t rightHandPose = handPoses->rightHand;

    /*printf("Body %d Left hand pose: %d Right hand pose: %d\n",
        body->id,
        leftHandPose,
        rightHandPose);*/
  }

  void output_bodies(astra_bodyframe_t bodyFrame)
  {
    int i;
    astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_body_list()\n", rc);
        return;
    }

    for(i = 0; i < bodyList.count; ++i)
    {
      astra_body_t* body = &bodyList.bodies[i];

      // Pixels in the body mask with the same value as bodyId are
      // from the same body.
      int bodyId = (int)body->id; // astra_body_id_t 

      // Tracking status
      // NOT_TRACKING = 0
      // TRACKING_LOST = 1
      // TRACKING_STARTED = 2
      // TRACKING = 3

      astra_body_status_t bodyStatus = body->status;
      if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED)
      {
          printf("Body Id: %d Status: Tracking started\n", bodyId);
      }
      else if (bodyStatus == ASTRA_BODY_STATUS_TRACKING)
      {
          printf("Body Id: %d Status: Tracking\n", bodyId);
      }
      else if (bodyStatus == ASTRA_BODY_STATUS_LOST)
      {
          printf("Body %u Status: Tracking lost.\n", bodyId);
      }
      else // bodyStatus == ASTRA_BODY_STATUS_NOT_TRACKING
      {
          printf("Body Id: %d Status: Not Tracking\n", bodyId);
      }

      const astra_vector3f_t* centerOfMass = &body->centerOfMass;
      const astra_body_tracking_feature_flags_t features = body->features;
      astra_joint_t* joint;  // AstraSDK/include/astra/capi/streams/body_types.h

      const bool jointTrackingEnabled = 
        (features & ASTRA_BODY_TRACKING_JOINTS) == ASTRA_BODY_TRACKING_JOINTS;
      const bool handPoseRecognitionEnabled = 
        (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

      printf("Body %d CenterOfMass (%f, %f, %f)\n",
          bodyId, centerOfMass->x, centerOfMass->y, centerOfMass->z);
      printf("    Joint Tracking Enabled: %s     Hand Pose Recognition Enabled: %s\n",
          jointTrackingEnabled       ? "True" : "False",
          handPoseRecognitionEnabled ? "True" : "False");

      ///////////////////////////////////////////////////////////////
      // Publish body tracking information, and display joint info for debug

      // Create structures for ROS Publisher data
      body_tracker_msgs::BodyTracker_ <body_tracker_msgs::BodyTracker> position_data;
      body_tracker_msgs::Skeleton_ <body_tracker_msgs::Skeleton> skeleton_data;
      //sensor_msgs::Image_ <sensor_msgs::Image> image_raw;

      // Fill in message data from AstraSDK data
      // Astra z,x,y coordinates are mapped to ROS x,y,z coordinates 
      // All values are relative to camera position in meters (ie, in Astra Camera TF frame)
      // ROS x = Astra z - distance to person
      // ROS y = Astra x - side to side
      // ROS z = Astra y - vertical height, *relative to camera position*

      // Basic Pose for person location tracking
      ///body_pose.header.frame_id = "astra_camera_link"; // "base_link";
      ///body_pose.header.stamp = ros::Time::now();

      // Skeleton Data for publilshing more detail

      position_data.body_id = bodyId;
      position_data.tracking_status = bodyStatus;
      position_data.gesture = -1; // No gesture yet

      if(bodyId != last_id_)
      {
        ROS_INFO("%s: detected person ID %d", _name.c_str(), bodyId);
        last_id_ = bodyId;
      }


      ///////////////////////////////////////////////////////////////
      // 2D position for camera servo tracking
      const float ASTRA_MINI_FOV_X = 1.047200; // (60 degrees horizontal)
      const float ASTRA_MINI_FOV_Y = -0.863938; // (49.5 degrees vertical)

      // Convert projection to radians
      // Astra proj is 0.0 (right) --> 0.628 (left)
      //           and 0.0 (top)   --> 0.628 (botom)
      // NOTE: TUNE THESE VALUSE AS NEEDED FOR YOUR CAMERA AND APPLICATION!

      joint = &body->joints[KEY_JOINT_TO_TRACK];
      float projection_x = ((astra_vector2f_t*)&joint->depthPosition)->x / 1000.0;
      float projection_y = ((astra_vector2f_t*)&joint->depthPosition)->y / 1000.0;
      position_data.position2d.x = (projection_x - 0.314) * ASTRA_MINI_FOV_X;
      position_data.position2d.y = (projection_y - 0.314) * ASTRA_MINI_FOV_Y;
      position_data.position2d.z = 0.0;

      std::cout << std::setprecision(4) << std::setw(7) 
        << "Astra: " << "2D Tracking for ID "
        << bodyId << " :  "  
        << " px: " << projection_x 
        << " py: " << projection_y
        << std::endl;

      std::cout << std::setprecision(4) << std::setw(7) 
        << " x: " << position_data.position2d.x 
        << " y: " << position_data.position2d.y
        << " z: " << position_data.position2d.z
        << std::endl;


      ///////////////////////////////////////////////////////////////
      // 3D position of person

      // *** POSITION 3D ***
      // THIS IS THE MOST RELIABLE TRACKING POINT, so we use it for person position in 3D!
      joint = &body->joints[KEY_JOINT_TO_TRACK];
      position_data.position3d.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      position_data.position3d.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      position_data.position3d.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;


      ///////////////////////////////////////////////////////////////
      // Skeleton data - published in skeleton message

      /// skeleton_data.frame_id = "astra_camera_link"; // "base_link";
      skeleton_data.body_id = bodyId;
      skeleton_data.tracking_status = bodyStatus;

      /// TODO skeleton_data.centerOfMass.x = centerOfMass->z;
      /// skeleton_data.centerOfMass.y = centerOfMass->x;
      /// skeleton_data.centerOfMass.z = centerOfMass->y;


      joint = &body->joints[ASTRA_JOINT_HEAD];
      //output_joint("Head", bodyId, joint );
      skeleton_data.joint_position_head.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_head.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_head.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_NECK];
      //output_joint("Neck", bodyId, joint );
      skeleton_data.joint_position_neck.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_neck.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_neck.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_SHOULDER_SPINE];
      //output_joint("Spine Top", bodyId, joint );
      skeleton_data.joint_position_spine_top.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_spine_top.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_spine_top.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_MID_SPINE];
      output_joint("Spine Mid", bodyId, joint );
      
      //sf::Color color(255, 255, 255, 255);
      float posx=((astra_vector3f_t*)&joint->depthPosition)->x;
      float posy=((astra_vector3f_t*)&joint->depthPosition)->y;
      //printf("POSITION: %lf,%lf\n",posx,posy);
      //sf::RectangleShape
      sf::CircleShape circle(30);
      circles_.clear();
			circle.setFillColor(sf::Color(0,0,0, 255));
			circle.setPosition( posx , posy );
      circles_.push_back(circle);
      
      std::clock_t newTimepoint = std::clock();
      long double currentBodyTime = (newTimepoint / static_cast<long double>(CLOCKS_PER_SEC));
			
      float midSpinePosX = ((astra_vector3f_t*)&joint->worldPosition)->x;
      float midSpinePosY = ((astra_vector3f_t*)&joint->worldPosition)->y;
      long double vMidSpine=0;
			if (lastBodyTime_ != 0 && currentBodyTime - lastBodyTime_ < 2) {
        printf("Last: %Lf \n",lastMidSpineY_);
				printf("MidSpine %f,%f	TimeSpan:%Lf	", midSpinePosX, midSpinePosY, currentBodyTime - lastBodyTime_);
				vMidSpine = (lastMidSpineY_-midSpinePosY) / (currentBodyTime - lastBodyTime_);
				printf("vMidSpine:%Lf pps \n", vMidSpine);
			}

			lastBodyTime_ = currentBodyTime;
			lastMidSpineY_ = midSpinePosY;

      skeleton_data.joint_position_spine_mid.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_spine_mid.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_spine_mid.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_BASE_SPINE];
      //output_joint("Spine Base", bodyId, joint );
      skeleton_data.joint_position_spine_bottom.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_spine_bottom.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_spine_bottom.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_LEFT_SHOULDER];
      //output_joint("Left Shoulder", bodyId, joint );
      float leftShoulderPosX = ((astra_vector3f_t*)&joint->depthPosition)->x;
      float leftShoulderPosY = ((astra_vector3f_t*)&joint->depthPosition)->y;
      skeleton_data.joint_position_left_shoulder.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_left_shoulder.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_left_shoulder.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_LEFT_ELBOW];
      //output_joint("Left Elbow", bodyId, joint );
      skeleton_data.joint_position_left_elbow.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_left_elbow.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_left_elbow.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_LEFT_HAND];
      //output_joint("Left Hand", bodyId, joint );
      skeleton_data.joint_position_left_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_left_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_left_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_RIGHT_SHOULDER];
      //output_joint("Right Shoulder", bodyId, joint );
      float rightShoulderPosX = ((astra_vector3f_t*)&joint->depthPosition)->x;
      float rightShoulderPosY = ((astra_vector3f_t*)&joint->depthPosition)->y;

      skeleton_data.joint_position_right_shoulder.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_right_shoulder.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_right_shoulder.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_RIGHT_ELBOW];
     //output_joint("Right Elbow", bodyId, joint );
      skeleton_data.joint_position_right_elbow.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_right_elbow.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_right_elbow.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_RIGHT_HAND];
      //output_joint("Right Hand", bodyId, joint );
      skeleton_data.joint_position_right_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_right_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_right_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_RIGHT_FOOT];
      float rightFootPosX = ((astra_vector3f_t*)&joint->worldPosition)->x;
      float rightFootPosY = ((astra_vector3f_t*)&joint->worldPosition)->y;
      //lastRightFootY_ = rightFootPosY;
      float baseH = rightFootPosY- lastMidSpineY_;
      printf("rightFootPosY,lastMidSpineY_:%Lf,%Lf\n",rightFootPosY,lastMidSpineY_);
      printf("baseH:%Lf\n",  baseH);
      rectangles_.clear();
      sf::RectangleShape rectangle;
      rectangle.setSize(sf::Vector2f(60,60));
      rectangle.setOutlineColor(sf::Color::Red);
      rectangle.setFillColor(sf::Color::Transparent);
      rectangle.setOutlineThickness(5);
      rectangle.setPosition(posx-30, posy-30);
      rectangles_.push_back(rectangle);
      
      std_msgs::String words;
      words.data = "跌倒了";
      newTimepoint = std::clock();
      long double currentTime = (newTimepoint / static_cast<long double>(CLOCKS_PER_SEC));
      printf("TimeDiff: %Ld\n",currentTime-lastFallDownTime_);
      if(vMidSpine>1000&&baseH<-150){
        falldownCount_++;
        printf("FALL DOWN DETECTED!!!!\n");

        if(falldownCount_>5){
          falldownCount_=0;
          printf("FALL DOWN DETECTED!!!!\n");
          printf("FALL DOWN DETECTED!!!!\n");
          printf("FALL DOWN DETECTED!!!!\n");
          printf("FALL DOWN DETECTED!!!!\n");
          printf("FALL DOWN DETECTED!!!!\n");
          //Fall down
          pub_words.publish(words);      
          newTimepoint = std::clock();
          lastFallDownTime_=currentTime;
        }
      }
      if(currentTime-lastFallDownTime_>100000){
        falldownCount_=0;
      }
      //

      // Is hand open (0) or grasping (1)?
      //output_hand_poses(body);
      const astra_handpose_info_t* handPoses = &body->handPoses;
      // We just publish "1" if either hand has any gesture detected
      skeleton_data.gesture = (handPoses->rightHand + handPoses->leftHand);

      ////////////////////////////////////////////////////
      // Publish everything
      body_tracking_position_pub_.publish(position_data); // position data
      body_tracking_skeleton_pub_.publish(skeleton_data); // full skeleton data


      PublishMarker(
        2, // ID
        skeleton_data.centerOfMass.x, // Distance to person = ROS X
        skeleton_data.centerOfMass.y, // side to side = ROS Y
        skeleton_data.centerOfMass.z, // Height = ROS Z
        1.0, 0.0, 1.0 ); // r,g,b

      PublishMarker(
        3, // ID
        skeleton_data.joint_position_head.x,
        skeleton_data.joint_position_head.y,
        skeleton_data.joint_position_head.z,
        0.7, 0.7, 0.7 ); // r,g,b

      PublishMarker(
        4, // ID
        skeleton_data.joint_position_spine_top.x,
        skeleton_data.joint_position_spine_top.y,
        skeleton_data.joint_position_spine_top.z,
        0.0, 0.0, 1.0 ); // r,g,b

      PublishMarker(
        5, // ID
        skeleton_data.joint_position_spine_mid.x,
        skeleton_data.joint_position_spine_mid.y,
        skeleton_data.joint_position_spine_mid.z,
        0.0, 1.0, 0.0 ); // r,g,b

      PublishMarker(
        6, // ID
        skeleton_data.joint_position_spine_bottom.x,
        skeleton_data.joint_position_spine_bottom.y,
        skeleton_data.joint_position_spine_bottom.z,
        1.0, 0.0, 0.0 ); // r,g,b

      printf("SPINE TOP: x=%3.0f, y=%3.0f, z=%3.0f inches\n",
        skeleton_data.joint_position_spine_top.x * 39.3,
        skeleton_data.joint_position_spine_top.y * 39.3,
        skeleton_data.joint_position_spine_top.z * 39.3);

      printf("----------------------------\n\n");

    } // for each body seen

  }

  void PublishMarker(int id, float x, float y, float z, float color_r, float color_g, float color_b)
  {
    // Display marker for RVIZ to show where robot thinks person is
    // For Markers info, see http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

    // ROS_INFO("DBG: PublishMarker called");
    //if( id != 1)
    //  printf ("DBG PublishMarker called for %f, %f, %f\n", x,y,z);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "astra_camera_link"; // "base_link";
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(1.0); // seconds

    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "astra_body_tracker";
    marker.id = id; // This must be id unique for each marker

    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = color_r;
    marker.color.g = color_g; 
    marker.color.b = color_b;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1; // size of marker in meters
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;  

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;


    //ROS_INFO("DBG: Publishing Marker");
    marker_pub_.publish(marker);

  }

  void output_bodyframe(astra_bodyframe_t bodyFrame)
  {
    output_floor(bodyFrame);

    output_body_mask(bodyFrame);

    output_bodyframe_info(bodyFrame);

    output_bodies(bodyFrame);
  }

  void runLoop()
  {
    set_key_handler();
    astra_initialize();
    const char* licenseString = "<INSERT LICENSE KEY HERE>";
    orbbec_body_tracking_set_license(licenseString);

    astra_streamsetconnection_t sensor;
    astra_streamset_open("device/default", &sensor);

    astra_reader_t reader;
    astra_reader_create(sensor, &reader);

    astra_bodystream_t bodyStream;
    astra_reader_get_bodystream(reader, &bodyStream);

    astra_stream_start(bodyStream);


    sf::RenderWindow window(sf::VideoMode(640, 480), "Color Viewer");

    astra::StreamSet streamSet;
    astra::StreamReader streamreader = streamSet.create_reader();

    streamreader.stream<astra::ColorStream>().start();

    ColorFrameListener listener;
    streamreader.add_listener(listener);
while (window.isOpen())
    {
        astra_update();

        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;
            case sf::Event::KeyPressed:
                {
                    if (event.key.code == sf::Keyboard::Escape ||
                        (event.key.code == sf::Keyboard::C && event.key.control))
                    {
                        window.close();
                    }
                }
            default:
                break;
            }
        }

        // clear the window with black color
        window.clear(sf::Color::Black);

        listener.drawTo(window);
        window.display();
         astra_update();

      astra_reader_frame_t frame;
      astra_reader_frame_t imgFrame;
      astra_status_t rc = astra_reader_open_frame(reader, 0, &frame);

      if (rc == ASTRA_STATUS_SUCCESS)
      {
        astra_colorframe_t imageFrame;
        astra_frame_get_colorframe(imgFrame,&imageFrame);
        //image_raw_pub_.publish(*imageFrame);
        astra_bodyframe_t bodyFrame;
        astra_frame_get_bodyframe(frame, &bodyFrame);

        astra_frame_index_t frameIndex;
        astra_bodyframe_get_frameindex(bodyFrame, &frameIndex);
        // printf("Frame index: %d\n", frameIndex);

        output_bodyframe(bodyFrame);

        //printf("----------------------------\n");

        astra_reader_close_frame(&frame);
      }

      ros::spinOnce();  // ROS

        if (!shouldContinue)
        {
            window.close();
        }
    }
    

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);

    astra_terminate();

  }


private:
  /////////////// DATA MEMBERS /////////////////////

  std::string _name;
  ros::NodeHandle nh_;
  std::string myparm1_;
  int last_id_;


  ros::Publisher body_tracking_position_pub_;
  ros::Publisher body_tracking_skeleton_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher image_raw_pub_;
  ros::Publisher pub_words;

};


// The main entry point for this node.
int main( int argc, char *argv[] )
{
  
  ros::init( argc, argv, "astra_body_tracker" );
  astra_body_tracker_node node(ros::this_node::getName());
  node.runLoop();
  //ros::spin();

  return 0;
}


