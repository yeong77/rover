/**
 * @file PX4 drone control and tracking person
 * @author Daehyen Kwon (dh0708@kumoh.ac.kr) WENS Lab
 * @date 2022.06.10
 * @version 1.1.3 
 */


#pragma once

#include <signal.h>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>


#include <Keyboard.h>
#include <Convert.h>

#define POSITION_MODE 1  // Move by setpoint_position
#define VELOCITY_MODE 0  // Move by setpoint_velocity

#define TRACKING_ON  1
#define TRACKING_OFF 0
#define TRACKING_DEACTIVATE -1


// Command information of keyboard input
struct CMD_INFO
{
    double time_in;     // Time when the command was entered
    double time_start;  // Time when the command was executed
    char command;
    short try_max;
    short try_num;
    bool success;
};

struct DRONE_DATA
{
    double timeout_in = 5.0;    // Timeout about time_in
    double timeout_start = 3.0; // Timeout about time_start
    double timeout_connection = 3.0;
    double timeout_detection = 1.5;
    double timeout_communication = 3.0;

    bool mode_control = false;
    bool mode_tracking = false;
    bool mode_planner = false;

    double current_yaw; // radian
    double default_height = 3.0;

    mavros_msgs::State current_state;
    ros::Time current_time;
    ros::Time prev_time;
};

struct DRONE_REQUEST
{
    bool arm;
    bool mode_control;
    bool mode_tracking;
    double velocity[3];
    std_msgs::String mode;
};

struct TRACKING_DATA
{
    bool check_data;
    double time_in;
    double time_start;
    geometry_msgs::TwistStamped msg;
};

struct TRACKING_VALUE_TIMEOUT
{
    bool trigger_stop;
    double time_in;
    double time_zero_veloity;
};

struct TRACKING_DETECTION_TIMEOUT
{
    bool trigger_hover;     
    double time_last_in;
};

class Drone{
    private:
        ros::NodeHandle nh;
    
        ros::Subscriber sub_state;
        ros::Subscriber sub_pose;
        ros::Subscriber sub_tracking;

        // Fast Planner & Octomap
        ros::Subscriber sub_planner;

        ros::Subscriber sub_request;
        ros::Subscriber sub_detection;

        ros::Publisher pub_setLocalVelocity;
        ros::Publisher pub_setLocalPose;

        // Fast Planner
        ros::Publisher pub_setLocalRaw;

        ros::Publisher pub_request;
        ros::Publisher pub_respond;


        ros::ServiceClient client_arming;
        ros::ServiceClient client_setMode;

        std::string param_tracking = "drone/mode/tracking";
        std::string param_control = "drone/mode/control";
        std::string param_activate_tracking = "drone/activate/tracking";
        
        int activate_tracking = TRACKING_DEACTIVATE;

        // Fast Planner ================================
        bool fastplanner = false;
        mavros_msgs::PositionTarget set_localRaw;
        

        geometry_msgs::PoseStamped set_localPose;
        geometry_msgs::PoseStamped current_pose;

        geometry_msgs::TwistStamped set_localVelocity;

        //new ===================================
        std_msgs::UInt8 set_mode;


        CMD_INFO cmd_info = {};
        DRONE_DATA drone_data = {};
        DRONE_REQUEST drone_request = {};
        TRACKING_DATA tracking_data = {};
        TRACKING_VALUE_TIMEOUT tracking_value_timeout = {};
        TRACKING_DETECTION_TIMEOUT tracking_detection_timeout = {};

        ros::Rate rate = ros::Rate(10.0);
       
        bool ReadParam();
        void SetInit();
        void Shutdown();

        bool CheckTimeout(double time_input, double timeout);
        void CheckCommand(char input);
        bool CheckCommandSuccess();
        bool CheckConnection(double timeout);
        bool CheckArm(bool requested_arm);
        bool CheckFlying();
        bool CheckMode(std_msgs::String mode);
        bool CheckControlMode(bool requested_mode);
        bool CheckTracking(bool requested_mode);

        void PrintCMDSuccess(CMD_INFO &data);
        void PrintTrackingSucccess();
        void InitCMDData(CMD_INFO &data);
        void DoCommand();
        void DoTracking();
        //new ===============
        void PrintSuccess();

        void SetCurrentPose();
        void SetCurrentYaw();
        void SetDefaultHeight();


        void Arming();
        void ChangeMode(std_msgs::String mode);

        void MovePosition(double requested_poseX, double requested_poseY, double requested_poseZ);
        void MoveVelocity(double requetsed_velocityX, double requetsed_velocityY, double requetsed_velocityZ);

        void MovePositionAngle(double requested_angle);
        void MoveVelocityAngle(double requested_angleVelocity);

        void Hovering();

        void Change_ControlMode(bool requested_move);
        void Change_TrackingMode(bool requested_mode);

        bool CheckTrackingValue();

        void Land();

        void Callback_state(const mavros_msgs::State::ConstPtr& state_msg);
        void Callback_localPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
        void Callback_tracking(const geometry_msgs::TwistStamped::ConstPtr& tracking_msg);
        void Callback_request(const std_msgs::String::ConstPtr& request_msg);
        void callback_fastplanner(const mavros_msgs::PositionTarget::ConstPtr& planner_msg);
        void callback_fastplannerOcto(const nav_msgs::Path::ConstPtr& planner_msg);
        void callback_detection(const std_msgs::UInt8::ConstPtr& data);
        

    public:
        explicit Drone(const ros::NodeHandle& _nodeHandle);
        void Control(); // Main control code

};