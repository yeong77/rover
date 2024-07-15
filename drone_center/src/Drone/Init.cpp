#include <drone.h>


Drone::Drone(const ros::NodeHandle& _nodeHandle):
    nh(_nodeHandle),

    sub_state(nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Drone::Callback_state, this)),
    sub_pose(nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Drone::Callback_localPose, this)),
    
    sub_tracking(nh.subscribe<geometry_msgs::TwistStamped>("drone/cmd/tracking", 1, &Drone::Callback_tracking, this)),
    sub_planner(nh.subscribe<mavros_msgs::PositionTarget>("fastplanner/waypoint", 1, &Drone::callback_fastplanner, this)),

    sub_request(nh.subscribe<std_msgs::String>("drone/cmd/request", 1, &Drone::Callback_request, this)),
    sub_detection(nh.subscribe<std_msgs::UInt8>("detect", 1, &Drone::callback_detection, this)),


    pub_setLocalVelocity(nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10)),
    pub_setLocalPose(nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10)),
    pub_setLocalRaw(nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1)),

    pub_request(nh.advertise<std_msgs::String>("drone/cmd/request", 10)),
    pub_respond(nh.advertise<std_msgs::Header>("drone/cmd/respond", 10)),
    
    client_arming(nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming")),
    client_setMode(nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode"))

{
    
    set_localPose = {};
    set_localVelocity = {};
    SetDefaultHeight();

    ReadParam();

    nh.setParam(param_activate_tracking, activate_tracking);

    drone_data.mode_control = POSITION_MODE;
    nh.setParam(param_control, POSITION_MODE); //POSITION_MODE

    drone_data.mode_tracking = false;
    nh.setParam(param_tracking, false);

 
    if (!CheckConnection(drone_data.timeout_connection))
    {
        Shutdown();
        return; 
    }

    init_keyboard();
}
