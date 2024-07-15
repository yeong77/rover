#include <drone.h>


// Read launchfile parameter
bool Drone::ReadParam()
{
    int error_count = 0;

    if(!nh.getParam("timeout_in", drone_data.timeout_in)){
        ROS_WARN("[FAIL] Get [timeout_in : %.1lf]", drone_data.timeout_in);
        error_count++;
    }
        
    if(!nh.getParam("timeout_start", drone_data.timeout_start)){
        ROS_WARN("[FAIL] Get [timeout_start : %.1lf]", drone_data.timeout_start);
        error_count++;
    }
    
    if(!nh.getParam("timeout_connection", drone_data.timeout_connection)){
        ROS_WARN("[FAIL] Get [timeout_connection : %.1lf]", drone_data.timeout_connection);
        error_count++;
    }

    if(!nh.getParam("default_height", drone_data.default_height)){
        ROS_WARN("[FAIL] Get [default_height : %.2lf]", drone_data.default_height);
        error_count++;
    }

    if(!nh.getParam("activate_tracking", activate_tracking)){
        ROS_WARN("[FAIL] Get [activate_tracking : %d]", activate_tracking);
        error_count++;
    }

    if (error_count == 0)
    {
        ROS_INFO("[PARAM] Get [timeout_in : %.1lf]", drone_data.timeout_in);
        ROS_INFO("[PARAM] Get [timeout_start : %.1lf]", drone_data.timeout_start);
        ROS_INFO("[PARAM] Get [timeout_connection : %.1lf]", drone_data.timeout_connection);
        ROS_INFO("[PARAM] Get [default_height : %.2lf]", drone_data.default_height);
        ROS_INFO("[PARAM] Get [activate_tracking : %d]", activate_tracking);
    }

    return error_count;
}


// Check px4 connection
bool Drone::CheckConnection(double timeout)
{
    double time_start = 0;

    ROS_INFO("[TRY] Connect....");

    while(ros::ok())
    {
        if (time_start == 0)
            time_start = ros::Time::now().toSec();

        drone_data.current_time = ros::Time::now();

        if (drone_data.current_state.connected)
        {
            ROS_INFO("[SUCCESS] Connected");
            return true;
        }

        if (CheckTimeout(time_start, timeout)){
            ROS_ERROR("[TIMEOUT] Connection");
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_ERROR("[FAIL] Connect");
    return false;
}



// Check Arm
bool Drone::CheckArm(bool requested_arm)
{
    if (drone_data.current_state.armed == requested_arm)
        return true;
    return false;
}

// Check if Drone is flying
bool Drone::CheckFlying()
{
    // system_status 4 = MAV_STATE_ACTIVE
    if (drone_data.current_state.armed == true && drone_data.current_state.system_status == 4){
        return true;
    }

    return false;
}

// Check if Mode is same with parameter
bool Drone::CheckMode(std_msgs::String mode)
{
    if (drone_data.current_state.mode == mode.data)
        return true;

    return false;
}

void Drone::Arming()
{
    if (CheckArm(true))
    {
        ROS_ERROR("Already Armed");
        return;
    }

    drone_request.arm = true;

    mavros_msgs::CommandBool cmd_arm;
    cmd_arm.request.value = drone_request.arm;

    client_arming.call(cmd_arm);

    if (cmd_arm.response.success)
    {
        ROS_INFO("[Try] Send Arming Command [R:%d  S:%d]", cmd_arm.response.result, cmd_arm.response.success);
    }
    else
    {
        ROS_WARN("[FAIL] Send Arming Command [R:%d  S:%d]", cmd_arm.response.result, cmd_arm.response.success);
    }
}

// Change PX4 mode as parameter
void Drone::ChangeMode(std_msgs::String mode)
{
    if (CheckMode(mode))
    {
        ROS_WARN("[FAIL] Already %s mode", drone_data.current_state.mode.c_str());
        return;
    }

    ROS_INFO("[TRY] Change Mode to [%s]", mode.data.c_str());

    mavros_msgs::SetMode cmd_mode;
    cmd_mode.request.base_mode = 0;
    cmd_mode.request.custom_mode = mode.data;

    client_setMode.call(cmd_mode);
}

// Set local position as current position
void Drone::SetCurrentPose()
{
    set_localPose.pose.position.x = current_pose.pose.position.x;
    set_localPose.pose.position.y = current_pose.pose.position.y;
    set_localPose.pose.position.z = current_pose.pose.position.z;
}

// Set Local yaw as current yaw
void Drone::SetCurrentYaw()
{
    set_localPose.pose.orientation.w = current_pose.pose.orientation.w;
    set_localPose.pose.orientation.x = current_pose.pose.orientation.x;
    set_localPose.pose.orientation.y = current_pose.pose.orientation.y;
    set_localPose.pose.orientation.z = current_pose.pose.orientation.z;

    double *current_RPY = quaternion_to_euler(set_localPose.pose.orientation.x, set_localPose.pose.orientation.y,
                                              set_localPose.pose.orientation.z, set_localPose.pose.orientation.w);

    if (current_RPY[2] < 0)
        current_RPY[2] = current_RPY[2] + M_PI * 2;
    else if (current_RPY[2] > M_PI * 2)
        current_RPY[2] = current_RPY[2] - M_PI * 2;

    drone_data.current_yaw = current_RPY[2];

    delete[] current_RPY;
}

// Move Drone by setpoint_position
void Drone::MovePosition(double requested_poseX, double requested_poseY, double requested_poseZ)
{
    // Stop
    if (requested_poseX == 0 && requested_poseY == 0 && requested_poseZ == 0)
    {
        SetCurrentPose();
    }

    else
    {
        double goal_position[3] = {0, 0, requested_poseZ};

        if (requested_poseX > 0)
        {
            goal_position[0] = requested_poseX * cos(drone_data.current_yaw);
            goal_position[1] = requested_poseX * sin(drone_data.current_yaw);
        }
        else if (requested_poseX < 0)
        {
            goal_position[0] = abs(requested_poseX) * -cos(drone_data.current_yaw);
            goal_position[1] = abs(requested_poseX) * -sin(drone_data.current_yaw);
        }

        if (requested_poseY > 0)
        {
            goal_position[0] = requested_poseY * sin(drone_data.current_yaw);  //-cos(current_yaw + M_PI_2);
            goal_position[1] = requested_poseY * -cos(drone_data.current_yaw); //-sin(current_yaw + M_PI_2);
        }
        else if (requested_poseY < 0)
        {
            goal_position[0] = abs(requested_poseY) * -sin(drone_data.current_yaw); // cos(current_yaw + M_PI_2);
            goal_position[1] = abs(requested_poseY) * cos(drone_data.current_yaw);  // sin(current_yaw + M_PI_2);
        }

        set_localPose.pose.position.x += goal_position[0];
        set_localPose.pose.position.y += goal_position[1];
        set_localPose.pose.position.z += goal_position[2];

    }
}

/**
 * @brief Turn Drone in position mode
 * 
 * @param requested_angle radian
 */
void Drone::MovePositionAngle(double requested_angle)
{
    if (requested_angle == 0)
    {
        SetCurrentYaw();
    }
    else
    {
        double goal_radian = drone_data.current_yaw + degree_to_radian(requested_angle);

        if (goal_radian >= M_PI * 2)
            goal_radian -= M_PI * 2;
        else if (goal_radian <= 0)
            goal_radian += M_PI * 2;

        drone_data.current_yaw = goal_radian;

        double *q;
        q = euler_to_quaternion(0, 0, drone_data.current_yaw);

        set_localPose.pose.orientation.x = q[0];
        set_localPose.pose.orientation.y = q[1];
        set_localPose.pose.orientation.z = q[2];
        set_localPose.pose.orientation.w = q[3];

        delete[] q;
    }
}

// Move drone by setpoint_velocity
void Drone::MoveVelocity(double requested_velocityX, double requested_velocityY, double requested_velocityZ)
{
    drone_request.velocity[0] = requested_velocityX;
    drone_request.velocity[1] = requested_velocityY;
    drone_request.velocity[2] = requested_velocityZ;

    if (requested_velocityX == 0 && requested_velocityY == 0 && requested_velocityZ == 0){
        set_localVelocity.twist.linear = {};
        SetCurrentYaw();
    }

    else{
        double goal_speed[3] = {0, 0, requested_velocityZ};

        if (requested_velocityX > 0)
        {
            goal_speed[0] = requested_velocityX * cos(drone_data.current_yaw);
            goal_speed[1] = requested_velocityX * sin(drone_data.current_yaw);
        }
        else if (requested_velocityX < 0)
        {
            goal_speed[0] = abs(requested_velocityX) * -cos(drone_data.current_yaw);
            goal_speed[1] = abs(requested_velocityX) * -sin(drone_data.current_yaw);
        }

        if (requested_velocityY > 0)
        {
            goal_speed[0] = requested_velocityY * sin(drone_data.current_yaw);  //-cos(current_yaw + M_PI_2);
            goal_speed[1] = requested_velocityY * -cos(drone_data.current_yaw); //-sin(current_yaw + M_PI_2);
        }
        else if (requested_velocityY < 0)
        {
            goal_speed[0] = abs(requested_velocityY) * -sin(drone_data.current_yaw); // cos(current_yaw + M_PI_2);
            goal_speed[1] = abs(requested_velocityY) * cos(drone_data.current_yaw);  // sin(current_yaw + M_PI_2);
        }

        set_localVelocity.twist.linear.x = goal_speed[0];
        set_localVelocity.twist.linear.y = goal_speed[1];
        set_localVelocity.twist.linear.z = goal_speed[2];
    }
}

// Trun drone by setpoint_velocity
void Drone::MoveVelocityAngle(double requested_angleVelocity)
{
    set_localVelocity.twist.angular.z = requested_angleVelocity;
    if (requested_angleVelocity == 0)
        SetCurrentYaw();
}

void Drone::Land()
{
    if (!CheckFlying())
    {
        ROS_WARN("[FAIL] Not Flying [Arm:%d , Status:%d]", drone_data.current_state.armed, drone_data.current_state.system_status);
        return;
    }

    Change_TrackingMode(false);

    drone_request.mode.data = "AUTO.LAND";
    ChangeMode(drone_request.mode); // Change mode as AUTO.LAND
}

void Drone::Hovering()
{
    if(fastplanner)
    {
        pub_setLocalRaw.publish(set_localRaw);
        return;
    }

    if(drone_data.mode_control == POSITION_MODE) 
        pub_setLocalPose.publish(set_localPose);
    else{
        SetCurrentYaw();
        pub_setLocalVelocity.publish(set_localVelocity);
    }
}

void Drone::SetDefaultHeight()
{
    set_localPose.pose.position.z = drone_data.default_height;
}

void Drone::Shutdown()
{
    std_msgs::String msg_shutdown;
    msg_shutdown.data = "q";
    pub_request.publish(msg_shutdown); // Transfer shutdown cmd to other node 

    ROS_INFO("[EXIT] Drone node");
    close_keyboard();
    ros::shutdown();
}


bool Drone::CheckControlMode(bool requested_mode)
{
    if (drone_data.mode_control == requested_mode)
    {
        nh.setParam(param_control, drone_data.mode_control);
        return true;
    }
    
    return false;
}

bool Drone::CheckTracking(bool requested_mode)
{
    if (drone_data.mode_tracking == requested_mode)
    {
        nh.setParam(param_tracking, drone_data.mode_tracking);
        return true;
    }
    
    return false;
}



void Drone::Change_ControlMode(bool requested_move)
{
    if (requested_move == POSITION_MODE)
    {
        SetCurrentPose();
        SetCurrentYaw();
    }
    else{
        set_localVelocity = {};
    }

    drone_request.mode_control = requested_move;
    drone_data.mode_control = requested_move;
}


void Drone::Change_TrackingMode(bool requested_mode)
{
    if (requested_mode == TRACKING_ON)
    {
        Change_ControlMode(VELOCITY_MODE);
    }
    else
    {
        Change_ControlMode(POSITION_MODE);
    }

    drone_request.mode_tracking = requested_mode;
    drone_data.mode_tracking = requested_mode;

    nh.setParam(param_tracking, requested_mode);
}