/**
 * Keyboard setting
 * wasdx : move (s:stop)
 * u/j   : up/down
 * g     : LAND (ground) 
 * o/p   : Offboard mode / Arming (prepare)
 * m     : Change move mode (POSITION / VELOCITY)
 * t     : Tracking mode On/Off
 * 123   : Trun (2:stop)
 */

#include <drone.h>


void Drone::Control()
{
    bool cmd_trigger = false;
    ROS_INFO("Control Start");
    cmd_info = {};

    while (ros::ok())
    {
        // buffer check
        if (_kbhit())
        {   
            // If there is no working command, store input command
            if (cmd_info.command == '\0'){
                char ch = _getch();
                
                if (ch == 'q' || ch == 'Q')
                {
                    Shutdown();
                    return;
                }
                
                CheckCommand(ch);
            }
        }

        // If Command is entered, then Execute command only once and Check its success until timeout.
        // Command sucess -> initialize data , Comand fail -> Alert & initialize data
        // After initializing data , getting another message is possible
        if (cmd_info.command != 0){
            if(CheckTimeout(cmd_info.time_in, drone_data.timeout_in)){
                ROS_WARN("[TIMEOUT] command input");
                InitCMDData(cmd_info);
            }
            else if (cmd_info.time_start != 0 && CheckTimeout(cmd_info.time_start, drone_data.timeout_start)){
                ROS_WARN("[TIMEOUT] command start");
                InitCMDData(cmd_info);
            }
            else{
                if (cmd_info.time_start == 0)
                {
                    // Execute command only one time
                    DoCommand();
                }
                else{
                    // Check command success until timeout
                    if (CheckCommandSuccess()){
                        if (cmd_info.command == 'm')
                            nh.setParam(param_control, drone_data.mode_control);
                        if (cmd_info.command == 't')
                            nh.setParam(param_tracking, drone_data.mode_tracking);
                        
                        PrintCMDSuccess(cmd_info);
                        DoCommand();
                        InitCMDData(cmd_info);
                    }
                }
            }
        }


        // If tracking data is entered, then Execute tracking command only once and initialize data.
        // After initializing data, getting another message is possible.
        if (tracking_data.check_data == true)
        {
            if(CheckTimeout(tracking_data.time_in, drone_data.timeout_in))
            {
                ROS_WARN("[TIMEOUT] tracking input");
                tracking_data = {};
            }
            else
            {
                if (tracking_data.time_start == 0)
                {
                    // If tracking message have zero value of all data, Change move_mode to POSITION_MODE
                    // And there is any message having no-zero value of data, object tracking work again
                    if(CheckTrackingValue())
                        DoTracking();
                }
                else{
                    if (!tracking_value_timeout.trigger_stop){
                        ROS_INFO("[SUCCESS] Tracking [N:%.2lf,%.2lf,%.2lf][A:%.2lf]", set_localVelocity.twist.linear.x, 
                            set_localVelocity.twist.linear.y, set_localVelocity.twist.linear.z, set_localVelocity.twist.angular.z);
                    }
                    tracking_data = {};
                }
            }
        }
        else
        {
            // while tracking mode is working, If there is no tracking msg for a certain period of time,
            // Make hovering by changing mode to POSITION
            // If tracking msg is entered, Change mode to VELOCITY and Resume tracking
            if (drone_data.mode_tracking && activate_tracking != TRACKING_DEACTIVATE)
            {
                if(!tracking_detection_timeout.trigger_hover)
                {
                    if(CheckTimeout(tracking_detection_timeout.time_last_in, drone_data.timeout_detection))
                    {
                        tracking_detection_timeout.trigger_hover = true;
                        ROS_WARN("[TIMEOUT] Detection time, [%lf, %lf]", tracking_detection_timeout.time_last_in, drone_data.timeout_detection);
                        Change_ControlMode(POSITION_MODE);
                        set_localPose.pose.position.z = drone_data.default_height;
                    }
                }
            }
        }

        // To implement curve movements in velocity mode, 
        // If drone is moving and yaw velocity is not zero, Update current yaw
        if (set_localVelocity.twist.linear.x != 0 || set_localVelocity.twist.linear.y != 0)
        {   
            if (drone_data.mode_tracking == false)
            {
                SetCurrentYaw();
                MoveVelocity(drone_request.velocity[0], drone_request.velocity[1], drone_request.velocity[2]);
            }
        }

        // Publish setpoint message while code is working
        Hovering();

        ros::spinOnce();
        rate.sleep();
    }
}

// Print message about successfully working
void Drone::PrintCMDSuccess(CMD_INFO &data){
    switch (data.command)
    {
        case 'o':
            ROS_INFO("[SUCCESS] Mode Changed [M:%s]", drone_data.current_state.mode.c_str());
            break;
        case 'p':
            ROS_INFO("[SUCCESS] Arm Changed [A:%d]", drone_data.current_state.armed);
            break;
        case 'g':
            ROS_INFO("[SUCCESS] Landing... [M:%s]", drone_data.current_state.mode.c_str());
            break;
        case 'w':
        case 'x':
        case 'a':
        case 'd':
        case 'u':
        case 'j':
            if (drone_data.mode_control == POSITION_MODE)
                ROS_INFO("[SUCCESS] Move Position [N:%.1f,%.1f,%.1f][G:%.1f,%.1f,%.1f]",  
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    set_localPose.pose.position.x, set_localPose.pose.position.y, set_localPose.pose.position.z);
            else 
                ROS_INFO("[SUCCESS] Move Velocity [N:%.1f,%.1f,%.1f]", 
                    set_localVelocity.twist.linear.x, set_localVelocity.twist.linear.y, set_localVelocity.twist.linear.z);
            break;
        case 's': // Stop Moving
            if (drone_data.mode_control == POSITION_MODE)
                ROS_INFO("[SUCCESS] Stop Move Position [N:%.1f,%.1f,%.1f][G:%.1f,%.1f,%.1f]",  
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    set_localPose.pose.position.x, set_localPose.pose.position.y, set_localPose.pose.position.z);
            else
                ROS_INFO("[SUCCESS] Stop Move Velocity [N:%.1f,%.1f,%.1f]",
                    set_localVelocity.twist.linear.x, set_localVelocity.twist.linear.y, set_localVelocity.twist.linear.z);
            break;
        case '1':
        case '3':
            if (drone_data.mode_control == POSITION_MODE)
                ROS_INFO("[SUCCESS] Trun Position [A:%.2lf]",radian_to_degree(drone_data.current_yaw));
            else
                ROS_INFO("[SUCCESS] Trun Velocity");
            break;
        case '2':
            ROS_INFO("[SUCCESS] Trun Stop [A:%.2lf]", radian_to_degree(drone_data.current_yaw));
            break;

        case 'm':
            ROS_INFO("[SUCCESS] Move mode changed [%s] ", drone_data.mode_control ? "POSITION" : "VELOCITY");
            break;
        
        case 't':
            ROS_INFO("[SUCCESS] Tracking mode changed [%s][%s]", drone_data.mode_tracking ? "On" : "Off", 
                drone_data.mode_control ? "POSITION" : "VELOCITY");
            break;
    }
}

// Initialize Data & Buffer
void Drone::InitCMDData(CMD_INFO &data){
    data = {};              // init data
    tcflush(0, TCIFLUSH);   // init buffer
}

// Check entered command
void Drone::CheckCommand(char input){
    switch (input){
        case 'o':
        case 'p':
        case 'g':
        case 'w':
        case 'x':
        case 'a':
        case 's':
        case 'd':
        case 'u':
        case 'j':
        case '1':
        case '2':
        case '3':
        case 'm':
        case 't':
        case 'f':
            cmd_info.command = input;   //store command key
            cmd_info.time_in = ros::Time::now().toSec();
            break;
    }
}


bool Drone::CheckTimeout(double time_input, double timeout){

    drone_data.current_time = ros::Time::now();

    if (drone_data.current_time.toSec() - time_input >= timeout){
        return true;
    }

    return false;
}


bool Drone::CheckCommandSuccess()
{
    switch (cmd_info.command)
    {
    case 'p':
        cmd_info.success = CheckArm(drone_request.arm);
        break;
    case 'o':
        cmd_info.success = CheckMode(drone_request.mode);
        break;
    case 'g':
        cmd_info.success = CheckMode(drone_request.mode);
        break;
    case 'm':
        cmd_info.success = CheckControlMode(drone_request.mode_control);
        break;
    case 't':
        cmd_info.success = CheckTracking(drone_request.mode_tracking);
        break;
    case 'w':
    case 'a':
    case 's':
    case 'd':
    case 'x':
    case 'u':
    case 'j':
    case '1':
    case '2':
    case '3':
        cmd_info.success = true;
        break;
    }

    return cmd_info.success;
}

// Execute keyboard input command
void Drone::DoCommand()
{
    switch (cmd_info.command)
    {
    case 'o':
        drone_request.mode.data = "OFFBOARD";
        ChangeMode(drone_request.mode);
        break;
    case 'p':
        Arming();
        break;
    case 'g':
        Land();
        break;
    case 'w':
        if (drone_data.mode_control == POSITION_MODE) 
            MovePosition(1.0, 0, 0);
        else
            MoveVelocity(1.0, 0, 0);
        break;
    case 'x':
        if (drone_data.mode_control == POSITION_MODE)
            MovePosition(-1.0, 0, 0);
        else
            MoveVelocity(-1.0, 0, 0);

        break;
    case 'a':  
        if (drone_data.mode_control == POSITION_MODE)
            MovePosition(0, -1.0, 0);
        else
            MoveVelocity(0, -1.0, 0);
        break;
    case 's':
        if (drone_data.mode_control == POSITION_MODE)
            MovePosition(0, 0, 0);
        else
            MoveVelocity(0, 0, 0);    
        break;
    case 'd':
        if (drone_data.mode_control == POSITION_MODE)
            MovePosition(0, 1.0, 0);
        else
            MoveVelocity(0, 1.0, 0);    
        break;
    case 'u':
        if (drone_data.mode_control == POSITION_MODE)
            MovePosition(0, 0, 1.0);
        else
            MoveVelocity(0, 0, 1.0);    
        break;
    case 'j':
        if (drone_data.mode_control == POSITION_MODE)
            MovePosition(0, 0, -1.0);
        else
            MoveVelocity(0, 0, -1.0);
        break;
    case '1':
        if (drone_data.mode_control == POSITION_MODE)
            MovePositionAngle(1.0);
        else
            MoveVelocityAngle(0.5);
        break;
    case '2':
        if (drone_data.mode_control == POSITION_MODE)
            MovePositionAngle(0);
        else
            MoveVelocityAngle(0);
        break;
    case '3':
        if (drone_data.mode_control == POSITION_MODE)
            MovePositionAngle(-1.0);
        else
            MoveVelocityAngle(-0.5);
        break;

    case 'm':
        Change_ControlMode(!drone_data.mode_control);
        break;

    case 't':
        if (activate_tracking == TRACKING_DEACTIVATE)
        {
            ROS_WARN("[FAIL] Tracking is deactivate [%d]", activate_tracking);
            InitCMDData(cmd_info);
            return;
        }

        Change_TrackingMode(!drone_data.mode_tracking);
        tracking_detection_timeout = {};
        tracking_detection_timeout.time_last_in = ros::Time::now().toSec();
        break;
        
    case 'f':
        fastplanner = !fastplanner;
        if (!fastplanner)
        {
            SetCurrentPose();
            SetCurrentYaw();
        }
                
        ROS_INFO("[SUCCESS] Planner mode [%s]", fastplanner ? "on" : "Off");
        break;
        
    }

        

    cmd_info.time_start = ros::Time::now().toSec();
}

// Execute object tracking
void Drone::DoTracking(){

    if (tracking_data.msg.twist.linear.y == 1){
        Land();
        return;
    }

    // During tracking, Fix the height to default value
    double velocity_z = 0;
    double accel_z = 0.1;
    double threshold_z = 0.2;
    if (current_pose.pose.position.z >= (drone_data.default_height + threshold_z))
        velocity_z = -1 * accel_z;
    else if (current_pose.pose.position.z <= (drone_data.default_height - threshold_z))
        velocity_z = accel_z;

    MoveVelocity(tracking_data.msg.twist.linear.x, 0, velocity_z);
    MoveVelocityAngle(tracking_data.msg.twist.angular.z);
        
    tracking_data.time_start = ros::Time::now().toSec();
}


bool Drone::CheckTrackingValue()
{   
    if (tracking_data.msg.twist.linear.x == 0 && tracking_data.msg.twist.linear.y == 0 && tracking_data.msg.twist.linear.z == 0 && tracking_data.msg.twist.angular.z == 0)
    {
        // If All data in tracking_msg is zero-value, Change move_mode to POSITION_MODE for hovering
        if (!tracking_value_timeout.trigger_stop)
        {
            if (tracking_value_timeout.time_zero_veloity == 0)
            {
                tracking_value_timeout.time_zero_veloity = tracking_data.time_in;
            }
            else
            {
                if (tracking_data.time_in - tracking_value_timeout.time_zero_veloity >= 1.5)
                {
                    tracking_value_timeout.trigger_stop = true;
                    Change_ControlMode(POSITION_MODE);
                    set_localPose.pose.position.z = drone_data.default_height;
                    ROS_WARN("[TIMEOUT] No tracking tommand [N:%.2lf,%.2lf,%.2lf", set_localPose.pose.position.x, set_localPose.pose.position.y, set_localPose.pose.position.z);
                }
            }
        }
    }
    else{
        // If Any data in tracking_msg is non-zero value, Change move_mode to VELOCITY_MODE and Resume Tracking
        tracking_value_timeout = {};
        Change_ControlMode(VELOCITY_MODE);
    }

    tracking_data.time_start = ros::Time::now().toSec();
    return true;

}
