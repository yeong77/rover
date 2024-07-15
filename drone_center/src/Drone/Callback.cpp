#include <drone.h>

/* MAV_STATE -> system_status 
https://mavlink.io/en/messages/common.html
0	MAV_STATE_UNINIT	Uninitialized system, state is unknown.
1	MAV_STATE_BOOT	System is booting up.
2	MAV_STATE_CALIBRATING	System is calibrating and not flight-ready.
3	MAV_STATE_STANDBY	System is grounded and on standby. It can be launched any time.
4	MAV_STATE_ACTIVE	System is active and might be already airborne. Motors are engaged.
5	MAV_STATE_CRITICAL	System is in a non-normal flight mode. It can however still navigate.
6	MAV_STATE_EMERGENCY	System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
7	MAV_STATE_POWEROFF	System just initialized its power-down sequence, will shut down now.
8	MAV_STATE_FLIGHT_TERMINATION	System is terminating itself.
*/


void Drone::Callback_state(const mavros_msgs::State::ConstPtr& state_msg)
{
    if (drone_data.current_state.mode != state_msg->mode){
        cmd_info.success = true;
        ROS_INFO("[INFO] Mode Changed [%s->%s]", drone_data.current_state.mode.c_str(), state_msg->mode.c_str());
        
        if (state_msg->mode == "OFFBOARD" or state_msg->mode == "AUTO.MISSION" or state_msg->mode == "MANUAL")
        {   
            // Flying & Arming+offboard => set current Position & set default height for takeoff
            if (drone_data.current_state.armed){
                SetCurrentPose();
                SetCurrentYaw();
                
                // Check height and Set height
                if (current_pose.pose.position.z <= 0.4)
                    SetDefaultHeight();
            }
            else{
                // 
                SetCurrentPose();
                SetCurrentYaw();
                SetDefaultHeight();
            }
        }
    }    

    // Check failsafe
    if (state_msg->system_status >= 5){
        if (state_msg->system_status <= 8){
            ROS_ERROR("[EMERGENCY] Code Num : %d", state_msg->system_status);
            Shutdown();
            return;
            
        }
        else{
            ROS_ERROR("[EMERGENCY] Dump Value : %d", state_msg->system_status);
            return;
        }
    }

    drone_data.current_state = *state_msg;

}



void Drone::Callback_tracking(const geometry_msgs::TwistStamped::ConstPtr& tracking_msg)
{
    // Check tracking mode
    if (drone_data.mode_tracking == false)
        return;

    // Check working command
    if (tracking_data.check_data == true)
        return;

    // Store time when the command come in (Timeout)
    tracking_data.time_in = ros::Time::now().toSec();

    tracking_detection_timeout.trigger_hover = false;
    tracking_detection_timeout.time_last_in = tracking_data.time_in;
    
    // Communication Timeout (if Subscribed message is old, Don't execute the message)
    if (tracking_data.time_in - tracking_msg->header.stamp.toSec() >= drone_data.timeout_communication)
    {
        ROS_WARN("[TIMEOUT] Old Tracking data [Du:%lf]", tracking_data.time_in - tracking_msg->header.stamp.toSec());
        tracking_data = {};
        return;
    }

    // Tracking trigger On
    tracking_data.check_data = true;
    tracking_data.msg = *tracking_msg;
    
    // ROS_INFO("[DEBUG] tracking data [%lf]", tracking_data.time_in);

}

// Store a request to execute the request.
void Drone::Callback_request(const std_msgs::String::ConstPtr& request_msg)
{
    if (cmd_info.command == '\0')
    {
        char ch = *(request_msg->data.c_str());

        if (ch == 'q' || ch == 'Q')
        {
            Shutdown();
            return;
        }

        CheckCommand(ch);      

        std_msgs::Header respond_msg;
        respond_msg.stamp = ros::Time::now();
        respond_msg.frame_id += ch;
        respond_msg.frame_id += "received";
        pub_respond.publish(respond_msg);
    }
}


// Store current position
void Drone::Callback_localPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    current_pose = *pose_msg;
}

void Drone::callback_fastplanner(const mavros_msgs::PositionTarget::ConstPtr& planner_msg)
{
    if (!fastplanner)
        return;
    
    set_localRaw = *planner_msg;
}

void Drone::callback_detection(const std_msgs::UInt8::ConstPtr& data)
{
    set_mode = *data;
}
