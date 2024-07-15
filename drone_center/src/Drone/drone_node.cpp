#include <drone.h>


// http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
// void mySigintHandler(int sig)
// {
//     ros::shutdown();
// }


int main(int argc, char** argv){
    ros::init(argc, argv, "drone_control_node");
    ros::NodeHandle nodehandle;

    // signal(SIGINT, mySigintHandler);

    ROS_INFO("Node start");

    Drone drone(nodehandle);
    drone.Control();

    // ros::spin();
    
    return 0;
}
