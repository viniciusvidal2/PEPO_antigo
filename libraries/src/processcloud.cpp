#include "../include/processcloud.h"

ProcessCloud::ProcessCloud()
{

}

ProcessCloud::~ProcessCloud(){
    ros::shutdown();
    ros::waitForShutdown();
}
