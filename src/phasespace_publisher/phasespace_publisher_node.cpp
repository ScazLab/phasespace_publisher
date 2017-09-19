#include "phasespace_publisher/phasespace_publisher.h"

using namespace std;

int main(int argc, char ** argv)
{
    ROS_INFO("Reading points:");
    ros::init(argc, argv, "ps_markers");
    PhasespacePublisher ps("ps_markers");


    ros::spin();
    return 0;
}
