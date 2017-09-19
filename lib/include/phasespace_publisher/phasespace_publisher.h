#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "robot_utils/rviz_publisher.h"
#include "phasespace_publisher/PhasespacePt.h"
#include "phasespace_publisher/PhasespacePtArray.h"

class PhasespacePublisher
{
private:
    ros::NodeHandle nh;
    RVIZPublisher   rviz_pub;
    ros::Subscriber phasespace_sub;

public:
    PhasespacePublisher(std::string name);

    void passMarkers(const phasespace_publisher::PhasespacePtArray&);

    std::vector<RVIZMarker> getMarkers() { return rviz_pub.getMarkers(); };

};
