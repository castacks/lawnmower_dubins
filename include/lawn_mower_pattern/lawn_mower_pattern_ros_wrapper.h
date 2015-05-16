#ifndef LAWN_MOWER_PATTERN_ROS_WRAPPER_H
#define LAWN_MOWER_PATTERN_ROS_WRAPPER_H
#include "ros/ros.h"
//#include "tf/transform_listener.h"
#include "lawn_mower_pattern/lawn_mower_pattern.h"
#include "visualization_msgs/Marker.h"

namespace ca{

class LawnMowerPatternROS{
public:
    std::string _frame;
    LawnMowerPattern* _lawn_mower_pattern;
    std::vector<ca::LawnMowerPoint> _path;
    visualization_msgs::Marker GetPatternMarker(std::vector<ca::LawnMowerPoint> &path);
    void PublishPatternMarker();
    ros::Publisher _pattern_publisher;
    ros::Publisher _marker_publisher;
    LawnMowerPatternROS(ros::NodeHandle &n);
    ~LawnMowerPatternROS(){delete _lawn_mower_pattern;}
};
}
#endif // LAWN_MOWER_PATTERN_ROS_WRAPPER_H
