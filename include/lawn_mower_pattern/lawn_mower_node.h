#ifndef LAWN_MOWER_PATTERN_ROS_WRAPPER_H
#define LAWN_MOWER_PATTERN_ROS_WRAPPER_H
#include "ros/ros.h"
#include "lawn_mower_pattern/lawn_mower.h"
#include "visualization_msgs/Marker.h"
#include "ca_common/math.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_msgs/ColorRGBA.h"
#include "tf_utils/tf_utils.h"

namespace ca{

class LawnMowerPatternROS{
private:
    std::string _frame;
    ros::Publisher _marker_publisher;
    ros::Publisher _environment_markers;
    ros::Publisher _pattern_publisher;
    ros::Subscriber _odo_subscriber;
    ros::Subscriber _publish_trajectory_subscriber;
    tf::Transform _transform;
    double budget;
    double _scale;
    std_msgs::ColorRGBA _color;
    std::string _namespace;
    double _time_resolution;
    double _current_heading;
    bool _constant_heading;
public:
    visualization_msgs::Marker VisualizeEnvironmentBounds(std::vector<std::vector<double>> polygon);
    void PublishPatternTrajectory();
    void Initialize(ros::NodeHandle &n);
};
}
#endif // LAWN_MOWER_PATTERN_ROS_WRAPPER_H
