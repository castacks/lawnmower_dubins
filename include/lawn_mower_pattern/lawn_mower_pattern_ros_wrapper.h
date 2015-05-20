#ifndef LAWN_MOWER_PATTERN_ROS_WRAPPER_H
#define LAWN_MOWER_PATTERN_ROS_WRAPPER_H
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "lawn_mower_pattern/lawn_mower_pattern.h"
#include "visualization_msgs/Marker.h"
#include "lawn_mower_pattern/Trajectory.h"
#include "ca_common/math.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_msgs/ColorRGBA.h"
#include "tf_utils/tf_utils.h"

namespace ca{

class LawnMowerPatternROS{
private:
    std::string _frame;
    LawnMowerPattern* _lawn_mower_pattern;
    std::vector<ca::LawnMowerPoint> _path;
    ros::Publisher _marker_publisher;
    ros::Publisher _pattern_publisher;
    ros::Subscriber _odo_subscriber;
    ros::Subscriber _publish_trajectory_subscriber;
    tf::Transform _transform;
    lawn_mower_pattern::Trajectory _traj;
    bool _got_odometry;
    bool _initialized;
    inline double GetHeading(CA::QuatD q)
    {
      double heading = CA::quatToEuler(q).z();
      return heading;
    }
    void TransformTrajectory(lawn_mower_pattern::Trajectory &trajectory, tf::Transform &transform);
    double _scale;
    std_msgs::ColorRGBA _color;
    std::string _namespace;
public:
    visualization_msgs::Marker GetPatternMarker(std::vector<ca::LawnMowerPoint> &path);
    visualization_msgs::Marker GetTrajectoryMarker(lawn_mower_pattern::Trajectory &trajectory);
    lawn_mower_pattern::Trajectory GetPatternTrajectory(std::vector<ca::LawnMowerPoint> &path);
    void PublishPatternMarker();
    void PublishPatternTrajectory();
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odo_msg);
    void SendTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg){
        if(!_got_odometry){
            ROS_WARN("Lawn Mower Generator:: Odometry not received!");
            return;
        }
        _path.clear();
        PublishPatternTrajectory();
        visualization_msgs::Marker m = GetTrajectoryMarker(_traj);
        _marker_publisher.publish(m);
    }

    void Initialize(ros::NodeHandle &n);
    LawnMowerPatternROS();
    ~LawnMowerPatternROS(){
        if(_initialized)
            delete _lawn_mower_pattern;
    }
};
}
#endif // LAWN_MOWER_PATTERN_ROS_WRAPPER_H


// List of todos
//1. write the function to transform the trajectories  --done
//2. write callback to accept odometry -- done
//3. write the callback to publish trajectory --done
//4. test trajectory following
//5. Make the heading invariant mode
