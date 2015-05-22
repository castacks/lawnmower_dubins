#include "ros/ros.h"
#include "lawn_mower_pattern/lawn_mower_pattern_ros_wrapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lawn_mower_pattern_generator");

    ros::NodeHandle n("~");
    ca::LawnMowerPatternROS lmp;
    lmp.Initialize(n);
    ros::spin();
    return 0;
}
