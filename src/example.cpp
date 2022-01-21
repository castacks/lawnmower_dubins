#include "ros/ros.h"
#include "lawn_mower_pattern/lawn_mower_pattern_ros_wrapper.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle n("~");
    ros::Rate loop_rate(10);
    ca::LawnMowerPatternROS lmp;
    lmp.Initialize(n);
    int count = 0;
    while (ros::ok())
    {
        // lmp.PublishPatternMarker();
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }


    return 0;
}
