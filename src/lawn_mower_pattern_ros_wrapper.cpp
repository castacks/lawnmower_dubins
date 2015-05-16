#include "lawn_mower_pattern/lawn_mower_pattern_ros_wrapper.h"


visualization_msgs::Marker ca::LawnMowerPatternROS::GetPatternMarker(std::vector<ca::LawnMowerPoint> &path){
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = _frame;
    m.action = visualization_msgs::Marker::ADD;
    m.id = 0;
    m.ns = "lawn_mower";
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.lifetime = ros::Duration(0);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;

    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0;
    m.scale.x = 1.0; m.scale.y = 0.0; m.scale.z = 0.0;
    for(size_t i=0; i<path.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = path[i].position.x();
        p.y = path[i].position.y();
        p.z = path[i].position.z();

        m.points.push_back(p);
    }

    return m;
};

void ca::LawnMowerPatternROS::PublishPatternMarker(){
    if(_path.size()==0)
        _lawn_mower_pattern->GenerateLawnMowerPattern(_path);
    visualization_msgs::Marker m = GetPatternMarker(_path);
    _marker_publisher.publish(m);
}

ca::LawnMowerPatternROS::LawnMowerPatternROS(ros::NodeHandle &n)
{
    double box_x, box_y, altitude, row_distance,temporal_res,radius,velocity;
    box_x = box_y =100; row_distance = 10; temporal_res = 0.3; radius = 5; velocity = 1;
     _lawn_mower_pattern = new ca::LawnMowerPattern(box_x,box_y,altitude,row_distance,temporal_res,radius,velocity);
    _frame = "world";
     _pattern_publisher = n.advertise<visualization_msgs::Marker>("coverage_trajectory", 1000);;
     _marker_publisher = n.advertise<visualization_msgs::Marker>("coverage_marker", 1000);
}
