#include "lawn_mower_pattern/lawn_mower_pattern_ros_wrapper.h"

void ca::LawnMowerPatternROS::ConvertLocalToGlobalVelocities(lawn_mower_pattern::Trajectory &trajectory){
    if(trajectory.trajectory.size()==0)
        return;
    Eigen::Vector3d v; v.x()=0; v.y()=0; v.z()=0;

    for(size_t i=0; i<trajectory.trajectory.size()-1;i++){
        Eigen::Vector3d v1 = CA::msgc(trajectory.trajectory[i].position);
        Eigen::Vector3d v2 = CA::msgc(trajectory.trajectory[i+1].position);
        Eigen::Vector3d velocity = (v2-v1)/_time_resolution;
        trajectory.trajectory[i].velocity.x = velocity.x();
        trajectory.trajectory[i].velocity.y = velocity.y();
        trajectory.trajectory[i].velocity.z = velocity.z();
    }
    trajectory.trajectory[trajectory.trajectory.size()-1].velocity.x = v.x();
    trajectory.trajectory[trajectory.trajectory.size()-1].velocity.y = v.y();
    trajectory.trajectory[trajectory.trajectory.size()-1].velocity.z = v.z();

}


void ca::LawnMowerPatternROS::TransformTrajectory(lawn_mower_pattern::Trajectory &trajectory, tf::Transform &transform){
    for(size_t i=0;i<trajectory.trajectory.size();i++)
    {
        CA::Vector3D rpy(0.0,0.0,trajectory.trajectory[i].heading);
        geometry_msgs::Quaternion q = CA::msgc(CA::eulerToQuat(rpy));
        ca::tf_utils::transformQuaternion(transform,q);
        trajectory.trajectory[i].heading = GetHeading(CA::msgc(q));

        ca::tf_utils::transformPoint(transform, trajectory.trajectory[i].position);
    }
}


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

    m.color = _color;
    m.scale.x = _scale; m.scale.y = 0.0; m.scale.z = 0.0;
    for(size_t i=0; i<path.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = path[i].position.x();
        p.y = path[i].position.y();
        p.z = path[i].position.z();

        m.points.push_back(p);
    }

    return m;
}


visualization_msgs::Marker ca::LawnMowerPatternROS::GetTrajectoryMarker(lawn_mower_pattern::Trajectory &trajectory){
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = _frame;
    m.action = visualization_msgs::Marker::ADD;
    m.id = 0;
    m.ns = _namespace;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.lifetime = ros::Duration(0);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;

    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.color = _color;
    m.scale.x = _scale; m.scale.y = 0.0; m.scale.z = 0.0;
    for(size_t i=0; i<trajectory.trajectory.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = trajectory.trajectory[i].position.x;
        p.y = trajectory.trajectory[i].position.y;
        p.z = trajectory.trajectory[i].position.z;

        m.points.push_back(p);
    }

    return m;
}


lawn_mower_pattern::Trajectory ca::LawnMowerPatternROS::GetPatternTrajectory(std::vector<ca::LawnMowerPoint> &path){

    lawn_mower_pattern::Trajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = _frame;

    lawn_mower_pattern::TrajectoryPoint tp;
    for(size_t i=0; i<path.size(); i++){
        tp.position.x = path[i].position.x();
        tp.position.y = path[i].position.y();
        tp.position.z = path[i].position.z();

        tp.velocity.x = path[i].velocity.x();
        tp.velocity.y = path[i].velocity.y();
        tp.velocity.z = path[i].velocity.z();
        if(_constant_heading)
            tp.heading = _current_heading;
        else
            tp.heading = path[i].heading;

        traj.trajectory.push_back(tp);
    }

    return traj;
}


void ca::LawnMowerPatternROS::PublishPatternMarker(){
    if(_path.size()==0){
        _lawn_mower_pattern->GenerateLawnMowerPattern(_path);
        _traj = GetPatternTrajectory(_path);
    }
    TransformTrajectory(_traj,_transform);
    ConvertLocalToGlobalVelocities(_traj);
    visualization_msgs::Marker m = GetTrajectoryMarker(_traj);
    _marker_publisher.publish(m);
}

void ca::LawnMowerPatternROS::PublishPatternTrajectory(){
    if(_path.size()==0){
        _lawn_mower_pattern->GenerateLawnMowerPattern(_path);
        _traj = GetPatternTrajectory(_path);
    }
    TransformTrajectory(_traj,_transform);
    ConvertLocalToGlobalVelocities(_traj);
    _pattern_publisher.publish(_traj);
}

void ca::LawnMowerPatternROS::OdometryCallback(const nav_msgs::Odometry::ConstPtr& odo_msg){
    nav_msgs::Odometry odo = *odo_msg;
    tf::Vector3 v;
    v.setX(odo.pose.pose.position.x);
    v.setY(odo.pose.pose.position.y);
    v.setZ(odo.pose.pose.position.z);
    _transform.setOrigin(v);

    tf::Quaternion q;
    CA::QuatD q_ca = CA::msgc(odo.pose.pose.orientation);
    CA::Vector3D v_ca = CA::quatToEuler(q_ca);
    v_ca.x() = 0.0; v_ca.y() = 0.0;
    q_ca = CA::eulerToQuat(v_ca);
    q.setX(q_ca.x()); q.setY(q_ca.y()); q.setZ(q_ca.z()); q.setW(q_ca.w());
    _transform.setRotation(q);

    _current_heading = GetHeading(q_ca);
    _got_odometry = true;
}

ca::LawnMowerPatternROS::LawnMowerPatternROS()
{
    _got_odometry = false;
    _initialized = false;
}

void ca::LawnMowerPatternROS::Initialize(ros::NodeHandle &n)
{
    double box_x, box_y, altitude, row_distance,temporal_res,radius,velocity;
    box_x = box_y =10; row_distance = 1; temporal_res = 0.3; radius = 1; velocity = 0.2;
    std::string work_frame = "world";
    std::string trajectory_topic = "trajectory";
    std::string odo_topic = "odometry";
    std::string visualization_topic = "marker";
    std::string publish_trajectory_topic = "publish";
    bool got_param = true;

    got_param = got_param && n.getParam("work_frame", work_frame);
    got_param = got_param && n.getParam("trajectory_topic", trajectory_topic);
    got_param = got_param && n.getParam("odo_topic", odo_topic);
    got_param = got_param && n.getParam("visualization_topic", visualization_topic);
    got_param = got_param && n.getParam("publish_trajectory_topic", publish_trajectory_topic);

    got_param = got_param && n.getParam("box_x", box_x);
    got_param = got_param && n.getParam("box_y", box_y);
    got_param = got_param && n.getParam("row_distance", row_distance);
    got_param = got_param && n.getParam("temporal_res", temporal_res);
    got_param = got_param && n.getParam("radius", radius);
    got_param = got_param && n.getParam("velocity", velocity);
    got_param = got_param && n.getParam("altitude", altitude);
    _time_resolution = temporal_res;
    double r,g,b,a;
    got_param = got_param && n.getParam("scale",_scale);
    got_param = got_param && n.getParam("r",r);
    got_param = got_param && n.getParam("g",g);
    got_param = got_param && n.getParam("b",b);
    got_param = got_param && n.getParam("a",a);
    got_param = got_param && n.getParam("namespace",_namespace);
    got_param = got_param && n.getParam("constant_heading",_constant_heading);

    if(!got_param){
        ROS_ERROR("Lawn mower pattern generator could not get all the parameters.");
        exit(-1);
    }

    _color.r = r; _color.g = g; _color.b = b; _color.a = a;

    _lawn_mower_pattern = new ca::LawnMowerPattern(box_x,box_y,altitude,row_distance,temporal_res,radius,velocity);
    _frame = work_frame;
    _pattern_publisher = n.advertise<lawn_mower_pattern::Trajectory>(trajectory_topic, 10);
    _marker_publisher = n.advertise<visualization_msgs::Marker>(visualization_topic, 10);
    _odo_subscriber = n.subscribe(odo_topic, 1, &LawnMowerPatternROS::OdometryCallback, this);
    _publish_trajectory_subscriber = n.subscribe(publish_trajectory_topic, 1, &LawnMowerPatternROS::SendTrajectoryCallback, this);
}
