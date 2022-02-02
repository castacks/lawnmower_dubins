#include "lawn_mower_pattern/lawn_mower_node.h"
#include "lawn_mower_pattern/lawn_mower.h"

std::vector<std::vector<std::vector<double>>> polygons2 = { 
            {{425,53},{172,150},{319,273},{260,146}},
            {{292,150},{457,282},{219,398},{373,274}},
            {{150,21},{22,261},{193,456},{516,451},{676,138},{529,28}}
        };

std::vector<std::vector<std::vector<double>>> polygons1 = { 
            {{200,146},{111,235},{258,358},{363,221}},
            {{294,69},{591,171},{436,399},{473,206}},
            {{150,21},{22,261},{193,456},{516,451},{676,138},{529,28}} // last element is enclosing region
        };

int env_marker_id = 0;
bool _got_odometry = false;
bool _initialized = false;
double box_x, box_y, altitude, row_distance, temporal_res, radius, velocity, acceleration;

visualization_msgs::Marker ca::LawnMowerPatternROS::VisualizeEnvironmentBounds(std::vector<std::vector<double>> polygon)
{
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = _frame;
    m.ns = "environment_bounds";
    m.id = env_marker_id++;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 1.0;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    for (int i=0; i<polygon.size(); ++i)
    {
        geometry_msgs::Point from_point;
        from_point.x = polygon[i][0];
        from_point.y = polygon[i][1];
        from_point.z = altitude;
        m.points.push_back(from_point);
        
        geometry_msgs::Point to_point;
        to_point.x = polygon[(i+1)%polygon.size()][0];
        to_point.y = polygon[(i+1)%polygon.size()][1];
        to_point.z = altitude;
        m.points.push_back(to_point);

    }
    return m;
}

void ca::LawnMowerPatternROS::PublishPatternTrajectory(){  
    for (int i=0; i<polygons1.size(); ++i)
    {
        _environment_markers.publish(VisualizeEnvironmentBounds(polygons1[i]));
    }
}

void ca::LawnMowerPatternROS::Initialize(ros::NodeHandle &n)
{    
    box_x = box_y =10; row_distance = 1; temporal_res = 0.3; radius = 1; velocity = 0.2;
	acceleration = 0.5;
    std::string work_frame = "world";
    std::string trajectory_topic = "trajectory";
    std::string odo_topic = "odometry";
    std::string visualization_topic = "marker";
    std::string environment_topic = "environment";
    std::string publish_trajectory_topic = "publish";
    bool got_param = true;

    got_param = got_param && n.getParam("work_frame", work_frame);
    got_param = got_param && n.getParam("trajectory_topic", trajectory_topic);
    got_param = got_param && n.getParam("odo_topic", odo_topic);
    got_param = got_param && n.getParam("visualization_topic", visualization_topic);
    got_param = got_param && n.getParam("environment_topic", environment_topic);
    got_param = got_param && n.getParam("publish_trajectory_topic", publish_trajectory_topic);

    got_param = got_param && n.getParam("row_distance", row_distance);
    got_param = got_param && n.getParam("temporal_res", temporal_res);
    got_param = got_param && n.getParam("budget", budget);
    got_param = got_param && n.getParam("radius", radius);
    got_param = got_param && n.getParam("velocity", velocity);
	got_param = got_param && n.getParam("acceleration", acceleration);
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

    _frame = work_frame;
    _environment_markers = n.advertise<visualization_msgs::Marker>(environment_topic, 10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lawn_mower_node");
    std::cout << "hello!!" << std::endl;
    ros::NodeHandle n("~");
    ca::LawnMowerPatternROS lmp;
    lmp.Initialize(n);

    lm::LawnMower lawn_mower;
    lawn_mower.cellDecomposition(polygons1);
    std::vector<lm::Trapezoid*> trapezoids = lawn_mower.getTrapezoidList();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        lmp.PublishPatternTrajectory();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros::spin();
    return 0;
}
