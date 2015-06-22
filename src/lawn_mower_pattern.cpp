#include "lawn_mower_pattern/lawn_mower_pattern.h"

ca::LawnMowerPattern::LawnMowerPattern(double box_x, double box_y, double altitude, double row_distance, double temporal_resolution, double radius, double velocity, double acceleration){
    _box_size_x = box_x;
    _box_size_y = box_y;
    _pattern_altitude = altitude;
    _row_distance = row_distance;
    _temporal_resolution = temporal_resolution;
    _radius = radius;
    _velocity = velocity;
	_acceleration = acceleration;
}

std::vector<double> ca::LawnMowerPattern::TimeTrajectory(std::vector<LawnMowerPoint> &trajectory){

    double t =0;
    std::vector<double> time;
    time.push_back(t);
    for(size_t i=0; i<trajectory.size()-1;i++)
    {
         Eigen::Vector3d s = trajectory[i+1].position - trajectory[i].position;
         double distance = s.norm();

         Eigen::Vector3d v = 0.5*trajectory[i+1].velocity + 0.5*trajectory[i].velocity;
         double velocity = v.norm();
         t=t+ distance/velocity;
         time.push_back(t);
    }

    return time;
}

size_t ca::LawnMowerPattern::FindTimeBasedID(double time, std::vector<double> &traj_time, size_t start_id){
    size_t id=start_id;
    for(;id<(traj_time.size()-1);id++)
    {
        if(traj_time[id]>time)
            return (id-1);
    }
    return id;
}

ca::LawnMowerPoint ca::LawnMowerPattern::InterpolateWaypoint(double interp, size_t first_id, std::vector<LawnMowerPoint> &trajectory, std::vector<double> time){
    if(first_id == (trajectory.size()-1))
        return trajectory[first_id];



    Eigen::Vector3d s = trajectory[first_id+1].position - trajectory[first_id].position;
    double distance = s.norm();
    s.normalize();

    double v = trajectory[first_id+1].velocity.x();
    double u =  trajectory[first_id].velocity.x();

    double a = (v*v - u*u)/(2*distance);
    if(distance ==0)
        a = 0;

    double t = time[first_id+1] - time[first_id];
    t = t*interp;

    distance = u*t + 0.5*a*t*t;
    v = u + a*t;
    ca::LawnMowerPoint lmp;
    lmp.position = trajectory[first_id].position + s*distance;
    lmp.velocity.x() = v;
    lmp.velocity.y() = 0.0; lmp.velocity.z() = 0.0;

    double heading_change =  math_utils::angular_math::TurnDirection (trajectory[first_id+1].heading, trajectory[first_id].heading);
    lmp.heading = trajectory[first_id].heading + heading_change*interp;
    return lmp;
}

void ca::LawnMowerPattern::RampVelocityProfile(std::vector<LawnMowerPoint> &path)
{
    if(path.size()==0)
		return;

	size_t path_size = path.size(); 
	path[0].velocity.x() = 0.0;
	// Ramping Up
    for(size_t i=1; i<path_size; i++){
        Eigen::Vector3d s = path[i].position - path[i-1].position;
        double distance = s.norm();
        double u = path[i-1].velocity.norm();
        double v = std::sqrt(u*u + 2*_acceleration*distance);
        path[i].velocity.x() = v;
		if(path[i].velocity.x() > _velocity)
			path[i].velocity.x() = _velocity;
	}

	// Ramping Down
	path[path_size-1].velocity.x() = 0.0;
	for(size_t i=path_size-2; i>=0; i--){
        Eigen::Vector3d s = path[i].position - path[i+1].position;
        double distance = s.norm();
        double u = path[i+1].velocity.norm();
        double v = std::sqrt(u*u + 2*_acceleration*distance);
        if(v >= path[i].velocity.x())
			break;
		else
			path[i].velocity.x() = v;
	}


    std::vector<LawnMowerPoint> new_path;
    std::vector<double> new_time = TimeTrajectory(path);

    double end_time = new_time[new_time.size()-1];

    size_t search_start_id = 0;
    for(double t = 0; t<= end_time; t = t+_temporal_resolution)
    {
        size_t id = FindTimeBasedID(t, new_time, search_start_id);
        search_start_id = id;
        //ROS_ERROR_STREAM("t::"<<t<<"::id::"<<id<<"::x::"<<path[id].position.x()<<"::vx::"<<path[id].velocity.x());
        double interp_constant = 0.0;
        if(id < (path.size()-1))
            interp_constant = (t - new_time.at(id))/(new_time.at(id+1) - new_time.at(id));

        LawnMowerPoint lmpoint = InterpolateWaypoint(interp_constant, id, path, new_time);
        //ROS_ERROR_STREAM("OP::"<<t<<"::x::"<<lmpoint.position.x()<<"::vx::"<<lmpoint.velocity.x());
        new_path.push_back(lmpoint);
    }
    path = new_path;
}

void ca::LawnMowerPattern::GeneratePath(double box_size_x, double box_size_y, double row_distance){
    double x,y,heading;
    x=y=heading=0;
    _path_x_y_heading.clear();

    if(1){//box_size_x >= box_size_y){
        heading = 0.0;
        double prev_x=x;
        double prev_y=y;
        double prev_heading = heading;
        while (y <= box_size_y){
            Eigen::Vector3d v(x,y,heading);
            _path_x_y_heading.push_back(v);
            if(x==0 && heading==0){
                x = box_size_x;
                y = prev_y;
                heading = 0.0;
            }
            else if(x==0 && heading==M_PI){
                x = 0.0;
                y = prev_y + row_distance;
                heading = 0.0;
            }
            else if(x==box_size_x && heading==0){
                x = box_size_x;
                y = prev_y + row_distance;
                heading = M_PI;
            }
            else if(x==box_size_x && heading==M_PI){
                x = 0.0;
                y = prev_y;
                heading = M_PI;
            }
            prev_y = y;
            prev_heading = heading;
        }

    }
}


bool ca::LawnMowerPattern::GenerateLawnMowerPattern(std::vector<LawnMowerPoint> & path){
    path.clear();
    GeneratePath(_box_size_x, _box_size_y, _row_distance);

    ca::dubin::DubinPath dubin_path;
    double sampling_distance = _velocity*_temporal_resolution;
    double t_offset=0;
    for(size_t i=0; i<_path_x_y_heading.size()-1;i++){
        Eigen::Vector3d v1 = _path_x_y_heading[i];
        Eigen::Vector3d v2 = _path_x_y_heading[i+1];
        double q0[3]; q0[0] = v1.x(); q0[1] = v1.y(); q0[2] = v1.z();
        double q1[3]; q1[0] = v2.x(); q1[1] = v2.y(); q1[2] = v2.z();

        if(!(ca::dubin::dubin_init(q0,q1, _radius, &dubin_path)==0)){
            return false;
        }

        double path_length = dubin_path_length( &dubin_path);
        double t= t_offset;
        for(; t<path_length; t=t+sampling_distance){
            double q[3];
            if(!(dubin_path_sample( &dubin_path, t, q)==0))
                    return false;

            LawnMowerPoint lmp;
            lmp.heading = q[2];
            lmp.position.x() = q[0]; lmp.position.y() = q[1]; lmp.position.z() = _pattern_altitude;
            lmp.velocity.x() = _velocity; lmp.velocity.y() = 0.0; lmp.velocity.z() = 0.0;
            path.push_back(lmp);

        }

        t_offset = t-path_length;
    }

	RampVelocityProfile(path);

    return true;
}
