#include "lawn_mower_pattern/lawn_mower_pattern.h"

ca::LawnMowerPattern::LawnMowerPattern(double box_x, double box_y, double altitude, double row_distance, double temporal_resolution, double radius, double velocity){
    _box_size_x = box_x;
    _box_size_y = box_y;
    _pattern_altitude = altitude;
    _row_distance = row_distance;
    _temporal_resolution = temporal_resolution;
    _radius = radius;
    _velocity = velocity;
}

void ca::LawnMowerPattern::GeneratePath(double box_size_x, double box_size_y, double row_distance){
    double x,y,heading;
    x=y=heading=0;
    _path_x_y_heading.clear();

    if(box_size_x >= box_size_y){
        heading = 0.0;
        double prev_x=x;
        double prev_y=y;
        double prev_heading = heading;
        while (y < box_size_y){
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

    return true;
}
