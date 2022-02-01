#ifndef LAWN_MOWER_PATTERN_H
#define LAWN_MOWER_PATTERN_H
#include "math_utils/math_utils.h"
#include <vector>
#include "circular_curve_lib/dubins.h"
#include "ros/ros.h"

namespace ca{

struct LawnMowerPoint{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double heading;
};

class LawnMowerPattern{
private:
    double _box_size_x;
    double _box_size_y;
    double _pattern_altitude;
    double _row_distance;
    double _temporal_resolution;
    double _radius;
    double _velocity;
	double _acceleration;
    std::vector<Eigen::Vector3d> _path_x_y_heading;
    std::vector<double> _trajectory_time;
    double GetAcceleration(double distance, double u, double v ){
        return distance/(v*v - u*u);
    };
    std::vector<double> TimeTrajectory(std::vector<LawnMowerPoint> &trajectory);
    size_t FindTimeBasedID(double time, std::vector<double> &traj_time, size_t start_id);
    LawnMowerPoint InterpolateWaypoint(double interp, size_t first_id, std::vector<LawnMowerPoint> &trajectory, std::vector<double> time);
public:
    void GeneratePath(double box_size_x, double box_size_y, double row_distance);
    std::vector<std::vector<double>> intersectBoundary(std::vector<std::vector<double>> boundary_polygon, std::vector<double> coord);
	void RampVelocityProfile(std::vector<LawnMowerPoint> &path);
    bool GenerateLawnMowerPattern(std::vector<LawnMowerPoint> & path);
    void set_box_size(double x, double y){
        _box_size_x = x; _box_size_y = y;
    }
    void set_pattern_altitude(double param){
        _pattern_altitude = param;
    }
    void set_row_distance(double param){
        _row_distance = param;
    }
    void set_temporal_resolution(double param){
        _temporal_resolution = param;
    }
    void set_radius(double param){
        _radius = param;
    }
    void set_velocity(double param){
        _velocity = param;
    }

    LawnMowerPattern(double box_x, double box_y, double altitude, double row_distance, double temporal_resolution, double radius, double velocity, double acceleration);
};
}
#endif // LAWN_MOWER_PATTERN_H
