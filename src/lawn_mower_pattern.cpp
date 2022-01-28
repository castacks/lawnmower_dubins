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

int ca::LawnMowerPattern::x_intersect(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
{
    int num = (x1*y2 - y1*x2) * (x3-x4) -
              (x1-x2) * (x3*y4 - y3*x4);
    int den = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4);
    return num/den;
}
  
// Returns y-value of point of intersection of
// two lines
int ca::LawnMowerPattern::y_intersect(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
{
    int num = (x1*y2 - y1*x2) * (y3-y4) -
              (y1-y2) * (x3*y4 - y3*x4);
    int den = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4);
    return num/den;
}

std::vector<std::vector<double>> ca::LawnMowerPattern::intersectBoundary(std::vector<std::vector<double>> boundary_polygon, std::vector<double> coord){
    std::vector<std::vector<double>> intersects;
    for (int i=0; i<boundary_polygon.size(); ++i)
    {
        double a_val = boundary_polygon[i][1] - coord[1] - M_PI_2 * (boundary_polygon[i][0] - coord[0]);
        double b_val = boundary_polygon[(i+1) % boundary_polygon.size()][1] - coord[1] - M_PI_2 * (boundary_polygon[(i+1) % boundary_polygon.size()][0] - coord[0]);
        // if intersect
        if (a_val/abs(a_val) != b_val/abs(b_val))
        {
            // find slope of edge
            double m = boundary_polygon[(i+1) % boundary_polygon.size()][1] - boundary_polygon[i][1] / (boundary_polygon[(i+1) % boundary_polygon.size()][0] - boundary_polygon[i][0]);
            intersects.push_back(
                {
                    (- (coord[1] - M_PI_2 * coord[0]) + (boundary_polygon[i][1] - m*boundary_polygon[i][0])) / (- m + M_PI_2), 
                    ((boundary_polygon[i][1] - m*boundary_polygon[i][0]) * M_PI_2 - (coord[1] - M_PI_2 * coord[0]) * m) / (- m + M_PI_2),
                    -1  // -1 denotes boundary intercept
                }
            );   
        }
    }
    return intersects;
}

void ca::LawnMowerPattern::clip(std::vector<std::vector<double>> poly_points, int poly_size, double x1, double y1, double x2, double y2)
{
    std::vector<std::vector<double>> new_points;
    int new_poly_size = 0;
  
    // (ix,iy),(kx,ky) are the co-ordinate values of
    // the points
    for (int i = 0; i < poly_size; i++)
    {
        // i and k form a line in polygon
        int k = (i+1) % poly_size;
        int ix = poly_points[i][0], iy = poly_points[i][1];
        int kx = poly_points[k][0], ky = poly_points[k][1];
  
        // Calculating position of first point
        // w.r.t. clipper line
        int i_pos = (x2-x1) * (iy-y1) - (y2-y1) * (ix-x1);
  
        // Calculating position of second point
        // w.r.t. clipper line
        int k_pos = (x2-x1) * (ky-y1) - (y2-y1) * (kx-x1);
  
        // Case 1 : When both points are inside
        if (i_pos < 0  && k_pos < 0)
        {
            //Only second point is added
            new_points[new_poly_size][0] = kx;
            new_points[new_poly_size][1] = ky;
            new_poly_size++;
        }
  
        // Case 2: When only first point is outside
        else if (i_pos >= 0  && k_pos < 0)
        {
            // Point of intersection with edge
            // and the second point is added
            new_points[new_poly_size][0] = x_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_points[new_poly_size][1] = y_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_poly_size++;
  
            new_points[new_poly_size][0] = kx;
            new_points[new_poly_size][1] = ky;
            new_poly_size++;
        }
  
        // Case 3: When only second point is outside
        else if (i_pos < 0  && k_pos >= 0)
        {
            //Only point of intersection with edge is added
            new_points[new_poly_size][0] = x_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_points[new_poly_size][1] = y_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_poly_size++;
        }
  
        // Case 4: When both points are outside
        else
        {
            //No points are added
        }
    }
  
    // Copying new points into original array
    // and changing the no. of vertices
    poly_size = new_poly_size;
    for (int i = 0; i < poly_size; i++)
    {
        poly_points[i][0] = new_points[i][0];
        poly_points[i][1] = new_points[i][1];
    }
}

// Implement Sutherland–Hodgman algorithm
void ca::LawnMowerPattern::suthHodgClip(std::vector<std::vector<double>> poly_points, std::vector<std::vector<double>> clipper_points)
{
    //i and k are two consecutive indexes
    for (int i=0; i<clipper_points.size(); i++)
    {
        int k = (i+1) % clipper_points.size();
  
        // We pass the current array of vertices, it's size
        // and the end points of the selected clipper line
        clip(poly_points, poly_points.size(), clipper_points[i][0],
             clipper_points[i][1], clipper_points[k][0],
             clipper_points[k][1]);
    }
  
    // Printing vertices of clipped polygon
    for (int i=0; i < poly_points.size(); i++)
        std::cout << '(' << poly_points[i][0] <<
                ", " << poly_points[i][1] << ") ";
}

void ca::LawnMowerPattern::cellDecomposition(std::vector<std::vector<double>> boundary_polygon, std::vector<std::vector<std::vector<double>>> obstacle_polygons, double row_distance){
    double x,y,heading;
    x=y=heading=0;
    _path_x_y_heading.clear();

    /* sort the boundary and obstacle polygons by x-coords */
    // identify boundary coords by 0  
    for (int i=0; i<boundary_polygon.size(); ++i)
        boundary_polygon[i].push_back(0.0);
    
    // identify obstacle coords by 1
    for (int i=0; i<obstacle_polygons.size(); ++i)
    {
        for (int j=0; j<obstacle_polygons[i].size(); ++j)
            obstacle_polygons[i][j].push_back(1.0);
    }
    // copy the boundary polygon into a new vector for manipulation
    auto ordered_coords = boundary_polygon;
    
    // first flatten obstacle polygon vector into a 2D vector of coords in boundary polygon vector
    for (int i=0; i<obstacle_polygons.size(); ++i)
        ordered_coords.insert(ordered_coords.end(), obstacle_polygons[i].begin(), obstacle_polygons[i].end());


    // sort 2D vector based on x-coord in ascending order
    std::sort(ordered_coords.begin(), ordered_coords.end(),
          [](const std::vector<double>& a, const std::vector<double>& b) {
                    return a[0] < b[0];
                    });
    
    /* trapezoidal cell decomposition */
    // iterate over ordered_coords and draw a vertical line through every coord
    std::vector<std::vector<double>> cell_coords; 
    std::vector<std::vector<std::vector<double>>> cells;
    std::vector<std::vector<double>> coord_backlog;
    // let's assume for now its a convex polygon
    // we leave out the first and last coord while iterating
    for (int i=1; i<ordered_coords.size(); ++i)
    {
        // find intersects with boundary of polygon
        std::vector<std::vector<double>> intersects = intersectBoundary(boundary_polygon, ordered_coords[i]);
        if (i==1)
        {
            cell_coords.push_back(ordered_coords[i-1]);
            for (int i=0; i<intersects.size(); ++i)
                cell_coords.push_back(intersects[i]);
        }
        else if (i==ordered_coords.size()-1)
        {
            int intersect_count = 0;
            for (int j=coord_backlog.size(); j>0; --j)
            {
                if (intersect_count==2)
                    break;
                else
                {
                    cell_coords.push_back(coord_backlog[i]);
                    if (coord_backlog[i][2] == -1)
                        ++intersect_count;
                }
            }
            for (int i=0; i<intersects.size(); ++i)
                cell_coords.push_back(intersects[i]);
        }
        else
        {
            int intersect_count = 0;
            for (int j=coord_backlog.size(); j>0; --j)
            {
                if (intersect_count==2)
                    break;
                else
                {
                    cell_coords.push_back(coord_backlog[i]);
                    if (coord_backlog[i][2] == -1)
                        ++intersect_count;
                }
            }
            for (int i=0; i<intersects.size(); ++i)
                cell_coords.push_back(intersects[i]);
        }
        // check if current_coord is boundary point or obstacle point
        // find up line and down line
        // if boundary coord, check if up/down line in boundary or not
        // if obstacle coord, check if up/down line in obstacle or not
        // if not in obstacle / if not outside bounds then
        // find intersect of line with line segment formed by closest coord
        // to do this, find all closest coords (behind the coord in ordered_coord)
        // for each, check if y < current_coord_y
        // if less, segment coord is behind
        // if more, segment coord is ahead
        // once intersect obtained, status of current coord is changed to open, coords behind changed to closed
        // store bounds of closed cell and index of closed cell

        coord_backlog = cell_coords;
        cells.push_back(cell_coords);
    }

    // find obstacle intersects and build adjacency graph
    std::vector<std::vector<int>> cell_adjacency_graph;
    for (int i=0; i<cells.size(); ++i)
    {
        for (int j=0; j<obstacle_polygons.size(); ++j)
        {
            // cell is clipper, obstacle is polygon
            suthHodgClip(obstacle_polygons[j], cells[i]);
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
