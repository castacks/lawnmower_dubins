#include "lawn_mower_pattern/lawn_mower.h"

int edge_counter = 0;
int trapezoid_counter = 0;
std::vector<std::vector<std::vector<double>>> obstacle_list;
std::vector<lm::PolygonPoint*> polygonPoints;

lm::Edge::Edge(std::vector<double> p1, std::vector<double> p2, std::vector<double> ref)
{
    lm::Edge::point1 = p1;
    lm::Edge::point2 = p2;
    lm::Edge::middle = {(lm::Edge::point1[0] + lm::Edge::point2[0])/2, (lm::Edge::point1[1] + lm::Edge::point2[1])/2};
    lm::Edge::edge_id = edge_counter++;
}

double lm::Edge::getLength(std::vector<double> p1, std::vector<double> p2)
{
    return sqrt(pow((p1[0] - p2[0]), 2) + pow((p1[1] - p2[1]), 2));
}

lm::Trapezoid::Trapezoid(lm::Edge* edge1, lm::Edge* edge2, double average)
{
    lm::Trapezoid::trapezoid_id = trapezoid_counter++;

    lm::Trapezoid::edge1 = edge1;
    lm::Trapezoid::edge2 = edge2;
    lm::Trapezoid::edge1point = lm::Trapezoid::edge1->middle;
    lm::Trapezoid::edge2point = lm::Trapezoid::edge2->middle;

    lm::Trapezoid::visited = 0;
}

lm::PolygonPoint::PolygonPoint(std::vector<double> point)
{
    lm::PolygonPoint::point = point;
    lm::PolygonPoint::type = -1;
    lm::PolygonPoint::edgeExtension = -1;

    // represent the vertical lines through polygon points
    lm::PolygonPoint::up = {point, {point[0], DBL_MAX}};
    lm::PolygonPoint::down = {point, {point[0], DBL_MIN}};

    lm::PolygonPoint::intersectsUp = lm::PolygonPoint::getObstacleCrossings(up);
    lm::PolygonPoint::intersectsDown = lm::PolygonPoint::getObstacleCrossings(down);

    if (intersectsUp.size()%2 == 1)
    {
        lm::PolygonPoint::intPointUp = lm::PolygonPoint::getClosestIntersect(lm::PolygonPoint::point,
                                                                                lm::PolygonPoint::intersectsUp);
        lm::PolygonPoint::edgeExtension = 1;                                                                                
    }

    if (intersectsDown.size()%2 == 1)
    {
        lm::PolygonPoint::intPointDown = lm::PolygonPoint::getClosestIntersect(lm::PolygonPoint::point,
                                                                                lm::PolygonPoint::intersectsDown);
        lm::PolygonPoint::edgeExtension = 2;                                                                                
    }

    // if there are both upper and lower extensions
    if (lm::PolygonPoint::intPointUp.size() != 0 && lm::PolygonPoint::intPointDown.size() != 0)  
    {
        lm::PolygonPoint::edgeExtension = 3;
        lm::PolygonPoint::circle = {lm::PolygonPoint::point[0], lm::PolygonPoint::point[1], /*offset radius*/ 5};

        std::vector<std::vector<double>> circleIntercepts = lm::PolygonPoint::getCircleCrossings(circle);
        std::vector<double> interceptVector1 = {circleIntercepts[0][0] - lm::PolygonPoint::point[0], circleIntercepts[0][1] - lm::PolygonPoint::point[1]};
        std::vector<double> interceptVector2 = {circleIntercepts[1][0] - lm::PolygonPoint::point[0], circleIntercepts[1][1] - lm::PolygonPoint::point[1]};

        lm::PolygonPoint::vector = {interceptVector1[0] + interceptVector2[0], interceptVector1[1] + interceptVector2[1]};

        if(lm::PolygonPoint::vector[0]>0)
            lm::PolygonPoint::type = 1;
        else if(lm::PolygonPoint::vector[1]<0)
            lm::PolygonPoint::type = 2;
    }

    if(lm::PolygonPoint::intPointUp.size()==0 && lm::PolygonPoint::intPointDown.size()==0)
        lm::PolygonPoint::edgeExtension = -1;

}

void lm::LawnMower::obtainAllPolygonPoints()
{
    std::vector<std::vector<double>> ordered_coords;

    // flatten obstacle list into 2D vector
    for (int i=0; i<obstacle_list.size(); ++i)
        ordered_coords.insert(ordered_coords.end(), obstacle_list[i].begin(), obstacle_list[i].end());
    
    // sort ordered coords in ascending order of x-coords
    std::sort(ordered_coords.begin(), ordered_coords.end(),
          [](const std::vector<double>& a, const std::vector<double>& b) {
                    return a[0] < b[0];
                    });
    
    for (int i=0; i<ordered_coords.size(); ++i)
    {
        auto polyPoint = new lm::PolygonPoint(ordered_coords[i]);
        polygonPoints.push_back(polyPoint);
    }
}

void lm::LawnMower::cellDecomposition()
{
    lm::LawnMower::obtainAllPolygonPoints();
    for (int i=0; i<polygonPoints.size(); ++i)
        findAvailablePaths(i);
}

bool lm::LawnMower::check_counterclockwise(std::vector<double> pt1, std::vector<double> pt2, std::vector<double> pt3)
{
    return (pt3[1] - pt1[1]) * (pt2[0] - pt1[0]) > (pt2[1] - pt1[1]) * (pt3[0] - pt1[0]);
}

std::vector<double> lm::LawnMower::hitTest(std::vector<double> p1, std::vector<double> p2, std::vector<std::vector<double>> obstacle)
{
    std::vector<double> collision_pt;
    for (int i=0; i<obstacle.size(); ++i)
    {
        if (lm::LawnMower::check_counterclockwise(p1, obstacle[i], obstacle[(i+1)%obstacle.size()]) 
                        != lm::LawnMower::check_counterclockwise(p2, obstacle[i], obstacle[(i+1)%obstacle.size()])
            && lm::LawnMower::check_counterclockwise(p1, p2, obstacle[(i+1)%obstacle.size()]))
            {
                double m1 = (p2[1] - p1[1]) / (p2[0] - p1[0]);
                double m2 = (obstacle[(i+1)%obstacle.size()][1] - obstacle[i][1]) / (obstacle[(i+1)%obstacle.size()][0] - obstacle[i][0]);

                double xIntercept = (obstacle[i][1] - p1[1] + m1*p1[0] - m2*obstacle[i][0]) / (m1 - m2);
                double yIntercept = m2*(xIntercept-obstacle[i][0]) + obstacle[i][1];
                collision_pt.push_back(xIntercept);
                collision_pt.push_back(yIntercept);
                return collision_pt;
            }
    }
    return collision_pt;
}

std::vector<int> lm::LawnMower::getNextAvailablePaths(int i)
{
    std::vector<int> paths;
    std::vector<double> vector = polygonPoints[i]->vector;

    // find best visible points
    for (int j=i+1; j<polygonPoints.size(); ++j)
    {
        std::vector<double> pt1 = {polygonPoints[j]->point[0], polygonPoints[j]->point[1]};
        std::vector<double> pt2 = {polygonPoints[i]->point[0], polygonPoints[i]->point[1]};                                                        

        std::vector<double> hit;
        for (int k=0; obstacle_list.size()-1; ++k)
        {
            hit = lm::LawnMower::hitTest(pt1, pt2, obstacle_list[k]);
            if (hit.size() > 0)
                break;
        }

        if (hit.size() == 0)
        {
            
        }
    }
}

void lm::LawnMower::findAvailablePaths(int i)
{

}