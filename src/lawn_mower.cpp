#include "lawn_mower_pattern/lawn_mower.h"

int edge_counter = 0;
int trapezoid_counter = 0;
std::vector<std::vector<std::vector<double>>> obstacle_list;
std::vector<lm::PolygonPoint*> polygonPoints;
std::vector<lm::Trapezoid*> trapezoidList;

lm::Edge::Edge(std::vector<double> p1, std::vector<double> p2, int ref)
{
    lm::Edge::point1 = p1;
    lm::Edge::point2 = p2;
    lm::Edge::middle = {(lm::Edge::point1[0] + lm::Edge::point2[0])/2, (lm::Edge::point1[1] + lm::Edge::point2[1])/2};
    lm::Edge::refp = ref;
    lm::Edge::edge_id = edge_counter++;
}

double lm::Edge::getLength(std::vector<double> p1, std::vector<double> p2)
{
    return sqrt(pow((p1[0] - p2[0]), 2) + pow((p1[1] - p2[1]), 2));
}

lm::Trapezoid::Trapezoid(lm::Edge* edge1, lm::Edge* edge2, std::vector<double> average)
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

void lm::LawnMower::trapezoidNeighboring()
{
    for (int i=0; i<trapezoidList.size(); ++i)
        for (int j=0; j<trapezoidList.size(); ++j)
        {
            if (trapezoidList[i]->edge2->refp==trapezoidList[j]->edge1->refp)
            {
                trapezoidList[i]->rightNeighbors.push_back(trapezoidList[j]);
                trapezoidList[i]->leftNeighbors.push_back(trapezoidList[i]);
            }
        }
}

void lm::LawnMower::cellDecomposition()
{
    lm::LawnMower::obtainAllPolygonPoints();
    for (int i=0; i<polygonPoints.size(); ++i)
        findAvailablePaths(i);
    lm::LawnMower::trapezoidNeighboring();
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
    std::vector<int> paths;  // stores all available edges if its "in" type. otherwise return first available one
    std::vector<double> vector = polygonPoints[i]->vector;

    // find best visible points
    for (int j=i+1; j<polygonPoints.size(); ++j)
    {
        std::vector<double> pt1 = {polygonPoints[j]->point[0], polygonPoints[j]->point[1]}; // to-point
        std::vector<double> pt2 = {polygonPoints[i]->point[0], polygonPoints[i]->point[1]}; // from-point                                                       

        std::vector<double> hit;
        for (int k=0; obstacle_list.size()-1; ++k)  // except last obstacle as it is the outer boundary
        {
            hit = lm::LawnMower::hitTest(pt1, pt2, obstacle_list[k]);
            if (hit.size() > 0)  // if hit==true, there is obstacle between i and j
                break;
        }

        if (hit.size() == 0)  // if no hit, then point visible
        {
            if (polygonPoints[i]->type==1)
                paths.push_back(j);  // it is like 6->11, 7 (they are actually determined in the next for)
            else
                return {j};  // it can go like 4->6
        }
    }

    int up = INT_MIN, down = INT_MIN;
    for (int k=0; k<paths.size(); ++k)
    {
        auto temp = polygonPoints[paths[k]]->point;
        std::vector<double> p = {temp[0] - polygonPoints[i]->point[0], temp[1] - polygonPoints[i]->point[1]};

        if ((p[0]*polygonPoints[i]->vector[1] - p[1]*polygonPoints[i]->vector[0]) > 0)  // direction is left
            if (up==INT_MIN) up = paths[k];  // store only first found one
        else  // direction is right
            if (down==INT_MIN) down = paths[k];
    }

    if (up==INT_MIN || down==INT_MIN)
        return {};
    return {up, down};
}

std::vector<double> lm::LawnMower::averagePoint(std::vector<int> nodes)
{
    std::vector<double> total = {0, 0};
    for (int i=1; i<nodes.size(); ++i)
    {
        total[0] += polygonPoints[nodes[i]]->point[0];
        total[1] += polygonPoints[nodes[i]]->point[1];
    }

    if (nodes.size() > 2)
    {
        double divisor = nodes.size()-2;
        std::transform(total.begin(), total.end(), total.begin(), [divisor](double &element){return element/divisor;});
        return total;
    }
    else
        return {};
}

std::vector<double> lm::LawnMower::averageStartingPoint(std::vector<int> nodes)
{
    if (nodes.size()>2)
        return lm::LawnMower::averagePoint(nodes);
    else
        return polygonPoints[nodes[0]]->point;
}

std::vector<double> lm::LawnMower::averageEndingPoint(std::vector<int> nodes)
{
    std::vector<double> total = {0, 0};
    if (nodes.size()>2)
        return lm::LawnMower::averagePoint(nodes);
    else
        return polygonPoints[nodes[nodes.size()-1]]->point;
}

std::pair<std::vector<double>, lm::Edge*> lm::LawnMower::getEdge(int i, int idx)
{
    std::vector<double> p = polygonPoints[i]->point;
    std::vector<double> up = polygonPoints[i]->intPointUp;
    std::vector<double> down = polygonPoints[i]->intPointDown;
    std::pair<std::vector<double>, lm::Edge*> return_pair;

    if(idx==1)  // if idx==up
    {
        lm::Edge* edge_ptr = new lm::Edge(up, p, i);
        return_pair.first = {};
        return_pair.second = edge_ptr;
        return return_pair;
    }

    if (idx==2)  // if idx==down
    {
        lm::Edge* edge_ptr = new lm::Edge(p, down, i);
        return_pair.first = {};
        return_pair.second = edge_ptr;
        return return_pair;
    }

    // the following is for idx==both
    if (up.size()>0 && down.size()==0)
    {
        lm::Edge* edge_ptr = new lm::Edge(up, p, i);
        return_pair.first = {};
        return_pair.second = edge_ptr;
        return return_pair;
    }

    else if (up.size()==0 && down.size()>0)
    {
        lm::Edge* edge_ptr = new lm::Edge(p, down, i);
        return_pair.first = {};
        return_pair.second = edge_ptr;
        return return_pair;
    }

    else if(up.size()>0 && down.size()>0)
    {
        lm::Edge* edge_ptr = new lm::Edge(up, down, i);
        return_pair.first = {};
        return_pair.second = edge_ptr;
        return return_pair;
    }

    else
    {
        lm::Edge* edge_ptr = new lm::Edge(up, down, i);
        return_pair.first = p;  // return points itself
        return_pair.second = edge_ptr;  // ignore as we want the point, not edge
        return return_pair;
    }
}

std::pair<std::vector<double>, lm::Edge*> lm::LawnMower::obtainTrueStartingEdge(std::vector<int> nodes)
{
    int i = nodes[0];
    int j = nodes[nodes.size()-1];

    if(polygonPoints[i]->type==1)
    {
        std::vector<double> averagePoint = lm::LawnMower::averageEndingPoint(nodes);
        std::vector<double> vector = polygonPoints[i]->vector;
        std::vector<double> vector2 = {averagePoint[0] - polygonPoints[i]->point[0], averagePoint[1] - polygonPoints[i]->point[1]};

        if(vector[0]*vector2[1] - vector[1]*vector2[0] < 0)
            return lm::LawnMower::getEdge(i, 1);
        else
            return lm::LawnMower::getEdge(i, 2);
    }
    else
        return lm::LawnMower::getEdge(i, 3);
}

std::pair<std::vector<double>, lm::Edge*> lm::LawnMower::obtainTrueEndingEdge(std::vector<int> nodes)
{
    int i = nodes[0];
    int j = nodes[nodes.size()-1];

    if(polygonPoints[j]->type==2)
    {
        std::vector<double> averagePoint = lm::LawnMower::averageStartingPoint(nodes);
        std::vector<double> vector = polygonPoints[j]->vector;
        std::vector<double> vector2 = {averagePoint[0] - polygonPoints[j]->point[0], averagePoint[1] - polygonPoints[j]->point[1]};

        if(vector[0]*vector2[1] - vector[1]*vector2[0] < 0)
            return lm::LawnMower::getEdge(j, 1);
        else
            return lm::LawnMower::getEdge(j, 2);
    }
    else
        return lm::LawnMower::getEdge(j, 3);
}

std::vector<int> lm::LawnMower::getInterNodes(int a, int b)
{
    std::vector<int> interNodes;
    interNodes.push_back(a);
    std::vector<int> tempPaths;
    while(polygonPoints[b]->type==-1 && polygonPoints[b]->edgeExtension!=-1)
    {
        interNodes.push_back(b);
        tempPaths = lm::LawnMower::getNextAvailablePaths(b);
        b = tempPaths[0];
    }
    interNodes.insert(interNodes.end(), tempPaths.begin(), tempPaths.end());
    return interNodes;
}

void lm::LawnMower::createTrapezoid(std::vector<int> nodes)
{
    auto edge1 = lm::LawnMower::obtainTrueStartingEdge(nodes);
    lm::Edge* e1;
    if (edge1.first.size()==0)
        e1 = edge1.second;
    auto edge2 = lm::LawnMower::obtainTrueEndingEdge(nodes);
    lm::Edge* e2;
    if (edge2.first.size()==0)
        e2 = edge2.second;
    auto average = lm::LawnMower::averagePoint(nodes);

    lm::Trapezoid* trapezoidPtr = new lm::Trapezoid(e1, e2, average);

    trapezoidList.push_back(trapezoidPtr);
}

void lm::LawnMower::findAvailablePaths(int i)
{
    auto j = getNextAvailablePaths(i);  // if next point is dead end, it returns empty vector
    if (i!=0 && (j.size()==0 || (polygonPoints[i]->type==-1)))
        ROS_ERROR("DEAD END ENCOUNTERED");
    if (j.size() == 1)
        lm::LawnMower::createTrapezoid(lm::LawnMower::getInterNodes(i, j[0]));
    else
    {
        lm::LawnMower::createTrapezoid(lm::LawnMower::getInterNodes(i, j[0]));
        lm::LawnMower::createTrapezoid(lm::LawnMower::getInterNodes(i, j[1]));
    }

}