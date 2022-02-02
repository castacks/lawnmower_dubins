#ifndef LAWN_MOWER_H
#define LAWN_MOWER_H
#include "math_utils/math_utils.h"
#include <vector>
#include "circular_curve_lib/dubins.h"
#include "ros/ros.h"

namespace lm{
class PolygonPoint {
    private:
        std::vector<std::vector<double>> up;
        std::vector<std::vector<double>> down;

        std::vector<std::vector<double>> intersectsUp;
        std::vector<std::vector<double>> intersectsDown;

        std::vector<double> circle;

    public:
        PolygonPoint(std::vector<double> point);
        std::vector<double> point;
        std::vector<double> vector;  // get vector to the region if available
        
        int type;  // in=1, out=2, none=-1
        int edgeExtension;  // up=1, down=2, both=3, none=-1

        std::vector<double> intPointUp;  // intersection in the upper extension
        std::vector<double> intPointDown;  // intersection in the lower extension

        // check for each point, the obstacles the line through that point crosses
        std::vector<std::vector<double>> getObstacleCrossings(std::vector<std::vector<double>> line);
        std::vector<double> getClosestIntersect(std::vector<double> point, std::vector<std::vector<double>> intersectsUp);
        std::vector<std::vector<double>> getCircleCrossings(std::vector<double> circle);

};


class Edge {
    private:
    public:
        std::vector<double> point1;
        std::vector<double> point2;
        std::vector<double> middle;
        int refp;
        int edge_id;

        Edge(std::vector<double> p1, std::vector<double> p2, int ref);
        double getLength(std::vector<double> p1, std::vector<double> p2);
};


class Trapezoid {
    private:
    public:
    int trapezoid_id;
    Edge* edge1;
    Edge* edge2;
    std::vector<double> center;
    std::vector<Trapezoid* > leftNeighbors;
    std::vector<Trapezoid* > rightNeighbors;
    int visited;

    std::vector<double> edge1point;
    std::vector<double> edge2point;
    Trapezoid(Edge* edge1, Edge* edge2, std::vector<double> average);
};


class LawnMower {
    private:
        std::vector<double> averagePoint(std::vector<int> nodes);
        std::vector<double> averageStartingPoint(std::vector<int> nodes);
        std::vector<double> averageEndingPoint(std::vector<int> nodes);
        std::pair<std::vector<double>, Edge*> getEdge(int i, int idx);
        std::pair<std::vector<double>, Edge*> obtainTrueStartingEdge(std::vector<int> nodes);
        std::pair<std::vector<double>, Edge*> obtainTrueEndingEdge(std::vector<int> nodes);
        std::vector<int> getInterNodes(int a, int b);
        void createTrapezoid(std::vector<int> nodes);
        void trapezoidNeighboring();
        void trapezoidListTraverse(Trapezoid*);
        void cellCoverage(Trapezoid* );
    public:
        void obtainAllPolygonPoints();
        void cellDecomposition(std::vector<std::vector<std::vector<double>>> obsList);
        void coverage();
        void findAvailablePaths(int i);
        std::vector<lm::Trapezoid*> getTrapezoidList();
        std::vector<int> getNextAvailablePaths(int i);
        std::vector<double> hitTest(std::vector<double> p1, std::vector<double> p2, std::vector<std::vector<double>> obstacle);
};

}

#endif  // LAWN_MOWER_H