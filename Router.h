#ifndef ROUTER_H
#define ROUTER_H

#include <osmscout/routing/RoutingService.h>
#include <osmscout/util/Geometry.h>

#include "DrawMap.h"
#include "CustomStructures.h"


class Router
{
public:
    Router();
    std::vector<Node>&  getGraph() { return graph_; }
    std::vector<Path>&  getPathList() {return _pathList;}

    void                LoadDataNodes(const Arguments &args, const osmscout::Distance &maxRange,
                            osmscout::GeoCoord coord);
    void                SetupGraphFromNodes();
    void                OpenFile(const Arguments &args);
    void                generateDensities(double intervalTime);
    std::vector<int>    findPathAStar(int startNodeIndex, int targetNodeIndex);
    std::vector<int>    findPathAStarTime(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime);
    std::vector<int>    findPathDijkstra(int startNodeIndex, int targetNodeIndex);
    std::vector<int>    findPathDijkstraTime(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime);
    double              trafficDiagrammFunctionTriangular(double density);

private:
    std::vector<osmscout::RouteNode>    NodeList_;
    osmscout::FileScanner               routeReader_;
    osmscout::Vehicle                   vehicle_ = osmscout::vehicleCar;
    std::vector<Node>                   graph_;
    std::vector<Path>                   _pathList;
    uint32_t                            nodeCount_;
    const int                           TIME_RANGE = 1440;  // в минутах
};

#endif // ROUTER_H
