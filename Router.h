#ifndef ROUTER_H
#define ROUTER_H

#include <osmscout/routing/RoutingService.h>
#include <osmscout/util/Geometry.h>

#include "DrawMap.h"
#include "CustomStructures.h"

#include <QObject>



class Router : public QObject
{
    Q_OBJECT

public:
    Router();

    std::vector<Node>&  getGraph()
    {
        return _graph;
    }

    std::vector<Path>&  getPathList()
    {
        return _pathList;
    }

    void setIntervalTime(float time)
    {
        _intervalTime = time;
    }

    void                loadDataNodes(const Arguments &args, const osmscout::Distance &maxRange,
                            osmscout::GeoCoord coord);
    void                setupGraphFromNodes();
    void                generateDensities(double intervalTime);
    void                setMode(WeightType);
    void                setDatabase(osmscout::DatabaseRef database);

    Route               findPathAStar(int startNodeIndex, int targetNodeIndex, int startTime, float weight, bool densityUpdate);
    Route               findPathAStarTime(int startNodeIndex, int targetNodeIndex, int startTime, float weight, bool densityUpdate);
    Route               findPathDijkstra(int startNodeIndex, int targetNodeIndex, int startTime, bool densityUpdate);
    Route               findPathDijkstraTime(int startNodeIndex, int targetNodeIndex, int startTime, bool densityUpdate);
    Route               findPathUniversal(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime, WeightType mode, Algorithm algorithm, bool densityUpdate);
    Route               findPathBellmanFord(int startNodeIndex, int targetNodeIndex, int startTime, bool densityUpdate);
    Route               findPathBellmanFordTime(int startNodeIndex, int targetNodeIndex, int startTime, bool densityUpdate);

    float               trafficDiagrammFunctionTriangular(float density, float vf);

    float               calculateRouteCost(const std::vector<int>& route, int startTime, bool densityUpdate);

private:
    std::vector<osmscout::RouteNode>    _nodeList;
    osmscout::FileScanner               _routeReader;
    std::vector<Node>                   _graph;
    std::vector<Path>                   _pathList;
    uint32_t                            _nodeCount;
    osmscout::DatabaseRef               _database;
    unsigned int                        _maxspeed = 90;

    const int                           TIME_RANGE = 86400;  // 24 часа в секундах
    const int                           DENSITY_LIMIT = 125;
    float                               _intervalTime = 30;

};

#endif // ROUTER_H
