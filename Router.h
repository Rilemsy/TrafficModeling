#ifndef ROUTER_H
#define ROUTER_H

#include <osmscout/routing/RoutingService.h>
#include <osmscout/util/Geometry.h>

#include "Map.h"
#include "CustomStructures.h"

#include <QObject>



class Router : public QObject
{
    Q_OBJECT

public:
    Router();

    std::vector<Node>&  getNodeList()
    {
        return _nodeList;
    }

    std::vector<Path>&  getPathList()
    {
        return _pathList;
    }

    void setIntervalTime(float time)
    {
        _intervalTime = time;
    }

    void                loadNodesData(const Arguments &args, const osmscout::Distance &maxRange,
                            osmscout::GeoCoord coord);
    void                buildGraph();
    void                initDensities(double intervalTime);
    void                setDatabase(osmscout::DatabaseRef database);

    Route               findPath(int startNodeIndex, int endNodeIndex, int startTime, float weight, bool withLoad, bool densityUpdate, Algorithm algorithm);
    Route               findPathAStar(int startNodeIndex, int targetNodeIndex, int startTime, float weight, bool withLoad, bool densityUpdate);
    Route               findPathDijkstra(int startNodeIndex, int targetNodeIndex, int startTime, bool withLoad, bool densityUpdate);
    Route               findPathBellmanFord(int startNodeIndex, int targetNodeIndex, int startTime, bool withLoad, bool densityUpdate);

    void                addRoutes(unsigned int numOfCars, float weight, bool withLoad, bool densityUpdate, Algorithm algorithm);
    float               trafficDiagramFunction(float density, float vf);
    float               calculateRouteCost(const std::vector<int>& route, int startTime, bool densityUpdate);

private:
    std::vector<osmscout::RouteNode>    _routeNodeList;

    std::vector<Node>                   _nodeList;
    std::vector<Path>                   _pathList;
    uint32_t                            _nodeCount;
    osmscout::DatabaseRef               _database;
    unsigned int                        _maxspeed = 90;

    const int                           TIME_RANGE = 43200;  // в секундах
    const int                           DENSITY_LIMIT = 120;
    float                               _intervalTime = 30;

};

#endif // ROUTER_H
