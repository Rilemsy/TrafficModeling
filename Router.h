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
    void                setMode(PlanningMode);
    void                setDatabase(osmscout::DatabaseRef database);

    Route               findPathAStar(int startNodeIndex, int targetNodeIndex, int startTime, float weight);
    Route               findPathAStarTime(int startNodeIndex, int targetNodeIndex, int startTime, float weight);
    Route               findPathDijkstra(int startNodeIndex, int targetNodeIndex, int startTime);
    Route               findPathDijkstraTime(int startNodeIndex, int targetNodeIndex, int startTime);
    Route               findPathUniversal(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime, PlanningMode mode, Algorithm algorithm, bool densityUpdate);
    Route               findPathBellmanFord(int startNodeIndex, int targetNodeIndex, int startTime);
    Route               findPathBellmanFordTime(int startNodeIndex, int targetNodeIndex, int startTime);

    float               trafficDiagrammFunctionTriangular(float density, float vf, short int lanes);

    float               calculateRouteCost(const std::vector<int>& route, int startTime, bool densityUpdate);


signals:
    void message(QString str);

private:
    std::vector<osmscout::RouteNode>    _nodeList;
    osmscout::FileScanner               _routeReader;
    std::vector<Node>                   _graph;
    std::vector<Path>                   _pathList;
    uint32_t                            _nodeCount;
    bool                                _congestion = false;
    osmscout::DatabaseRef               _database;

    const int                           TIME_RANGE = 86400;  // 24 часа в секундах
    const int                           DENSITY_LIMIT = 125;
    float                               _intervalTime = 30;

};

#endif // ROUTER_H
