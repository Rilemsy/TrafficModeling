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
    double getTravelTime()
    {
        auto value = _travelTime;
        _travelTime = 0;
        return value;
    }
    bool getCongestion()
    {
        bool value= _congestion;
        _congestion = false;
        return value;
    }

    void                loadDataNodes(const Arguments &args, const osmscout::Distance &maxRange,
                            osmscout::GeoCoord coord);
    void                setupGraphFromNodes();
    void                openFile(const Arguments &args);
    void                generateDensities(double intervalTime);
    std::vector<int>    findPathAStar(int startNodeIndex, int targetNodeIndex);
    std::vector<int>    findPathAStarTime(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime);
    std::vector<int>    findPathDijkstra(int startNodeIndex, int targetNodeIndex);
    std::vector<int>    findPathDijkstraTime(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime);
    std::vector<int>    findPathUniversal(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime, PlanningMode mode, Algorithm algorithm);

    double              trafficDiagrammFunctionTriangular(double density);

signals:
    void message(QString str);

private:
    std::vector<osmscout::RouteNode>    _nodeList;
    osmscout::FileScanner               _routeReader;
    osmscout::Vehicle                   _vehicle = osmscout::vehicleCar;
    std::vector<Node>                   _graph;
    std::vector<Path>                   _pathList;
    uint32_t                            _nodeCount;
    const int                           TIME_RANGE = 1440;  // 24 часа в минутах
    PlanningMode                        _planningMode = PlanningMode::DriverInfluence;
    const double                        MIN_PATH_LENGTH = 100;
    double                              _travelTime = 0;
    bool                                _congestion = false;
};

#endif // ROUTER_H
