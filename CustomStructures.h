#ifndef CUSTOMSTRUCTURES_H
#define CUSTOMSTRUCTURES_H

#include <osmscout/routing/RoutingService.h>

#include <qglobal.h>

struct Path
{
    osmscout::Distance distanceLength;
    osmscout::ObjectFileRef fileRef;
    unsigned int startNodeIndex;            // for paint paths indexes
    unsigned int targetNodeIndex;
    double cost;
    std::vector<float> densities;
};

struct Node
{
    osmscout::Point point;
    std::vector<unsigned int> paths;
    double cost;
    bool isVisited = false;
};

enum class PlanningMode
{
    OnlyDistance = 0,
    HistoricalData,
    DriverInfluence        // не особо важно, с historicaldata или с нулевыми плотностями
};

enum class Algorithm
{
    Dijkstra = 0,
    AStar
};

enum Option
{
    NoOptions = 0x0,
    ShowNodeNumber = 0x1,
    ShowEdgeNumber = 0x2,
    ShowTraffic = 0x4,
    ShowLastRoute = 0x8,
    ShowNodeDot = 0x10
};

Q_DECLARE_FLAGS(Options, Option)
Q_DECLARE_OPERATORS_FOR_FLAGS(Options)
#endif // CUSTOMSTRUCTURES_H
