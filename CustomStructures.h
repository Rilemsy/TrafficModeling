#ifndef CUSTOMSTRUCTURES_H
#define CUSTOMSTRUCTURES_H

#include <osmscout/routing/RoutingService.h>

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

#endif // CUSTOMSTRUCTURES_H
