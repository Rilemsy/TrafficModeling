#ifndef CUSTOMSTRUCTURES_H
#define CUSTOMSTRUCTURES_H

#include <osmscout/routing/RoutingService.h>

#include <qglobal.h>

struct Path
{
    osmscout::Distance distanceLength;
    unsigned int startNodeIndex;
    unsigned int targetNodeIndex;
    float maxSpeed = 90;
    short int lanes = 1;
    std::vector<float> densities;
};

struct Node
{
    osmscout::Point point;
    std::vector<unsigned int> paths;
    bool isVisited = false;
};

struct Route
{
    std::vector<int> constructedRoute;
    float cost = 0;
    double execTime = 0;
    unsigned int visitedNodeCount = 0;
};

enum class WeightType
{
    Distance = 0,
    Time
};

enum class Algorithm
{
    Dijkstra = 0,
    AStar,
    WeightedAStar,
    BellManFord
};

#endif // CUSTOMSTRUCTURES_H
