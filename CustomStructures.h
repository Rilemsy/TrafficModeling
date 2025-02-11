#ifndef CUSTOMSTRUCTURES_H
#define CUSTOMSTRUCTURES_H

#include <osmscout/routing/RoutingService.h>

struct Path
{
    osmscout::Distance distanceLength;
    osmscout::ObjectFileRef fileRef;
    size_t targetNodeIndex;
    uint8_t flags;
    double costs;
    std::vector<double> densities;
};

struct Node
{
    osmscout::Point point;
    //std::vector<size_t> paths;
    std::vector<unsigned int> paths;
    double costs;

    bool flag = false;
    size_t previousNodeIndex = SIZE_MAX;
};

struct Way
{
    Node start;
    Node end;

};

#endif // CUSTOMSTRUCTURES_H
