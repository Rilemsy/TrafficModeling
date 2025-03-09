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

#endif // CUSTOMSTRUCTURES_H
