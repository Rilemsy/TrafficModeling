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
    std::vector<Node> &getGraph() { return graph_; }

    void LoadDataNodes(const Arguments &args, const osmscout::Distance &maxRange,
                       osmscout::GeoCoord coord);
    void SetupGraphFromNodes();
    void OpenFile(const Arguments &args);
private:
    std::vector<osmscout::RouteNode> NodeList_;
    osmscout::FileScanner routeReader_;
    osmscout::Vehicle vehicle_ = osmscout::vehicleCar;
    std::vector<Node> graph_;
    uint32_t nodeCount_;
};

#endif // ROUTER_H
