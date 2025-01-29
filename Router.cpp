#include "Router.h"

Router::Router() {}

void Router::LoadDataNodes(const Arguments &args,
                           //const osmscout::Distance &maxRange,
                           osmscout::GeoCoord coord)
{
    try
    {
        if (!routeReader_.IsOpen())
        {
            OpenFile(args);
        }
        NodeList_.clear();
        for (uint32_t i = 1; i <= nodeCount_; ++i)
        {
            osmscout::RouteNode node;
            node.Read(routeReader_);
            size_t validPaths = 0;
            for (const auto &path : node.paths)
            {
                if (path.IsUsable(vehicle_) || path.IsRestricted(vehicle_))
                {
                    ++validPaths;
                }
            }
            osmscout::Distance directDistance =
                osmscout::GetSphericalDistance(coord, node.GetCoord());
            if (validPaths > 0 )//&& directDistance <= maxRange)
            {
                NodeList_.push_back(node);
            }
        }
        if (routeReader_.IsOpen())
        {
            routeReader_.Close();
        }
    }
    catch (osmscout::IOException &e)
    {
        osmscout::log.Error() << "Error: " << e.GetErrorMsg();
        routeReader_.CloseFailsafe();
        throw e;
    }
}

void Router::SetupGraphFromNodes()
{
    graph_.clear();
    std::unordered_map<osmscout::Id, size_t> idToIndexMap;
    for (size_t i = 0; i < NodeList_.size(); ++i)
    {
        idToIndexMap[NodeList_[i].GetId()] = i;
    }
    graph_.reserve(NodeList_.size());
    for (const auto &routeNode : NodeList_)
    {
        Node node;
        node.point = routeNode.GetPoint();

        // for (const auto &routePath : routeNode.paths)
        // {
        //     if (!routePath.IsUsable(vehicle_) || routePath.IsRestricted(vehicle_))
        //     {
        //         continue;
        //     }
        //     Path path;
        //     path.distanceLength = routePath.distance;
        //     path.targetNodeIndex = idToIndexMap[routePath.id];
        //     path.flags = routePath.flags;
        //     path.fileRef = routeNode.objects[routePath.objectIndex].object;
        //     node.paths.push_back(path);
        // }
        graph_.push_back(node);
    }
}
void Router::OpenFile(const Arguments &args)
{
    routeReader_.Open(args.map + "router.dat", osmscout::FileScanner::Sequential,
                      true);
    routeReader_.ReadFileOffset();
    nodeCount_ = routeReader_.ReadUInt32();
    routeReader_.ReadUInt32();
    NodeList_.reserve(nodeCount_);
}
