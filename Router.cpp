#include "Router.h"
#include <queue>
#include <set>

Router::Router() {}

void Router::LoadDataNodes(const Arguments &args,
                           const osmscout::Distance &maxRange,
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
            size_t validPaths = 0;                                              /////////////
            for (const auto &path : node.paths)
            {
                if (path.IsUsable(vehicle_) || path.IsRestricted(vehicle_))
                {
                    ++validPaths;
                }
            }
            osmscout::Distance directDistance =
                osmscout::GetSphericalDistance(coord, node.GetCoord());
            if (validPaths > 0  && directDistance <= maxRange)
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

        for (const auto &routePath : routeNode.paths)
        {
            if (!routePath.IsUsable(vehicle_) || routePath.IsRestricted(vehicle_))
            {
                continue;
            }
            Path path;
            path.distanceLength = routePath.distance;
            path.targetNodeIndex = idToIndexMap[routePath.id];
            path.flags = routePath.flags;
            path.fileRef = routeNode.objects[routePath.objectIndex].object;
            int lastIndex = _pathList.size();
            _pathList.push_back(path);
            node.paths.push_back(lastIndex);
            //node.paths.push_back(path);
        }
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

void Router::generateDensities()
{
    auto gauseFunction = [](unsigned int t, double rPeak1, unsigned int tPeak1, double sPeak1, double rPeak2, unsigned int tPeak2, double sPeak2)
    { return rPeak1*exp(-(((t-tPeak1)*(t-tPeak1))/(2*sPeak1*sPeak1))) +  rPeak2*exp(-(((t-tPeak2)*(t-tPeak2))/(2*sPeak2*sPeak2))); };
    //int time = 0; // in hours
    for (auto& path : _pathList)
    {
        for (int i = 0; i < 24; i++)
        {
            path.densities.push_back(gauseFunction(i,90,8,2.4,110,18,3.2));
        }
    }
}

void Router::findPathAStar(int startNodeIndex, int targetNodeIndex)
{
    int FIRST_DENSITY = 0;
    int MAX_SPEED = 80;

    auto compare = [](std::pair<int,double> a, std::pair<int,double> b) {return a.second < b.second;};
    std::priority_queue<std::pair<int, double>, std::vector<std::pair<int,double>>, decltype(compare)> priorityQueue(compare);

    priorityQueue.push({startNodeIndex, 0});

    std::set<int> closedSet;

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            return;

        closedSet.insert(currentIndex);
        //auto currentNode = graph_[currentIndex];
        for (const auto& path: graph_[currentIndex].paths)
        {
            //auto& neighborNode = _pathList[path].targetNodeIndex;
            if (closedSet.find(_pathList[path].targetNodeIndex) != closedSet.end())
                continue;

            double cost = (_pathList[path].distanceLength.AsMeter() / 1000) / trafficDiagrammFunctionTriangular(_pathList[path].densities[FIRST_DENSITY]);
            double h = (graph_[currentIndex].point.GetCoord().GetDistance(graph_[currentIndex].point.GetCoord()).AsMeter() / 1000) / MAX_SPEED;
        }

    }

}

double Router::trafficDiagrammFunctionTriangular(double p)
{
    double qc = 1600;
    double pj = 120;
    double pc = 20;
    double vf = 80;

    if (p <= pc) // km/h
        return vf;
    else
        return qc * (1 - (p - pc)/(pj - pc)) / p;
}


