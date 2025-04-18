#include "Router.h"
#include <queue>
#include <set>
#include <unordered_map>

#include <QMap>
#include <QFile>
#include <QTextStream>

Router::Router() {}

void Router::loadDataNodes(const Arguments &args,
                           const osmscout::Distance &maxRange,
                           osmscout::GeoCoord coord)
{
    try
    {
        if (!_routeReader.IsOpen())
        {
            openFile(args);
        }
        _nodeList.clear();
        for (uint32_t i = 1; i <= _nodeCount; ++i)
        {
            osmscout::RouteNode node;
            node.Read(_routeReader);
            size_t validPaths = 0;                                              /////////////
            for (const auto &path : node.paths)
            {
                if (path.IsUsable(_vehicle) || path.IsRestricted(_vehicle))
                {
                    ++validPaths;
                }
            }
            osmscout::Distance directDistance =
                osmscout::GetSphericalDistance(coord, node.GetCoord());
            if (validPaths > 0  && directDistance <= maxRange)
            {
                _nodeList.push_back(node);
            }
        }
        if (_routeReader.IsOpen())
        {
            _routeReader.Close();
        }
    }
    catch (osmscout::IOException &e)
    {
        osmscout::log.Error() << "Error: " << e.GetErrorMsg();
        _routeReader.CloseFailsafe();
        throw e;
    }
}

void Router::setupGraphFromNodes()
{
    _graph.clear();
    std::unordered_map<osmscout::Id, size_t> idToIndexMap;
    for (size_t i = 0; i < _nodeList.size(); ++i)
    {
        idToIndexMap[_nodeList[i].GetId()] = i;
    }
    _graph.reserve(_nodeList.size());
    for (const auto &routeNode : _nodeList)
    {
        Node node;
        node.point = routeNode.GetPoint();

        for (const auto &routePath : routeNode.paths)
        {
            if (!routePath.IsUsable(_vehicle) || routePath.IsRestricted(_vehicle))
            {
                continue;
            }
            Path path;
            path.distanceLength = routePath.distance;
            path.startNodeIndex = _graph.size();
            path.targetNodeIndex = idToIndexMap[routePath.id];
            //path.flags = routePath.flags;
            path.fileRef = routeNode.objects[routePath.objectIndex].object;
            int lastIndex = _pathList.size();
            _pathList.push_back(path);
            node.paths.push_back(lastIndex);
            //node.paths.push_back(path);
        }
        _graph.push_back(node);
    }

    // QFile file("allPaths.csv");
    // if (file.exists())
    //     file.remove();


}
void Router::openFile(const Arguments &args)
{
    _routeReader.Open(args.map + "router.dat", osmscout::FileScanner::Sequential,
                      true);
    _routeReader.ReadFileOffset();
    _nodeCount = _routeReader.ReadUInt32();
    _routeReader.ReadUInt32();
    _nodeList.reserve(_nodeCount);
}

void Router::generateDensities(double intervalTime)
{
    auto gauseFunction = [](unsigned int t, double rPeak1, unsigned int tPeak1, double sPeak1, double rPeak2, unsigned int tPeak2, double sPeak2)
    { return rPeak1*exp(-(((t-tPeak1)*(t-tPeak1))/(2*sPeak1*sPeak1))) +  rPeak2*exp(-(((t-tPeak2)*(t-tPeak2))/(2*sPeak2*sPeak2))); };
    //int time = 0; // in hours
    int intervalsCount = double(TIME_RANGE) / intervalTime;
    for (auto& path : _pathList)
    {
        path.densities.clear();
        if (path.distanceLength.AsMeter() < MIN_PATH_LENGTH)
        {
            for (int i = 0; i < intervalsCount; i++)
            {
                //path.densities.push_back(gauseFunction(i,90,8,2.4,110,18,3.2));
                path.densities.push_back(0.0);
            }
            continue;
        }
        for (int i = 0; i < intervalsCount; i++)
        {
            //path.densities.push_back(gauseFunction(i,90,8,2.4,110,18,3.2));
            path.densities.push_back(0.0);
        }
    }
}

std::vector<int> Router::findPathAStar(int startNodeIndex, int targetNodeIndex)
{
    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;
    std::unordered_map<int, int> previous;
    std::set<int> closedSet;

    // Initialize start node
    gScore[startNodeIndex] = 0.0;
    priorityQueue.push({startNodeIndex, 0.0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        if (closedSet.find(currentIndex) != closedSet.end())
            continue;

        closedSet.insert(currentIndex);

        // Iterate over neighbors
        for (const auto& path : _graph[currentIndex].paths)
        {
            int neighborIndex = _pathList[path].targetNodeIndex;

            if (closedSet.find(neighborIndex) != closedSet.end())
                continue;

            // Compute gScore (cost to reach this neighbor)
            double pathCost = _pathList[path].distanceLength.AsMeter() / 1000.0;
            double tentativeGScore = gScore[currentIndex] + pathCost;

            double h = _graph[neighborIndex].point.GetCoord().GetDistance(_graph[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0;

            // Check if this path to neighbor is better
            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore + h; // A* cost function

                priorityQueue.push({neighborIndex, fScore});
                previous[neighborIndex] = currentIndex;
            }
        }
    }

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    std::vector<int> path;
    for (int at = targetNodeIndex; at != startNodeIndex; at = previous[at])
    {
        path.push_back(at);
    }
    path.push_back(startNodeIndex);
    std::reverse(path.begin(), path.end());

    std::cout << "Path found: ";
    for (int node : path)
        std::cout << node << " -> ";
    return path;
}

std::vector<int> Router::findPathAStarTime(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime)
{
    int MAX_SPEED = 80;

    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;         // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous; // в паре индекс узла, индекс ребра
    std::set<int> closedSet;

    // Initialize start node
    gScore[startNodeIndex] = startTime;
    priorityQueue.push({startNodeIndex, 0.0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        if (closedSet.find(currentIndex) != closedSet.end())
            continue;

        closedSet.insert(currentIndex);

        // Iterate over neighbors
        for (const auto pathIndex : _graph[currentIndex].paths)
        {
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;

            if (closedSet.find(neighborIndex) != closedSet.end())
                continue;

            // Compute gScore (cost to reach this neighbor)
            double density = _pathList[pathIndex].densities[std::floor(gScore[currentIndex]/intervalTime)];
            auto diagramRes = trafficDiagrammFunctionTriangular(density);
            double pathCost = ((_pathList[pathIndex].distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 60; // в минутах
            double tentativeGScore = gScore[currentIndex] + pathCost;

            double h = double(_graph[neighborIndex].point.GetCoord().GetDistance(_graph[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0) / MAX_SPEED;


            // Check if this path to neighbor is better
            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore + h; // A* cost function

                priorityQueue.push({neighborIndex, fScore});
                previous[neighborIndex].first = currentIndex;
                previous[neighborIndex].second = pathIndex;
            }
        }
    }

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    std::vector<int> route;
    std::vector<std::pair<int,double>> paths;

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {

        auto curPathIndex = previous[at].second;
        _pathList[curPathIndex].densities[gScore[previous[at].first]/intervalTime] += numberOfCars /
                            (_pathList[curPathIndex].distanceLength.AsMeter()/1000);
        route.push_back(at);
        paths.push_back({curPathIndex, _pathList[curPathIndex].densities[gScore[previous[at].first]/intervalTime]});
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    std::cout << "Route found: ";

    QFile file("output.csv");

    if (file.open(QFile::WriteOnly | QFile::Append))
    {
        QTextStream out(&file);
        out << "Time,Node,Path,Density,Length\n";

        int size = route.size();
        for (int i = 0; i < size; i++)
        {
            if (i != size -1)
                out << gScore[route[i]] << "," << route[i] << "," << paths[i].first << "," << paths[i].second << "," <<
                    _pathList[paths[i].first].distanceLength.AsMeter() / 1000 << "\n";
            else
                out << gScore[route[i]] << "," << route[i] << "," << "-" << "," << "-" << ",-" << "\n";
        }

        // for (int node : route)
        // {
        //     out <<"Node: " << node << ", Time: " << gScore[node] << " -> ";
        // }
        // out << "\n";
        // for (auto path : paths)
        // {
        //     out << "Path: " << path.first << ", Density: " << path.second << " -> ";
        // }
        out << "\n\n";
    }
    return route;
}

std::vector<int> Router::findPathDijkstra(int startNodeIndex, int targetNodeIndex)
{
    using NodeCostPair = std::pair<int, double>;

    // Comparator for a min-heap based on cost
    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;  // Cost to reach each node (in km or time)
    std::unordered_map<int, int> previous;     // For path reconstruction: maps node to its predecessor
    std::set<int> closedSet;

    gScore[startNodeIndex] = 0.0;
    priorityQueue.push({startNodeIndex, 0.0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        double currentCost = priorityQueue.top().second;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        if (closedSet.find(currentIndex) != closedSet.end())
            continue;

        closedSet.insert(currentIndex);

        // Iterate over all neighbor paths from current node
        for (const auto &pathIndex : _graph[currentIndex].paths)
        {
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;
            if (closedSet.find(neighborIndex) != closedSet.end())
                continue;

            // Calculate cost to reach neighbor from current node
            double pathCost = _pathList[pathIndex].distanceLength.AsMeter() / 1000.0; // Distance in kilometers
            double tentativeGScore = currentCost + pathCost;

            // Update cost if new path is better
            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                priorityQueue.push({neighborIndex, tentativeGScore});
                previous[neighborIndex] = currentIndex;
            }
        }
    }

    // Reconstruct path from target to start
    std::vector<int> path;
    if (previous.find(targetNodeIndex) == previous.end())
        return path;

    int node = targetNodeIndex;
    while (node != startNodeIndex)
    {
        path.push_back(node);
        node = previous[node];
    }
    path.push_back(startNodeIndex);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> Router::findPathDijkstraTime(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime)
{
    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;         // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous; // в паре индекс предыдущего узла, индекс ребра
    std::set<int> closedSet;

    // Initialize start node
    gScore[startNodeIndex] = startTime;
    priorityQueue.push({startNodeIndex, 0.0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        if (closedSet.find(currentIndex) != closedSet.end())
            continue;

        closedSet.insert(currentIndex);

        // Iterate over neighbors
        for (const auto pathIndex : _graph[currentIndex].paths)
        {
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;

            if (closedSet.find(neighborIndex) != closedSet.end())
                continue;

            // Compute gScore (cost to reach this neighbor)
            double density = _pathList[pathIndex].densities[std::floor(gScore[currentIndex]/intervalTime)];
            auto diagramRes = trafficDiagrammFunctionTriangular(density);
            double pathCost = ((_pathList[pathIndex].distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 60; // в минутах
            double tentativeGScore = gScore[currentIndex] + pathCost;

            //double h = double(graph_[neighborIndex].point.GetCoord().GetDistance(graph_[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0) / MAX_SPEED;

            // Check if this path to neighbor is better
            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;

                priorityQueue.push({neighborIndex, tentativeGScore});
                previous[neighborIndex].first = currentIndex;
                previous[neighborIndex].second = pathIndex;
            }
        }
    }

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    std::vector<int> route;                     // index of node in route
    std::vector<std::pair<int,double>> paths;

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {

        auto curPathIndex = previous[at].second;
        float pathMomentTime = gScore[previous[at].first]/intervalTime;
        int indexMomentTime = std::floor(pathMomentTime);
        _pathList[curPathIndex].densities[indexMomentTime] += numberOfCars /
                                                                                        (_pathList[curPathIndex].distanceLength.AsMeter()/1000);
        route.push_back(at);
        paths.push_back({curPathIndex, _pathList[curPathIndex].densities[gScore[previous[at].first]/intervalTime]});
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    std::cout << "Route found: ";

    QFile file("output.csv");

    if (file.open(QFile::WriteOnly | QFile::Append))
    {
        QTextStream out(&file);
        out << "Time,Node,Path,Density,Length\n";

        int size = route.size();
        for (int i = 0; i < size; i++)
        {
            if (i != size -1)
                out << gScore[route[i]] << "," << route[i] << "," << paths[i].first << "," << paths[i].second << "," <<
                    _pathList[paths[i].first].distanceLength.AsMeter() / 1000 << "\n";
            else
                out << gScore[route[i]] << "," << route[i] << "," << "-" << "," << "-" << ",-" << "\n";
        }

        // for (int node : route)
        // {
        //     out <<"Node: " << node << ", Time: " << gScore[node] << " -> ";
        // }
        // out << "\n";
        // for (auto path : paths)
        // {
        //     out << "Path: " << path.first << ", Density: " << path.second << " -> ";
        // }
        out << "\n\n";
    }

    return route;
}

std::vector<int>    Router::findPathUniversal(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime, PlanningMode planningMode, Algorithm algorithm)
{
    float MAX_SPEED = 80;

    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;         // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous; // в паре индекс узла, индекс ребра
    std::set<int> closedSet;

    // Initialize start node
    gScore[startNodeIndex] = startTime;
    priorityQueue.push({startNodeIndex, 0.0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        if (closedSet.find(currentIndex) != closedSet.end())
            continue;

        closedSet.insert(currentIndex);

        // Iterate over neighbors
        for (const auto pathIndex : _graph[currentIndex].paths)
        {
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;

            if (closedSet.find(neighborIndex) != closedSet.end())
                continue;

            double h = 0, pathCost = 0;

            switch (planningMode) {
            case PlanningMode::OnlyDistance:
            {
                pathCost = _pathList[pathIndex].distanceLength.AsMeter() / 1000.0;
                h = _graph[neighborIndex].point.GetCoord().GetDistance(_graph[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0;
                break;
            }
            case PlanningMode::HistoricalData:
            case PlanningMode::DriverInfluence:
            {
                double density = _pathList[pathIndex].densities[std::floor(gScore[currentIndex]/intervalTime)];
                auto diagramRes = trafficDiagrammFunctionTriangular(density);
                pathCost = ((_pathList[pathIndex].distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 60; // в минутах
                if (algorithm == Algorithm::AStar)
                    h = (double(_graph[neighborIndex].point.GetCoord().GetDistance(_graph[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0) / MAX_SPEED)*60;
                break;
            }
            default:
                break;
            }

            // Compute gScore (cost to reach this neighbor)

            double tentativeGScore = gScore[currentIndex] + pathCost;


            // Check if this path to neighbor is better
            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore + h; // A* cost function
                _graph[neighborIndex].isVisited = true;
                priorityQueue.push({neighborIndex, fScore});
                previous[neighborIndex].first = currentIndex;
                previous[neighborIndex].second = pathIndex;
            }
        }
    }

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    int count = std::count_if(_graph.begin(), _graph.end(), [](Node elem){return elem.isVisited == true;});

    std::vector<int> route;
    std::vector<std::pair<int,double>> paths;

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {
        auto curPathIndex = previous[at].second;
        if (_pathList[curPathIndex].distanceLength.AsMeter() >= MIN_PATH_LENGTH)
        {
            _pathList[curPathIndex].densities[gScore[previous[at].first]/intervalTime] += numberOfCars /
                                                                                (_pathList[curPathIndex].distanceLength.AsMeter()/1000);
        }
        route.push_back(at);
        paths.push_back({curPathIndex, _pathList[curPathIndex].densities[gScore[previous[at].first]/intervalTime]});
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    std::cout << "Route found: ";

    QFile file("output.csv");

    if (file.open(QFile::WriteOnly | QFile::Append))
    {
        QTextStream out(&file);
        out << "Time,Node,Path,Density,Length\n";

        int size = route.size();
        for (int i = 0; i < size; i++)
        {
            if (i != size -1)
                out << gScore[route[i]] << "," << route[i] << "," << paths[i].first << "," << paths[i].second << "," <<
                    _pathList[paths[i].first].distanceLength.AsMeter() / 1000 << "\n";
            else
                out << gScore[route[i]] << "," << route[i] << "," << "-" << "," << "-" << ",-" << "\n";
        }

        // for (int node : route)
        // {
        //     out <<"Node: " << node << ", Time: " << gScore[node] << " -> ";
        // }
        // out << "\n";
        // for (auto path : paths)
        // {
        //     out << "Path: " << path.first << ", Density: " << path.second << " -> ";
        // }
        out << "\n\n";
    }

    _travelTime = gScore[route.back()] - startTime;

    return route;
}

double Router::trafficDiagrammFunctionTriangular(double p)
{
    double qc = 1600;
    double pj = 120;
    double pc = 20;
    double vf = 80;

    if (p >= 120)
    {
        _congestion = true;
        emit message("Density >= 120");
    }

    if (p <= pc) // km/h
        return vf;
    else
        return qc * ((1 - ((p - pc)/(pj - pc))) / p);
}


