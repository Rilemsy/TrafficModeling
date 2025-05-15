#include "Router.h"
#include <queue>
#include <set>
#include <unordered_map>
#include <chrono>

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
        _routeReader.Open(args.dbPath + "router.dat", osmscout::FileScanner::Sequential, true);
        _routeReader.ReadFileOffset();
        _nodeCount = _routeReader.ReadUInt32();
        _routeReader.ReadUInt32();
        _nodeList.reserve(_nodeCount);

        for (uint32_t i = 1; i <= _nodeCount; ++i)
        {
            osmscout::RouteNode node;
            node.Read(_routeReader);
            size_t validPaths = 0;                                              /////////////
            for (const auto &path : node.paths)
            {
                if (path.IsUsable(osmscout::vehicleCar) && !path.IsRestricted(osmscout::vehicleCar))
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
            if (!routePath.IsUsable(osmscout::vehicleCar) || routePath.IsRestricted(osmscout::vehicleCar))
            {
                continue;
            }
            Path path;
            path.distanceLength = routePath.distance;
            path.startNodeIndex = _graph.size();
            path.targetNodeIndex = idToIndexMap[routePath.id];
            //path.flags = routePath.flags;
            path.fileRef = routeNode.objects[routePath.objectIndex].object;
            if (path.fileRef.GetType()==osmscout::RefType::refWay)
            {
                osmscout::WayRef way;
                if (_database->GetWayByOffset(path.fileRef.GetFileOffset(), way))
                {
                    auto features = way->GetFeatureValueBuffer();
                    for (const auto& featureInstance: features.GetType()->GetFeatures())
                    {
                        if (features.HasFeature(featureInstance.GetIndex()))
                        {
                            osmscout::FeatureRef feature=featureInstance.GetFeature();
                            if (feature->HasValue())
                            {
                                osmscout::FeatureValue *value=features.GetValue(featureInstance.GetIndex());
                                const auto *lanes = dynamic_cast<const osmscout::LanesFeatureValue*>(value);
                                if (lanes!=nullptr)
                                {
                                    path.lanes = (short int)lanes->GetLanes();
                                }
                                const auto *maxSpeed = dynamic_cast<const osmscout::MaxSpeedFeatureValue*>(value);
                                if (maxSpeed!=nullptr)
                                {
                                    path.maxSpeed = (int)maxSpeed->GetMaxSpeed();
                                    if (path.maxSpeed > _maxspeed)
                                        _maxspeed = path.maxSpeed;
                                }
                            }
                        }
                    }
                }
            }


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

void Router::generateDensities(double intervalTime)
{
    int intervalsCount = double(TIME_RANGE) / intervalTime;
    for (auto& path : _pathList)
    {
        path.densities.clear();
        for (int i = 0; i < intervalsCount; i++)
        {
            path.densities.push_back(0.0);
        }
    }
}

void Router::setDatabase(osmscout::DatabaseRef database)
{
    _database = database;
}

Route Router::findPathAStar(int startNodeIndex, int targetNodeIndex, int startTime, float weight, bool densityUpdate)
{
    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;                 // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous;   // в паре индекс узла, индекс ребра

    unsigned int visitedNodesCounter = 0;

    gScore[startNodeIndex] = 0;
    priorityQueue.push({startNodeIndex, 0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        for (const auto pathIndex : _graph[currentIndex].paths)
        {
            Path path = _pathList[pathIndex];
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;

            double h = 0, pathCost = 0;

            pathCost = path.distanceLength.AsMeter() / 1000.0;
            h = (_graph[neighborIndex].point.GetCoord().GetDistance(_graph[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0) * weight;

            double tentativeGScore = gScore[currentIndex] + pathCost;

            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore + h;
                visitedNodesCounter++;
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

    std::vector<int> route; // индекс узла
    std::vector<int> paths; // индекс ребра

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {
        auto curPathIndex = previous[at].second;
        route.push_back(at);
        paths.push_back(curPathIndex);
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    auto travelTime = calculateRouteCost(paths, startTime, densityUpdate);
    float execTime = 0;

    return {route, travelTime, execTime, visitedNodesCounter};
}

Route Router::findPathAStarTime(int startNodeIndex, int targetNodeIndex, int startTime, float weight, bool densityUpdate)
{
    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;                 // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous;   // в паре индекс узла, индекс ребра
    unsigned int visitedNodesCounter = 0;

    gScore[startNodeIndex] = startTime;
    priorityQueue.push({startNodeIndex, startTime});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        for (const auto pathIndex : _graph[currentIndex].paths)
        {
            Path path = _pathList[pathIndex];
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;

            double h = 0, pathCost = 0, density = 0;

            density = path.densities[std::floor(gScore[currentIndex]/_intervalTime)];
            auto diagramRes = trafficDiagrammFunctionTriangular(density, path.maxSpeed);
            pathCost = ((_pathList[pathIndex].distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600; // в секундах
            h = (double(_graph[neighborIndex].point.GetCoord().GetDistance(_graph[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0) / _maxspeed) * 3600 * weight;

            double tentativeGScore = gScore[currentIndex] + pathCost;

            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore + h;
                visitedNodesCounter++;
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

    std::vector<int> route; // индекс узла
    std::vector<int> paths; // индекс ребра

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {
        auto curPathIndex = previous[at].second;
        if (densityUpdate)
        {
            _pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] += float(numberOfCars) /
                ((_pathList[curPathIndex].distanceLength.AsMeter()/1000)*_pathList[curPathIndex].lanes);
            if (_pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] > DENSITY_LIMIT)
                _pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] = DENSITY_LIMIT;
        }
        route.push_back(at);
        paths.push_back(curPathIndex);
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    float routeCost = gScore[route.back()];
    auto travelTime = routeCost - startTime;
    float execTime = 0;

    return {route, travelTime, execTime, visitedNodesCounter};
}

Route Router::findPathDijkstra(int startNodeIndex, int targetNodeIndex, int startTime, bool densityUpdate)
{
    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;                 // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous;   // в паре индекс узла, индекс ребра
    unsigned int visitedNodesCounter = 0;

    gScore[startNodeIndex] = 0;
    priorityQueue.push({startNodeIndex, 0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        for (const auto pathIndex : _graph[currentIndex].paths)
        {
            Path path = _pathList[pathIndex];
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;

            double pathCost = 0, density = 0;

            pathCost = path.distanceLength.AsMeter() / 1000.0;
            double tentativeGScore = gScore[currentIndex] + pathCost;

            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore;
                visitedNodesCounter++;
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

    std::vector<int> route; // индекс узла
    std::vector<int> paths; // индекс ребра

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {
        auto curPathIndex = previous[at].second;
        route.push_back(at);
        paths.push_back(curPathIndex);
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    auto travelTime = calculateRouteCost(paths, startTime, densityUpdate);

    float execTime = 0;

    return {route, travelTime, execTime, visitedNodesCounter};
}

Route Router::findPathDijkstraTime(int startNodeIndex, int targetNodeIndex, int startTime, bool densityUpdate)
{
    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;                 // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous;   // в паре индекс узла, индекс ребра
    unsigned int visitedNodesCounter = 0;

    gScore[startNodeIndex] = startTime;
    priorityQueue.push({startNodeIndex, startTime});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        for (const auto pathIndex : _graph[currentIndex].paths)
        {
            Path path = _pathList[pathIndex];
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;

            double pathCost = 0, density = 0;

            density = path.densities[std::floor(gScore[currentIndex]/_intervalTime)];
            auto diagramRes = trafficDiagrammFunctionTriangular(density, path.maxSpeed);
            pathCost = ((_pathList[pathIndex].distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600; // в секундах

            double tentativeGScore = gScore[currentIndex] + pathCost;

            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore;
                visitedNodesCounter++;
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

    std::vector<int> route; // индекс узла
    std::vector<int> paths; // индекс ребра

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {
        auto curPathIndex = previous[at].second;
        if(densityUpdate)
        {
            _pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] += float(numberOfCars) /
                ((_pathList[curPathIndex].distanceLength.AsMeter()/1000)*_pathList[curPathIndex].lanes);
            if (_pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] > DENSITY_LIMIT)
                _pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] = DENSITY_LIMIT;
        }
        route.push_back(at);
        paths.push_back(curPathIndex);
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    float routeCost = gScore[route.back()];
    auto travelTime = routeCost - startTime;
    float execTime = 0;

    return {route, travelTime, execTime, visitedNodesCounter};
}

Route Router::findPathUniversal(int startNodeIndex, int targetNodeIndex, int startTime, int intervalTime, WeightType planningMode, Algorithm algorithm, bool densityUpdate)
{
    auto startMeasure = std::chrono::steady_clock::now();

    using NodeCostPair = std::pair<int, double>;

    auto compare = [](const NodeCostPair& a, const NodeCostPair& b) {
        return a.second > b.second;
    };

    std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;                 // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous;   // в паре индекс узла, индекс ребра
    unsigned int visitedNodesCounter = 0;

    switch(planningMode)
    {
    case WeightType::Distance:
    {
        gScore[startNodeIndex] = 0;
        priorityQueue.push({startNodeIndex, 0});
        break;
    }
    case WeightType::Time:
    {
        gScore[startNodeIndex] = startTime;
        priorityQueue.push({startNodeIndex, startTime});
        break;
    }
    }

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        for (const auto pathIndex : _graph[currentIndex].paths)
        {
            Path path = _pathList[pathIndex];
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;

            double h = 0, pathCost = 0, density = 0;

            switch (planningMode)
            {
            case WeightType::Distance:
            {
                pathCost = path.distanceLength.AsMeter() / 1000.0;
                h = _graph[neighborIndex].point.GetCoord().GetDistance(_graph[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0;
                break;
            }
            case WeightType::Time:
            {
                density = path.densities[std::floor(gScore[currentIndex]/intervalTime)];
                auto diagramRes = trafficDiagrammFunctionTriangular(density, path.maxSpeed);
                pathCost = ((_pathList[pathIndex].distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600; // в секундах
                if (algorithm == Algorithm::AStar)
                    h = (double(_graph[neighborIndex].point.GetCoord().GetDistance(_graph[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0) / _maxspeed)*3600;
                break;
            }
            default:
                break;
            }

            double tentativeGScore = gScore[currentIndex] + pathCost;

            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore + h;
                visitedNodesCounter++;
                priorityQueue.push({neighborIndex, fScore});
                previous[neighborIndex].first = currentIndex;
                previous[neighborIndex].second = pathIndex;
            }
        }
    }

    auto finishMeasure = std::chrono::steady_clock::now();
    auto elapsed =std::chrono::duration_cast<std::chrono::nanoseconds>(finishMeasure - startMeasure);
    std::cout << "Elapsed: " << elapsed.count() * 1e-9 << " s" << std::endl;

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    int count = std::count_if(_graph.begin(), _graph.end(), [](Node elem){return elem.isVisited == true;});

    std::vector<int> route; // индекс узла
    std::vector<int> paths; // индекс ребра

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {
        auto curPathIndex = previous[at].second;
        if (densityUpdate)/*(_pathList[curPathIndex].distanceLength.AsMeter() >= MIN_PATH_LENGTH)*/
        {
            _pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/intervalTime)] += float(numberOfCars) /
                ((_pathList[curPathIndex].distanceLength.AsMeter()/1000)*_pathList[curPathIndex].lanes);
            if (_pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/intervalTime)] > DENSITY_LIMIT)
                _pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/intervalTime)] = DENSITY_LIMIT;
        }
        route.push_back(at);
        paths.push_back(curPathIndex);
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    float travelTime = 0;

    switch(planningMode)
    {
    case WeightType::Distance:
    {
        travelTime = calculateRouteCost(paths, startTime, true);
        break;
    }
    case WeightType::Time:
    {
        float routeCost = gScore[route.back()];
        travelTime = routeCost - startTime;
        break;
    }
    }


    float execTime = 0;

    return {route, travelTime, execTime, visitedNodesCounter};
}

Route Router::findPathBellmanFord(int startNodeIndex, int targetNodeIndex, int startTime, bool densityUpdate)
{
    std::unordered_map<int, double> gScore;         // стоимость
    std::unordered_map<int, std::pair<int,int>> previous; // в паре индекс узла, индекс ребра

    int graphSize = _graph.size();
    gScore[startNodeIndex] = 0;

    for (int i = 0; i < graphSize - 1; i++)
    {
        int pathIndex = 0;
        for (Path path : _pathList)
        {
            float pathCost = 0;

            pathCost = path.distanceLength.AsMeter() / 1000.0;

            if (gScore.find(path.startNodeIndex) != gScore.end() && (gScore.find(path.targetNodeIndex) == gScore.end() ||
                gScore[path.startNodeIndex] + pathCost < gScore[path.targetNodeIndex]))
            {
                gScore[path.targetNodeIndex] = gScore[path.startNodeIndex] + pathCost;
                previous[path.targetNodeIndex].first = path.startNodeIndex;
                previous[path.targetNodeIndex].second = pathIndex;

            }
            pathIndex++;
        }
    }

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    int count = std::count_if(_graph.begin(), _graph.end(), [](Node elem){return elem.isVisited == true;});

    std::vector<int> route; // индекс узла
    std::vector<int> paths; // индекс ребра

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {
        auto curPathIndex = previous[at].second;
        route.push_back(at);
        paths.push_back(curPathIndex);
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    auto travelTime = calculateRouteCost(paths, startTime, densityUpdate);

    float execTime = 0;

    return {route, travelTime, execTime};
}

Route Router::findPathBellmanFordTime(int startNodeIndex, int targetNodeIndex, int startTime, bool densityUpdate)
{
    std::unordered_map<int, double> gScore;         // стоимость
    std::unordered_map<int, std::pair<int,int>> previous; // в паре индекс узла, индекс ребра

    int graphSize = _graph.size();
    gScore[startNodeIndex] = startTime;

    for (int i = 0; i < graphSize - 1; i++)
    {
        int pathIndex = 0;
        for (Path path : _pathList)
        {
            float pathCost = 0;
            float density = 0;

            density = path.densities[std::floor(gScore[path.startNodeIndex]/_intervalTime)];
            auto diagramRes = trafficDiagrammFunctionTriangular(density, path.maxSpeed);
            pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600; // в секундах
            break;

            if (gScore.find(path.startNodeIndex) != gScore.end() && (gScore.find(path.targetNodeIndex) == gScore.end() ||
                                                                     gScore[path.startNodeIndex] + pathCost < gScore[path.targetNodeIndex]))
            {
                gScore[path.targetNodeIndex] = gScore[path.startNodeIndex] + pathCost;
                previous[path.targetNodeIndex].first = path.startNodeIndex;
                previous[path.targetNodeIndex].second = pathIndex;

            }
            pathIndex++;
        }
    }

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    int count = std::count_if(_graph.begin(), _graph.end(), [](Node elem){return elem.isVisited == true;});

    std::vector<int> route; // индекс узла
    std::vector<int> paths; // индекс ребра

    int numberOfCars = 1;
    for (int at = targetNodeIndex; at != startNodeIndex; )
    {
        auto curPathIndex = previous[at].second;
        if (densityUpdate)
        {
            _pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] += float(numberOfCars) /                                                                                                       ((_pathList[curPathIndex].distanceLength.AsMeter()/1000)*_pathList[curPathIndex].lanes);
            if (_pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] > DENSITY_LIMIT)
                _pathList[curPathIndex].densities[std::floor(gScore[previous[at].first]/_intervalTime)] = DENSITY_LIMIT;
        }
        route.push_back(at);
        paths.push_back(curPathIndex);
        at = previous[at].first;
    }
    route.push_back(startNodeIndex);
    std::reverse(route.begin(), route.end());
    std::reverse(paths.begin(), paths.end());

    float routeCost = gScore[route.back()];
    auto travelTime = routeCost - startTime;

    float execTime = 0;

    return {route, travelTime, execTime};
}

float Router::trafficDiagrammFunctionTriangular(float p, float vf)
{
    float pj = 134;
    float pc = 24;
    float qc = vf*pc;

    if (p > DENSITY_LIMIT)
    {
        p = DENSITY_LIMIT;
    }

    if (p <= pc) // km/h
        return vf;
    else
        return qc * ((1 - ((p - pc)/(pj - pc))) / p);
}

float Router::calculateRouteCost(const std::vector<int>& route, int startTime, bool densityUpdate)
{
    float travelTime = 0;
    for (int pathIndex : route)
    {
        Path path = _pathList[pathIndex];
        float density = path.densities[std::floor((travelTime+startTime)/_intervalTime)];
        auto diagramRes = trafficDiagrammFunctionTriangular(density, path.maxSpeed);
        float pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600; // в секундах
        if (densityUpdate)/*(_pathList[curPathIndex].distanceLength.AsMeter() >= MIN_PATH_LENGTH)*/
        {
            _pathList[pathIndex].densities[std::floor((travelTime+startTime)/_intervalTime)] += float(1) /
                ((_pathList[pathIndex].distanceLength.AsMeter()/1000)*_pathList[pathIndex].lanes);
            if (_pathList[pathIndex].densities[std::floor((travelTime+startTime)/_intervalTime)] > DENSITY_LIMIT)
                _pathList[pathIndex].densities[std::floor((travelTime+startTime)/_intervalTime)] = DENSITY_LIMIT;
        }

        travelTime += pathCost;

    }
    return travelTime;
}
