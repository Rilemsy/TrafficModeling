#include "Router.h"
#include <queue>
#include <set>
#include <unordered_map>
#include <chrono>

#include <QMap>
#include <QFile>
#include <QTextStream>
#include <QRandomGenerator>

Router::Router() {}

void Router::loadNodesData(const Arguments &args, const osmscout::Distance &maxRange, osmscout::GeoCoord coord)
{
    osmscout::FileScanner routeReader;
    routeReader.Open(args.dbPath + "router.dat", osmscout::FileScanner::Sequential, true);
    routeReader.ReadFileOffset();
    _nodeCount = routeReader.ReadUInt32();
    routeReader.ReadUInt32();
    _routeNodeList.reserve(_nodeCount);

    for (uint32_t i = 1; i <= _nodeCount; ++i)
    {
        osmscout::RouteNode node;
        node.Read(routeReader);
        size_t validPaths = 0;
        for (const auto &path : node.paths)
        {
            if (path.IsUsable(osmscout::vehicleCar) && !path.IsRestricted(osmscout::vehicleCar) && path.distance.AsMeter() > 0)
            {
                ++validPaths;
            }
        }
        osmscout::Distance directDistance = osmscout::GetSphericalDistance(coord, node.GetCoord());
        if (validPaths > 0  && directDistance <= maxRange)
        {
            _routeNodeList.push_back(node);
        }
    }
    if (routeReader.IsOpen())
    {
        routeReader.Close();
    }
}

void Router::buildGraph()
{
    _nodeList.clear();
    std::unordered_map<osmscout::Id, size_t> idToIndexMap;
    for (size_t i = 0; i < _routeNodeList.size(); ++i)
    {
        idToIndexMap[_routeNodeList[i].GetId()] = i;
    }
    _nodeList.reserve(_routeNodeList.size());

    for (const auto& routeNode : _routeNodeList)
    {
        Node node;
        node.point = routeNode.GetPoint();

        for (const auto& routePath : routeNode.paths)
        {
            if (!routePath.IsUsable(osmscout::vehicleCar) || routePath.IsRestricted(osmscout::vehicleCar) || routePath.distance.AsMeter() <= 0)
            {
                continue;
            }
            Path path;
            path.distanceLength = routePath.distance;
            path.startNodeIndex = _nodeList.size();
            if (idToIndexMap.find(routePath.id) != idToIndexMap.end())
                path.targetNodeIndex = idToIndexMap[routePath.id];
            else
                continue;

            osmscout::ObjectFileRef pathFileRef = routeNode.objects[routePath.objectIndex].object;
            if (pathFileRef.GetType() == osmscout::RefType::refWay)
            {
                osmscout::WayRef way;
                if (_database->GetWayByOffset(pathFileRef.GetFileOffset(), way))
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
                                    short int lanesValue = (short int)lanes->GetLanes();
                                    if (lanesValue > 0)
                                        path.lanes = lanesValue;
                                }
                                const auto *maxSpeed = dynamic_cast<const osmscout::MaxSpeedFeatureValue*>(value);
                                if (maxSpeed!=nullptr)
                                {
                                    int maxSpeedValue = (int)maxSpeed->GetMaxSpeed();
                                    if (maxSpeedValue > 0)
                                    {
                                        path.maxSpeed = maxSpeedValue;
                                        if (path.maxSpeed > _maxspeed)
                                            _maxspeed = path.maxSpeed;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            int lastIndex = _pathList.size();
            _pathList.push_back(path);
            node.paths.push_back(lastIndex);
        }
        _nodeList.push_back(node);
    }
}

void Router::initDensities(double intervalTime)
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


Route Router::findPathAStar(int startNodeIndex, int targetNodeIndex, float startTime, float weight, bool withLoad, bool densityUpdate)
{
    auto startMeasure = std::chrono::steady_clock::now();

    auto compare = [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second > b.second;
    };

    std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;                 // цена по времени
    std::unordered_map<int, std::pair<int,int>> previous;   // в паре индекс узла, индекс ребра

    gScore[startNodeIndex] = startTime;
    priorityQueue.push({startNodeIndex, 0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        for (const auto& pathIndex : _nodeList[currentIndex].paths)
        {
            Path path = _pathList[pathIndex];
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;
            _nodeList[neighborIndex].isVisited = true;

            double h = 0, pathCost = 0, density = 0;

            if (withLoad)
            {
                density = path.densities[std::floor(gScore[currentIndex]/_intervalTime)];
                auto diagramRes = trafficDiagramFunction(density/((path.distanceLength.AsMeter()/1000)*path.lanes), path.maxSpeed);
                pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600; // в секундах
            }
            else
            {
                pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (path.maxSpeed)) * 3600;
            }

            h = (double(_nodeList[neighborIndex].point.GetCoord().GetDistance(_nodeList[targetNodeIndex].point.GetCoord()).AsMeter() / 1000.0) / _maxspeed) * 3600 * weight;

            double tentativeGScore = gScore[currentIndex] + pathCost;

            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore + h;
                priorityQueue.push({neighborIndex, fScore});
                previous[neighborIndex].first = currentIndex;
                previous[neighborIndex].second = pathIndex;
            }
        }
    }

    auto finishMeasure = std::chrono::steady_clock::now();
    auto elapsed =std::chrono::duration_cast<std::chrono::nanoseconds>(finishMeasure - startMeasure);

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    unsigned visitedCount = std::count_if(_nodeList.begin(), _nodeList.end(), [](Node& elem){
        if (elem.isVisited){
            elem.isVisited = false;
            return true;
        }
        return false;
    });


    std::vector<int> route; // индекс узла
    std::vector<int> paths; // индекс ребра

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

    return {route, startTime, travelTime, elapsed.count() * 1e-9, visitedCount};
}


Route Router::findPathDijkstra(int startNodeIndex, int targetNodeIndex, float startTime, bool withLoad, bool densityUpdate)
{
    auto startMeasure = std::chrono::steady_clock::now();

    auto compare = [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second > b.second;
    };

    std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, decltype(compare)> priorityQueue(compare);
    std::unordered_map<int, double> gScore;
    std::unordered_map<int, std::pair<int,int>> previous;

    gScore[startNodeIndex] = startTime;
    priorityQueue.push({startNodeIndex, 0});

    while (!priorityQueue.empty())
    {
        int currentIndex = priorityQueue.top().first;
        priorityQueue.pop();

        if (currentIndex == targetNodeIndex)
            break;

        for (const auto& pathIndex : _nodeList[currentIndex].paths)
        {
            Path path = _pathList[pathIndex];
            int neighborIndex = _pathList[pathIndex].targetNodeIndex;
            _nodeList[neighborIndex].isVisited = true;

            double pathCost = 0, density = 0;

            if (withLoad)
            {
                density = path.densities[std::floor((gScore[currentIndex])/_intervalTime)];
                auto diagramRes = trafficDiagramFunction(density/((path.distanceLength.AsMeter()/1000)*path.lanes), path.maxSpeed);
                pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600;
            }
            else
            {
                pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (path.maxSpeed)) * 3600;
            }

            double tentativeGScore = gScore[currentIndex] + pathCost;

            if (gScore.find(neighborIndex) == gScore.end() || tentativeGScore < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeGScore;
                double fScore = tentativeGScore;
                priorityQueue.push({neighborIndex, fScore});
                previous[neighborIndex].first = currentIndex;
                previous[neighborIndex].second = pathIndex;
            }
        }
    }

    auto finishMeasure = std::chrono::steady_clock::now();
    auto elapsed =std::chrono::duration_cast<std::chrono::nanoseconds>(finishMeasure - startMeasure);

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    unsigned visitedCount = std::count_if(_nodeList.begin(), _nodeList.end(), [](Node& elem){
        if (elem.isVisited){
            elem.isVisited = false;
            return true;
        }
        return false;
    });


    std::vector<int> route;
    std::vector<int> paths;

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

    return {route, startTime, travelTime, elapsed.count() * 1e-9, visitedCount};
}

Route Router::findPathBellmanFord(int startNodeIndex, int targetNodeIndex, float startTime, bool withLoad, bool densityUpdate)
{
    auto startMeasure = std::chrono::steady_clock::now();

    std::vector<float> gScore(_nodeList.size(), std::numeric_limits<float>::max()); // стоимость
    std::unordered_map<int, std::pair<int,int>> previous; // в паре индекс узла, индекс ребра

    int graphSize = _nodeList.size();
    gScore[startNodeIndex] = startTime;
    int pathIndex = 0;

    for (int i = 0; i < graphSize - 1; ++i)
    {
        pathIndex = 0;
        for (const Path& path : _pathList)
        {
            _nodeList[path.startNodeIndex].isVisited = true;
            _nodeList[path.targetNodeIndex].isVisited = true;

            float pathCost = 0;

            if (withLoad)
            {
                float density = path.densities[std::floor(gScore[path.startNodeIndex]/_intervalTime)];
                auto diagramRes = trafficDiagramFunction(density/((path.distanceLength.AsMeter()/1000)*path.lanes), path.maxSpeed);
                pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600; // в секундах
            }
            else
            {
                pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (path.maxSpeed)) * 3600;
            }

            if (gScore[path.startNodeIndex] != std::numeric_limits<float>::max() && gScore[path.startNodeIndex] + pathCost < gScore[path.targetNodeIndex])
            {
                gScore[path.targetNodeIndex] = gScore[path.startNodeIndex] + pathCost;
                previous[path.targetNodeIndex].first = path.startNodeIndex;
                previous[path.targetNodeIndex].second = pathIndex;

            }
            pathIndex++;
        }
    }

    auto finishMeasure = std::chrono::steady_clock::now();
    auto elapsed =std::chrono::duration_cast<std::chrono::nanoseconds>(finishMeasure - startMeasure);

    if (previous.find(targetNodeIndex) == previous.end())
    {
        std::cout << "No path found!" << std::endl;
        return {};
    }

    unsigned visitedCount = std::count_if(_nodeList.begin(), _nodeList.end(), [](Node& elem){
        if (elem.isVisited){
            elem.isVisited = false;
            return true;
        }
        return false;
    });

    std::vector<int> route;
    std::vector<int> paths;

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

    return {route, startTime, travelTime, elapsed.count() * 1e-9, visitedCount};
}

float Router::trafficDiagramFunction(float p, float vf)
{
    if (p > DENSITY_LIMIT)
    {
        p = DENSITY_LIMIT;
    }

    float result = vf * std::log(_pj/p);

    if (result > vf) // км/ч
        return vf;
    else
        return result;

}

Route Router::findPath(int startNodeIndex, int endNodeIndex, float startTime, float weight, bool withLoad, bool densityUpdate, Algorithm algorithm)
{
    Route route;

    switch (algorithm)
    {
    case Algorithm::Dijkstra:
    {
        route = findPathDijkstra(startNodeIndex, endNodeIndex, startTime, withLoad, densityUpdate);
        break;
    }
    case Algorithm::AStar:
    {
        route = findPathAStar(startNodeIndex, endNodeIndex, startTime, 1, withLoad, densityUpdate);
        break;
    }
    case Algorithm::WeightedAStar:
    {
        route = findPathAStar(startNodeIndex, endNodeIndex, startTime, weight, withLoad, densityUpdate);
        break;
    }
    case Algorithm::BellManFord:
    {
        route = findPathBellmanFord(startNodeIndex, endNodeIndex, startTime, withLoad, densityUpdate);
        break;
    }
    default:
        break;
    }

    return route;
}

void Router::addRoutes(unsigned int numOfCars, int batchSize, int timeInterval, float weight, bool withLoad, bool densityUpdate, Algorithm algorithm, const Arguments &args)
{
    struct RouteStats
    {
        float cost = 0;
        double execTime = 0;
        int visitedNodeCount = 0;
    };

    int graphSize = _nodeList.size();
    std::vector<int> path;
    std::vector<double> travelTimes;
    std::vector<Route> routes;

    QFile file;

    if(withLoad)
        file.setFileName("simulationTime.csv");
    else
        file.setFileName("simulationDistance.csv");

    if (file.exists())
        file.remove();

    QTextStream out;


    if (file.open(QFile::WriteOnly | QFile::Append))
    {
        out.setDevice(&file);
    }

    std::vector<RouteStats> avgRoutes(numOfCars);

    initDensities(_intervalTime);

    unsigned int i = 0;

    QRandomGenerator random(1234);
    while (i < numOfCars)
    {
        int startNode = random.bounded(0,graphSize);
        int targetNode = random.bounded(0,graphSize);
        while (_nodeList[startNode].point.GetCoord().GetDistance(_nodeList[targetNode].point.GetCoord()).AsMeter() < (args.radius))
        {
            startNode = random.bounded(0,graphSize);
            targetNode = random.bounded(0,graphSize);
        }
        auto route = findPath(startNode, targetNode, timeInterval, weight, withLoad, densityUpdate, algorithm);
        path = route.constructedRoute;
        if (!path.empty())
        {
            out << i+1 << "," << route.travelTime << "," << route.execTime << "," << route.visitedNodeCount << "," << route.startTime <<"\n";

            if (i % batchSize == 0)
            {
                timeInterval += 2;
            }
            i++;
        }
    }

    file.close();
}

float Router::calculateRouteCost(const std::vector<int>& paths, float startTime, bool densityUpdate)
{
    float travelTime = 0;
    for (int pathIndex : paths)
    {
        Path path = _pathList[pathIndex];
        float density = path.densities[std::floor((travelTime+startTime)/_intervalTime)];
        auto diagramRes = trafficDiagramFunction(density/((path.distanceLength.AsMeter() / 1000.0)*path.lanes), path.maxSpeed);
        float pathCost = ((path.distanceLength.AsMeter() / 1000.0) / (diagramRes)) * 3600; // в секундах
        if (densityUpdate)
        {
            float updateTime = 0;
            while (updateTime < pathCost)
            {
                _pathList[pathIndex].densities[std::floor((travelTime+startTime+updateTime)/_intervalTime)] += 1.0;

                if (_pathList[pathIndex].densities[std::floor((travelTime+startTime+updateTime)/_intervalTime)] /
                        ((_pathList[pathIndex].distanceLength.AsMeter()/1000)*_pathList[pathIndex].lanes) > DENSITY_LIMIT)
                {
                    _pathList[pathIndex].densities[std::floor((travelTime+startTime+updateTime)/_intervalTime)] -= 1.0;
                }
                updateTime += _intervalTime;
            }

            if(std::floor((travelTime+startTime+updateTime-_intervalTime)/_intervalTime)<std::floor((travelTime+startTime+pathCost)/_intervalTime))
            {
                _pathList[pathIndex].densities[std::floor((travelTime+startTime+pathCost)/_intervalTime)] += 1.0;

                if (_pathList[pathIndex].densities[std::floor((travelTime+startTime+pathCost)/_intervalTime)] /
                        ((_pathList[pathIndex].distanceLength.AsMeter()/1000)*_pathList[pathIndex].lanes) > DENSITY_LIMIT)
                {
                    _pathList[pathIndex].densities[std::floor((travelTime+startTime+pathCost)/_intervalTime)] -= 1.0;
                }
            }
        }
        travelTime += pathCost;
    }
    return travelTime;
}
