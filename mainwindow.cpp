#include <osmscout/db/Database.h>
#include <osmscout/feature/ConstructionYearFeature.h>
#include <osmscoutmap/MapService.h>
#include <osmscoutmapqt/MapPainterQt.h>

#include <iostream>

#include <QGuiApplication>
#include <QApplication>
#include <QScreen>
#include <QPushButton>
#include <QLineEdit>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QLabel>
#include <QRandomGenerator>
#include <QFile>
#include <QMessageBox>
#include <QSpinBox>
#include <QTextStream>
#include <QStandardItemModel>
#include <QHBoxLayout>

#include <queue>
#include <unordered_set>

#include "mainwindow.h"

MainWindow::MainWindow(int argc, char *argv[], double screen, QWidget *parent)
    : QMainWindow(parent),
    _map("QMap", argc, argv, screen)
{
    _router = new Router();
    QFile file("output.csv");
    if (file.exists())
        file.remove();

    QGroupBox* timeGroupBox = new QGroupBox(this);
    QGridLayout* userLayout = new QGridLayout;

    QLabel* modelingTimeLabel = new QLabel("Время: " + (QString::number(_modelingTime)),timeGroupBox);
    QLabel* timeIntervalLabel = new QLabel("Интервал дискретизации:");
    QLineEdit* timeIntervalLineEdit = new QLineEdit(QString::number(_intervalTime),this);
    QPushButton* increaseTimeButton = new QPushButton("Увеличить время",this);
    QPushButton* decreaseTimeButton = new QPushButton("Уменьшить время",this);
    QLabel* startNodeLabel = new QLabel("Начальный узел:");
    _startNodeLineEdit = new QLineEdit(this);
    QLabel* targetNodeLabel = new QLabel("Конечный узел:");
    _targetNodeLineEdit = new QLineEdit(this);
    QHBoxLayout* nodeLayout = new QHBoxLayout;
    nodeLayout->addWidget(startNodeLabel);
    nodeLayout->addWidget(_startNodeLineEdit);
    nodeLayout->addWidget(targetNodeLabel);
    nodeLayout->addWidget(_targetNodeLineEdit);

    QPushButton* constructRouteButton = new QPushButton("Построить маршрут",this);
    QLabel* modeNameLabel = new QLabel("Вес ребра:", this);
    _weightTypeComboBox = new QComboBox(this);
    _weightTypeComboBox->insertItems(0,{"Расстояние", "Время"});
    _weightTypeComboBox->setCurrentIndex(0);
    _updateDensitiesCheckBox = new QCheckBox("Учет влияния построенных маршрутов", this);

    QLabel* algorithmNameLabel = new QLabel("Алгоритм:", this);
    _algorithmComboBox = new QComboBox(this);
    _algorithmComboBox->insertItems(0,{"Алгоритм Дейкстры", "A*", "Weighted A*", "Алгоритм Беллмана-Форда"});
    _algorithmComboBox->setCurrentIndex(0);
    QLabel* weigthLabel = new QLabel("Вес алгоритма Weighted A*:");
    _weightLineEdit = new QLineEdit(QString::number(1),this);
    QPushButton* resetButton = new QPushButton("Вернуть дороги с исходному состоянию",this);

    QGroupBox* optionsGroupBox = new QGroupBox(this);
    _showTrafficCheckBox = new QCheckBox("Отображение нагруженности дорог",this);
    _showLastRouteCheckBox = new QCheckBox("Отображение последнего маршрута",this);
    _showNodesCheckBox = new QCheckBox("Отображение узлов",this);
    _showNodesIndexCheckBox = new QCheckBox("Отображение номеров узлов",this);
    QVBoxLayout* optionsLayout = new QVBoxLayout;
    optionsLayout->addWidget(_showTrafficCheckBox);
    optionsLayout->addWidget(_showLastRouteCheckBox);
    optionsLayout->addWidget(_showNodesCheckBox);
    optionsLayout->addWidget(_showNodesIndexCheckBox);
    optionsGroupBox->setLayout(optionsLayout);

    QLabel* numOfCarsLabel = new QLabel("Количество автомобилей:", this);
    QSpinBox* numOfCarsSpinBox = new QSpinBox(this);
    numOfCarsSpinBox->setValue(10);
    numOfCarsSpinBox->setRange(0, 100000);
    QPushButton* runButton = new QPushButton("Запустить",this);
    connect(runButton, &QPushButton::clicked, [this, numOfCarsSpinBox]
        {
            addRoutes(numOfCarsSpinBox->value());
        });

    QLabel* resultLabel = new QLabel("Результат:",this);
    _resultTextEdit = new QTextEdit(this);

    userLayout->addWidget(modelingTimeLabel, 1, 0, 1, 2);
    userLayout->addWidget(timeIntervalLabel, 2, 0);
    userLayout->addWidget(timeIntervalLineEdit, 2, 1);
    userLayout->addWidget(increaseTimeButton, 4, 0);
    userLayout->addWidget(decreaseTimeButton, 4, 1);
    userLayout->addWidget(modeNameLabel, 5, 0);
    userLayout->addWidget(_weightTypeComboBox, 5, 1);
    userLayout->addWidget(algorithmNameLabel, 6, 0);
    userLayout->addWidget(_algorithmComboBox, 6, 1);
    userLayout->addWidget(weigthLabel, 7, 0);
    userLayout->addWidget(_weightLineEdit, 7, 1);
    userLayout->addWidget(_updateDensitiesCheckBox, 8, 0, 1, 2);
    userLayout->addLayout(nodeLayout, 9, 0, 1, 2);
    userLayout->addWidget(constructRouteButton, 10, 0, 1, 2);
    userLayout->addWidget(optionsGroupBox, 11, 0, 1, 2);
    userLayout->addWidget(resetButton, 12, 0, 1, 2);
    userLayout->addWidget(numOfCarsLabel, 13, 0);
    userLayout->addWidget(numOfCarsSpinBox, 13, 1);
    userLayout->addWidget(runButton, 14, 0, 1, 2);
    userLayout->addWidget(resultLabel, 15, 0);
    userLayout->addWidget(_resultTextEdit, 16, 0, 1, 2);

    userLayout->setColumnStretch(userLayout->columnCount(), 1);
    userLayout->setRowStretch(userLayout->rowCount(), 1);
    //timeLayout->addStretch(1);

    timeGroupBox->setLayout(userLayout);


    _scene = new GraphicsScene();
    auto screenGeometry = QApplication::primaryScreen()->geometry();
    //scene_->setSceneRect(0, 0, 800, 600);
    _scene->setSceneRect(0, 0, screenGeometry.width(), screenGeometry.height());

    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);


    _graphicsView = new QGraphicsView(_scene, this);
    // _graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    // _graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    //setCentralWidget(graphicsView);
    _graphicsView->setRenderHint(QPainter::Antialiasing);
    _graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
    _graphicsView->viewport()->installEventFilter(this);
    _scene->installEventFilter(this);

    mainLayout->addWidget(timeGroupBox);
    mainLayout->addWidget(_graphicsView, 1);

    auto size = _graphicsView->size();

    _map.openDatabase();
    _args = _map.GetArguments();
    _router->setDatabase(_map.database);
    _scene->setProjection(&_map.projection);
    _pixmap = new QPixmap(static_cast<int>(_args.width),
                          static_cast<int>(_args.height));
    _painter = new QPainter(_pixmap);
    //painter_->setBackground(QBrush(Qt::white));


    connect(increaseTimeButton, &QPushButton::clicked, [this, modelingTimeLabel]
    {
        _modelingTime += _intervalTime;
        modelingTimeLabel->setText("Время: " + QString::number(_modelingTime));
    });

    connect(decreaseTimeButton, &QPushButton::clicked, [this, modelingTimeLabel]
    {
        int res = _modelingTime - _intervalTime;
        if (res <= 0)
            _modelingTime = 0;
        else
            _modelingTime -= _intervalTime;
        modelingTimeLabel->setText("Время: " + QString::number(_modelingTime));
    });

    connect(constructRouteButton, &QPushButton::clicked, [this]
    {
        Route route;
        if(!_startNodeLineEdit->text().isEmpty() && !_targetNodeLineEdit->text().isEmpty())
        {
            route = findPath(_startNodeLineEdit->text().toInt(), _targetNodeLineEdit->text().toInt(), 0, _updateDensitiesCheckBox->isChecked());
        }
        else
        {
            QMessageBox::warning(this, "Предупреждение", "Узел не найден. Повторите ввод.");
            return;
        }
        if(!route.constructedRoute.empty())
        {
            //_scene->paintPath(route.constructedRoute);
            _lastRoute = route.constructedRoute;
            QMessageBox::information(this,"Информация", "Маршрут построен.");
            paintMap();
        }
        else
            QMessageBox::information(this,"Информация", "Маршрут не найден.");
    });

    connect(timeIntervalLineEdit, &QLineEdit::editingFinished, [this, timeIntervalLineEdit]
    {
        _intervalTime = timeIntervalLineEdit->text().toInt();
        _router->setIntervalTime(_intervalTime);
    });

    connect(_showLastRouteCheckBox, &QCheckBox::stateChanged, [this]
    {
        paintMap();
    });

    connect(_showNodesCheckBox, &QCheckBox::stateChanged, [this]
    {
        paintMap();
    });

    connect(_showTrafficCheckBox, &QCheckBox::stateChanged, [this]
    {
        paintMap();
    });

    connect(_showNodesIndexCheckBox, &QCheckBox::stateChanged, [this]
    {
        paintMap();
    });


    // connect(_weightTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)
    // {
    //     _planningMode = static_cast<WeightType>(index);
    // });

    // connect(algorithmComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)
    // {
    //     _algorithm = static_cast<Algorithm>(index);
    // });

}

void MainWindow::setMap()
{
    osmscout::MapPainterQt mapPainter(_map.styleConfig);
    _pixmap->fill(Qt::white);
    _map.loadData();
    if (mapPainter.DrawMap(_map.projection, _map.drawParameter,
                           _map.data, _painter))
    {
        _scene->setMap(_pixmap);
    }
}

void MainWindow::initGraph()
{
    osmscout::Distance dist(osmscout::Distance::Of<osmscout::Meter>(_args.radius));
    _router->loadNodesData(_args, dist, _args.center);
    _router->buildGraph();

    //auto& graph = router_->getGraph();
    _graphRef = &_router->getGraph();
    _pathListRef = &_router->getPathList();
    _scene->setGraph(_graphRef);
    _scene->setPathList(_pathListRef);
    _router->initDensities(_intervalTime);
    _router->setIntervalTime(_intervalTime);

    QIntValidator* nodeValidator = new QIntValidator(0, _graphRef->size()-1, this);
    _startNodeLineEdit->setValidator(nodeValidator);
    _targetNodeLineEdit->setValidator(nodeValidator);


    //_scene->paintDots(_graphRef);

    //scene_->paintCurrentTraffic(_graphRef, _pathListRef,_modelingTime,_intervalTime);

    //scene_->paintAllPathIndexes(_graphRef, _pathListRef);
    //scene_->paintAllNodeIndexes(_graphRef);

    //const auto& path = router_->findPathAStar(5,98);
    // const auto& path = router_->findPathAStarTime(5,98,_startTimeLineEdit->text().toInt(),_intervalTime);
    // scene_->paintPath(_graphRef, path);

    //onst auto& path = router_->findPathAStar(844,2);
    //const auto& path = router_->findPathDijkstra(844,2);
    //scene_->paintPath(_graphRef, path);
    //placeCars(24);
}

void MainWindow::changeMapZoom(double zoomFactor)
{
    _scene->clearMap();
    _scene->clear();
    _args.zoom.SetMagnification(_args.zoom.GetMagnification() * zoomFactor);
    _map.projection.Set(_args.center, _args.angle.AsRadians(), _args.zoom,
                            _args.dpi, _args.width, _args.height);
    delete _painter;
    delete _pixmap;
    _pixmap = new QPixmap(static_cast<int>(_args.width),
                          static_cast<int>(_args.height));
    _pixmap->fill(Qt::white);
    _painter = new QPainter(_pixmap);
    osmscout::MapPainterQt mapPainter(_map.styleConfig);
    _map.loadData();
    if (mapPainter.DrawMap(_map.projection, _map.drawParameter,
                           _map.data, _painter))
    {
        _scene->setMap(_pixmap);
    }
    //paintMap();

    if (_showTrafficCheckBox->isChecked())
        _scene->paintCurrentTraffic(_modelingTime,_intervalTime);
    if (_showNodesCheckBox->isChecked())
        _scene->paintNodes();
    if (_showLastRouteCheckBox->isChecked())
        _scene->paintPath(_lastRoute);
    if (_showNodesIndexCheckBox->isChecked())
        _scene->paintAllNodeIndexes();

}

void MainWindow::moveMap(osmscout::GeoCoord coord)
{
    _scene->clearMap();
    _scene->clear();
    _args.center = coord;
    _map.projection.Set(_args.center, _args.angle.AsRadians(), _args.zoom,
                            _args.dpi, _args.width, _args.height);
    delete _painter;
    delete _pixmap;
    _pixmap = new QPixmap(static_cast<int>(_args.width),
                          static_cast<int>(_args.height));
    _pixmap->fill(Qt::white);
    _painter = new QPainter(_pixmap);
    osmscout::MapPainterQt mapPainter(_map.styleConfig);
    _map.loadData();
    if (mapPainter.DrawMap(_map.projection, _map.drawParameter,
                           _map.data, _painter))
    {
        _scene->setMap(_pixmap);
    }
    //paintMap();

    if (_showTrafficCheckBox->isChecked())
        _scene->paintCurrentTraffic(_modelingTime,_intervalTime);
    if (_showNodesCheckBox->isChecked())
        _scene->paintNodes();
    if (_showLastRouteCheckBox->isChecked())
        _scene->paintPath(_lastRoute);
    if (_showNodesIndexCheckBox->isChecked())
        _scene->paintAllNodeIndexes();

}

void MainWindow::paintMap()
{
    _scene->clearMap();
    _scene->clear();
    changeMapZoom(1);
}

bool MainWindow::eventFilter(QObject *object, QEvent *e)
{
    if (object == _graphicsView->viewport() && e->type() == QEvent::Wheel)
    {
        auto event = static_cast<QWheelEvent *>(e);
        if (event->modifiers().testFlag(Qt::ControlModifier))
        {
            double scaleZoom = 1.15;
            _zoom *= scaleZoom;
            if (event->angleDelta().y() > 0)
            {
                changeMapZoom(scaleZoom);
            }
            else if (event->angleDelta().y() < 0)
            {
                changeMapZoom(1.0 / scaleZoom);
            }
            return true;
        }
    }
    if (object == _scene && e->type() == QGraphicsSceneMouseEvent::GraphicsSceneMousePress)
    {
        auto event = static_cast<QGraphicsSceneMouseEvent*>(e);
        if (event->buttons() & Qt::RightButton)
        {
            osmscout::GeoCoord coord;
            _scene->_projection->PixelToGeo(event->scenePos().x(), event->scenePos().y(), coord);
            moveMap(coord);
            return true;
        }
    }
    return false;
}

Route MainWindow::findPath(int startNodeIndex, int endNodeIndex, int startTime, bool densityUpdate)
{
    WeightType weightType = static_cast<WeightType>(_weightTypeComboBox->currentIndex());
    Algorithm algorithm = static_cast<Algorithm>(_algorithmComboBox->currentIndex());
    float weight = _weightLineEdit->text().toFloat();
    Route route;

    switch (algorithm)
    {
    case Algorithm::Dijkstra:
    {
        if(weightType == WeightType::Distance)
            route = _router->findPathDijkstra(startNodeIndex, endNodeIndex, startTime, densityUpdate);
        else
            route = _router->findPathDijkstraTime(startNodeIndex, endNodeIndex, startTime, densityUpdate);
        break;
    }
    case Algorithm::AStar:
    {
        if(weightType == WeightType::Distance)
            route = _router->findPathAStar(startNodeIndex, endNodeIndex, startTime, 1, densityUpdate);
        else
            route = _router->findPathAStarTime(startNodeIndex, endNodeIndex, startTime, 1, densityUpdate);
        break;
    }
    case Algorithm::WeightedAStar:
    {
        if(weightType == WeightType::Distance)
            route = _router->findPathAStar(startNodeIndex, endNodeIndex, startTime, weight, densityUpdate);
        else
            route = _router->findPathAStarTime(startNodeIndex, endNodeIndex, startTime, weight, densityUpdate);
        break;
    }
    case Algorithm::BellManFord:
    {
        if(weightType == WeightType::Distance)
            route = _router->findPathBellmanFord(startNodeIndex, endNodeIndex, startTime, densityUpdate);
        else
            route = _router->findPathBellmanFordTime(startNodeIndex, endNodeIndex, startTime, densityUpdate);
        break;
    }
    default:
        break;
    }

    return route;
}

void MainWindow::addRoutes(unsigned int numOfCars)
{
    struct RouteStats
    {
        float cost = 0;
        double execTime = 0;
        int visitedNodeCount = 0;
    };

    int graphSize = _graphRef->size();
    std::vector<int> path;
    std::vector<double> travelTimes;
    std::vector<std::vector<int>> routes;

    WeightType weightType = static_cast<WeightType>(_weightTypeComboBox->currentIndex());
    QFile file;

    if(weightType == WeightType::Distance)
        file.setFileName("simulationDistance.csv");
    else
        file.setFileName("simulationTime.csv");

    if (file.exists())
        file.remove();

    QTextStream out;


    if (file.open(QFile::WriteOnly | QFile::Append))
    {
        out.setDevice(&file);
    }

    std::vector<RouteStats> avgRoutes(numOfCars);

    _router->setIntervalTime(_intervalTime);
    _router->initDensities(_intervalTime);

    unsigned int i = 0;
    int routeStartTime = 0;
    bool densityUpdate = _updateDensitiesCheckBox->isChecked();

    int s =0;
    while (s<1)
    {
        i = 0;
        QRandomGenerator random(1234+s);
        //_router->generateDensities(_intervalTime);
        while (i < numOfCars)
        {// 398,543
            int startNode = random.bounded(0,graphSize);
            int targetNode = random.bounded(0,graphSize);
            while (startNode == targetNode)
            {
                startNode = random.bounded(0,graphSize);
                targetNode = random.bounded(0,graphSize);
            }

            auto route = findPath(startNode,targetNode,routeStartTime, densityUpdate);
            //auto route = _router->findPathUniversal(startNode,targetNode,routeStartTime,_intervalTime,weightType,Algorithm::AStar, densityUpdate);
            //std::cout << startNode << std::endl;
            path = route.constructedRoute;
            if (!path.empty())
            {
                routes.push_back(route.constructedRoute);
                // for (auto route : routes)
                // {

                // }
                travelTimes.push_back(route.cost);

                out << i+1 << "," << route.cost << "," << route.execTime << "," << route.visitedNodeCount << "\n";

                routeStartTime += 2;

                // avgRoutes[i].cost += (route.cost - avgRoutes[i].cost) / (s + 1);
                // avgRoutes[i].execTime += (route.execTime - avgRoutes[i].execTime) / (s + 1);
                // avgRoutes[i].visitedNodeCount += (int(route.visitedNodeCount) - avgRoutes[i].visitedNodeCount) / (s + 1);
                i++;
            }
        }
        s++;
    }

    // i = 0;
    // for (const auto& routeInfo : avgRoutes)
    // {
    //     out << i+1 << "," << routeInfo.cost << "," << routeInfo.execTime << "," << routeInfo.visitedNodeCount << "\n";
    //     i++;
    // }

    //double avgTravelTimeDefault = std::accumulate(travelTimes.begin(), travelTimes.end(), 0.0)/travelTimes.size();
    QMessageBox::information(this, "Информация","Моделирование завершено.");
    //int temp = 0;
}

MainWindow::~MainWindow()
{
}
