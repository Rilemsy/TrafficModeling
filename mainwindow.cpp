#include <osmscout/db/Database.h>
#include <osmscout/feature/ConstructionYearFeature.h>
#include <osmscoutmap/MapService.h>
#include <osmscoutmapqt/MapPainterQt.h>

#include <iostream>

#include <QGuiApplication>
#include <QApplication>
#include <QScreen>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QLabel>
#include <QRandomGenerator>
#include <QFile>
#include <QComboBox>
#include <QMessageBox>
#include <QSpinBox>
#include <QTextStream>

#include <queue>
#include <unordered_set>

#include "mainwindow.h"

MainWindow::MainWindow(int argc, char *argv[], double screen, QWidget *parent)
    : QMainWindow(parent),
    _mapData("QMap", argc, argv, screen)
{
    _router = new Router();
    QFile file("output.csv");
    if (file.exists())
        file.remove();

    QGroupBox* timeGroupBox = new QGroupBox(this);
    QGridLayout* timeLayout = new QGridLayout;

    QLabel* modelingTimeLabel = new QLabel("Время: " + (QString::number(_modelingTime)),timeGroupBox);
    QLabel* startTimeLabel = new QLabel("Время начала поездки:");
    _startTimeLineEdit = new QLineEdit(QString::number(_modelingTime),this);
    QLabel* timeIntervalLabel = new QLabel("Интервал времени:");
    QLineEdit* timeIntervalLineEdit = new QLineEdit(QString::number(_intervalTime),this);
    QPushButton* increaseTimeButton = new QPushButton("Увеличить время",this);
    QPushButton* addCarButton = new QPushButton("Добавить автомобиль",this);
    QPushButton* clearMapButton = new QPushButton("Очистить карту",this);
    QLabel* modeNameLabel = new QLabel("Режим:", this);
    QLabel* algorithmNameLabel = new QLabel("Алгоритм:", this);
    QComboBox* modeComboBox = new QComboBox(this);
    modeComboBox->insertItems(0,{"Только расстояние", "Исторические данные", "Влияние водителей"});
    modeComboBox->setCurrentIndex(2);
    QComboBox* algorithmComboBox = new QComboBox(this);
    algorithmComboBox->insertItems(0,{"Алгоритм Дийкстры", "A*"});
    algorithmComboBox->setCurrentIndex(1);

    QLabel* timeMomentLabel = new QLabel("Перейти к моменту времени:", this);
    QLineEdit* timeMomentLineEdit = new QLineEdit(QString::number(_momentTime),this);

    _optionsListWidget = new QListWidget(this);
    QListWidgetItem* listItem = new QListWidgetItem("Показать номера узлов", _optionsListWidget);
    listItem->setFlags(listItem->flags() | Qt::ItemIsUserCheckable);
    listItem->setCheckState(Qt::Unchecked);
    listItem = new QListWidgetItem("Показать номера путей", _optionsListWidget);
    listItem->setFlags(Qt::ItemIsUserCheckable | listItem->flags());
    listItem->setCheckState(Qt::Unchecked);
    listItem = new QListWidgetItem("Показать трафик", _optionsListWidget);
    listItem->setFlags(Qt::ItemIsUserCheckable | listItem->flags());
    listItem->setCheckState(Qt::Unchecked);
    listItem = new QListWidgetItem("Показать последний маршрут", _optionsListWidget);
    listItem->setFlags(Qt::ItemIsUserCheckable | listItem->flags());
    listItem->setCheckState(Qt::Unchecked);
    listItem = new QListWidgetItem("Показать узлы", _optionsListWidget);
    listItem->setFlags(Qt::ItemIsUserCheckable | listItem->flags());
    listItem->setCheckState(Qt::Unchecked);

    QSpinBox* numOfCarsSpinBox = new QSpinBox(this);
    numOfCarsSpinBox->setValue(10);
    numOfCarsSpinBox->setRange(0, 1000000);
    QPushButton* compareButton = new QPushButton("Сравнить",this);
    connect(compareButton, &QPushButton::clicked, [this, numOfCarsSpinBox]
        {
            runSimulation(numOfCarsSpinBox->value());
        });

    timeLayout->addWidget(startTimeLabel, 1, 0);
    timeLayout->addWidget(timeIntervalLabel, 2, 0);

    timeLayout->addWidget(_startTimeLineEdit, 1, 1);
    timeLayout->addWidget(timeIntervalLineEdit, 2, 1);
    timeLayout->addWidget(modelingTimeLabel, 3, 0, 1, 2);
    timeLayout->addWidget(increaseTimeButton, 4, 0, 1, 2);
    timeLayout->addWidget(addCarButton, 5, 0, 1, 2);
    timeLayout->addWidget(clearMapButton, 6, 0, 1, 2);
    timeLayout->addWidget(modeNameLabel, 7, 0);
    timeLayout->addWidget(modeComboBox, 7, 1);
    timeLayout->addWidget(algorithmNameLabel, 8, 0);
    timeLayout->addWidget(algorithmComboBox, 8, 1);
    timeLayout->addWidget(timeMomentLabel, 9, 0);
    timeLayout->addWidget(timeMomentLineEdit, 9, 1);
    timeLayout->addWidget(_optionsListWidget, 10, 0, 1, 2);
    timeLayout->addWidget(numOfCarsSpinBox, 11, 0);
    timeLayout->addWidget(compareButton, 11, 1);

    timeLayout->setColumnStretch(timeLayout->columnCount(), 1);
    timeLayout->setRowStretch(timeLayout->rowCount(), 1);
    //timeLayout->addStretch(1);

    timeGroupBox->setLayout(timeLayout);


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

    _mapData.openDatabase();
    _args = _mapData.GetArguments();
    _router->setDatabase(_mapData.database);
    _scene->setProjection(&_mapData.projection);
    _pixmap = new QPixmap(static_cast<int>(_args.width),
                          static_cast<int>(_args.height));
    _painter = new QPainter(_pixmap);
    //painter_->setBackground(QBrush(Qt::white));


    connect(increaseTimeButton, &QPushButton::clicked, [this, modelingTimeLabel]
    {
        _modelingTime += _intervalTime;
        modelingTimeLabel->setText("Время: " + QString::number(_modelingTime));
    });

    connect(addCarButton, &QPushButton::clicked, [this]
    {
        this->placeCars(1);
    });

    connect(timeIntervalLineEdit, &QLineEdit::editingFinished, [this, timeIntervalLineEdit]
    {
        _intervalTime = timeIntervalLineEdit->text().toDouble();
        _router->setIntervalTime(_intervalTime);
    });

    connect(timeMomentLineEdit, &QLineEdit::editingFinished, [this, timeMomentLineEdit]
    {
        _momentTime = timeMomentLineEdit->text().toDouble();
    });

    connect(modeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)
    {
        _planningMode = static_cast<PlanningMode>(index);
    });

    connect(algorithmComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)
    {
        _algorithm = static_cast<Algorithm>(index);
    });

    connect(_router, &Router::message, [this](QString str)
    {
        QMessageBox::warning(this, "Warning", str);
    });
    connect(_optionsListWidget, &QListWidget::itemChanged, [this](QListWidgetItem* item)
    {
        int currentRow = _optionsListWidget->row(item);
        int option = std::pow(2,currentRow) ;
        if (currentRow == 0)
            _options.setFlag(ShowNodeNumber, item->checkState());
        else
            _options.setFlag(static_cast<Option>(option), item->checkState());
        paintMap();
    });
}

void MainWindow::setData()
{
    osmscout::MapPainterQt mapPainter(_mapData.styleConfig);
    osmscout::TypeInfoRef buildingType =
        _mapData.database->GetTypeConfig()->GetTypeInfo("building");
    _pixmap->fill(Qt::white);
    _mapData.loadData();
    if (mapPainter.DrawMap(_mapData.projection, _mapData.drawParameter,
                           _mapData.data, _painter))
    {
        _scene->setMap(_pixmap);
    }
}

void MainWindow::initGraph()
{
    osmscout::GeoCoord pointFrom(55.6565, 41.8260);
    osmscout::Distance dist = pointFrom.GetDistance(osmscout::GeoCoord(55.6876, 42.6846));
    _router->loadDataNodes(_args, dist, pointFrom);
    _router->setupGraphFromNodes();

    //auto& graph = router_->getGraph();
    _graphRef = &_router->getGraph();
    _pathListRef = &_router->getPathList();
    _router->generateDensities(_intervalTime);
    _router->setIntervalTime(_intervalTime);


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
    _mapData.projection.Set(_args.center, _args.angle.AsRadians(), _args.zoom,
                            _args.dpi, _args.width, _args.height);
    delete _painter;
    delete _pixmap;
    _pixmap = new QPixmap(static_cast<int>(_args.width),
                          static_cast<int>(_args.height));
    _pixmap->fill(Qt::white);
    _painter = new QPainter(_pixmap);
    osmscout::MapPainterQt mapPainter(_mapData.styleConfig);
    _mapData.loadData();
    if (mapPainter.DrawMap(_mapData.projection, _mapData.drawParameter,
                           _mapData.data, _painter))
    {
        _scene->setMap(_pixmap);
    }
    //paintMap();

    if (_options.testFlag(ShowNodeDot))
        _scene->paintDots(_graphRef);
    if (_options.testFlag(ShowNodeNumber))
        _scene->paintAllNodeIndexes(_graphRef);
    if (_options.testFlag(ShowEdgeNumber))
        _scene->paintAllPathIndexes(_graphRef, _pathListRef);
    if (_options.testFlag(ShowTraffic))
        _scene->paintCurrentTraffic(_graphRef,_pathListRef, _modelingTime,_intervalTime, _planningMode);
    if (_options.testFlag(ShowLastRoute))
        _scene->paintPath(_graphRef,_lastRoute);

}

void MainWindow::moveMap(osmscout::GeoCoord coord)
{
    _scene->clearMap();
    _scene->clear();
    _args.center = coord;
    _mapData.projection.Set(_args.center, _args.angle.AsRadians(), _args.zoom,
                            _args.dpi, _args.width, _args.height);
    delete _painter;
    delete _pixmap;
    _pixmap = new QPixmap(static_cast<int>(_args.width),
                          static_cast<int>(_args.height));
    _pixmap->fill(Qt::white);
    _painter = new QPainter(_pixmap);
    osmscout::MapPainterQt mapPainter(_mapData.styleConfig);
    _mapData.loadData();
    if (mapPainter.DrawMap(_mapData.projection, _mapData.drawParameter,
                           _mapData.data, _painter))
    {
        _scene->setMap(_pixmap);
    }
    //paintMap();

    if (_options.testFlag(ShowNodeDot))
        _scene->paintDots(_graphRef);
    if (_options.testFlag(ShowNodeNumber))
        _scene->paintAllNodeIndexes(_graphRef);
    if (_options.testFlag(ShowEdgeNumber))
        _scene->paintAllPathIndexes(_graphRef, _pathListRef);
    if (_options.testFlag(ShowTraffic))
        _scene->paintCurrentTraffic(_graphRef,_pathListRef, _modelingTime,_intervalTime, _planningMode);
    if (_options.testFlag(ShowLastRoute))
        _scene->paintPath(_graphRef,_lastRoute);

}

void MainWindow::paintMap()
{
    _scene->clearMap();
    _scene->clear();
    changeMapZoom(1);

    // if (_options.testFlag(ShowNodeDot))
    //     _scene->paintDots(_graphRef);
    // if (_options.testFlag(ShowNodeNumber))
    //     _scene->paintAllNodeIndexes(_graphRef);
    // if (_options.testFlag(ShowEdgeNumber))
    //     _scene->paintAllPathIndexes(_graphRef, _pathListRef);
    // if (_options.testFlag(ShowTraffic))
    //     _scene->paintCurrentTraffic(_graphRef,_pathListRef, _modelingTime,_intervalTime, _planningMode);
    // if (_options.testFlag(ShowLastRoute))
    //     _scene->paintPath(_graphRef,_lastRoute);

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

void MainWindow::placeCars(int amount)
{
    QRandomGenerator random(1234);
    int size = _graphRef->size();
    std::vector<int> path;
    for (int i = 0; i < amount; i++)
    {
        //const auto& path = router_->findPathAStarTime(random.bounded(0, size),random.bounded(0, size),_startTimeLineEdit->text().toInt(),_intervalTime);
        //const auto& path = router_->findPathAStarTime(844,2,_startTimeLineEdit->text().toInt(),_intervalTime);
        //const auto& path = router_->findPathDijkstraTime(844,2,_startTimeLineEdit->text().toInt(),_intervalTime);
        //const auto& path = router_->findPathUniversal(844,2,_startTimeLineEdit->text().toInt(),_intervalTime, _planningMode, _algorithm);
        /*const auto&*/

        auto route = _router->findPathUniversal(398,543,_startTimeLineEdit->text().toInt(),_intervalTime, _planningMode, _algorithm, true);
        _lastRoute = route.constructedRoute;
        //_lastRoute = _router->findPathBellmanFord(398,543,_startTimeLineEdit->text().toInt(),_intervalTime, _planningMode);

    }

    paintMap();

    // _scene->clearMap();
    // _scene->clear();
    // changeMapZoom(1);
    // _scene->paintCurrentTraffic(_graphRef, _pathListRef,_modelingTime,_intervalTime, _planningMode);
    // _scene->paintPath(_graphRef, _lastRoute);
    //scene_->paintAllNodeIndexes(_graphRef);
}

void MainWindow::runSimulation(unsigned int numOfCars)
{
    QRandomGenerator random(1234);
    int graphSize = _graphRef->size();
    std::vector<int> path;
    std::vector<double> travelTimes;
    std::vector<std::vector<int>> routes;

    QFile file("simulationD.csv");
    if (file.exists())
        file.remove();

    QTextStream out;


    if (file.open(QFile::WriteOnly | QFile::Append))
    {
        out.setDevice(&file);
    }

    _router->generateDensities(_intervalTime);

    unsigned int i = 0;
    int routeStartTime = _startTimeLineEdit->text().toInt();
    while (i < numOfCars)
    {// 398,543
        int startNode = random.bounded(0,graphSize);
        int targetNode = random.bounded(0,graphSize);
        while (startNode == targetNode)
        {
            startNode = random.bounded(0,graphSize);
            targetNode = random.bounded(0,graphSize);
        }

        auto route = _router->findPathUniversal(startNode,targetNode,routeStartTime,_intervalTime, PlanningMode::OnlyDistance, _algorithm, true);
        if (!path.empty())
        {
            routes.push_back(route.constructedRoute);
            // for (auto route : routes)
            // {

            // }
            travelTimes.push_back(route.cost);

            out << i+1 << "," << route.cost << "\n";

            routeStartTime += 0;
            i++;
        }
    }
    double avgTravelTimeDefault = std::accumulate(travelTimes.begin(), travelTimes.end(), 0.0)/travelTimes.size();

}

MainWindow::~MainWindow()
{
}
