#include <osmscout/db/Database.h>
#include <osmscout/feature/ConstructionYearFeature.h>
#include <osmscoutmap/MapService.h>
#include <osmscoutmapqt/MapPainterQt.h>

#include <iostream>
#include <libsumo/libtraci.h>

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

#include <queue>
#include <unordered_set>

#include "mainwindow.h"

using namespace libtraci;

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

    _optionsList = new QListWidget(this);
    QListWidgetItem* listItem = new QListWidgetItem("Показать номера узлов", _optionsList);
    listItem->setFlags(listItem->flags() | Qt::ItemIsUserCheckable);
    listItem->setCheckState(Qt::Unchecked);
    listItem = new QListWidgetItem("Показать номера путей", _optionsList);
    listItem->setFlags(Qt::ItemIsUserCheckable | listItem->flags());
    listItem->setCheckState(Qt::Unchecked);
    listItem = new QListWidgetItem("Показать трафик", _optionsList);
    listItem->setFlags(Qt::ItemIsUserCheckable | listItem->flags());
    listItem->setCheckState(Qt::Unchecked);
    listItem = new QListWidgetItem("Показать последний маршрут", _optionsList);
    listItem->setFlags(Qt::ItemIsUserCheckable | listItem->flags());
    listItem->setCheckState(Qt::Unchecked);
    listItem = new QListWidgetItem("Показать узлы", _optionsList);
    listItem->setFlags(Qt::ItemIsUserCheckable | listItem->flags());
    listItem->setCheckState(Qt::Unchecked);

    QSpinBox* numOfCarsSpinBox = new QSpinBox(this);
    numOfCarsSpinBox->setValue(10);
    QPushButton* compareButton = new QPushButton("Сравнить",this);
    connect(compareButton, &QPushButton::clicked, [this, numOfCarsSpinBox]
        {
            compare(numOfCarsSpinBox->value());
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
    timeLayout->addWidget(_optionsList, 10, 0, 1, 2);
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
    _graphicsView->setRenderHint(QPainter::Antialiasing); // Enable smooth rendering
    _graphicsView->setDragMode(QGraphicsView::ScrollHandDrag); // Enable drag mode
    _graphicsView->viewport()->installEventFilter(this);
    _scene->installEventFilter(this);

    mainLayout->addWidget(timeGroupBox);
    mainLayout->addWidget(_graphicsView, 1);

    auto size = _graphicsView->size();

    _mapData.OpenDatabase();
    _args = _mapData.GetArguments();
    _scene->setProjections(&_mapData.projection);
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
    connect(_optionsList, &QListWidget::itemChanged, [this](QListWidgetItem* item)
    {
        int currentRow = _optionsList->row(item);
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
    _mapData.LoadData();
    if (mapPainter.DrawMap(_mapData.projection, _mapData.drawParameter,
                           _mapData.data, _painter))
    {
        _scene->setMap(_pixmap);
    }
}

void MainWindow::paintPoint()
{
    osmscout::GeoCoord pointFrom(55.6565, 41.8260);
    osmscout::Distance dist = pointFrom.GetDistance(osmscout::GeoCoord(55.6876, 42.6846));
    _router->loadDataNodes(_args, dist, pointFrom);
    _router->setupGraphFromNodes();

    //auto& graph = router_->getGraph();
    _graphRef = &_router->getGraph();
    _pathListRef = &_router->getPathList();
    _router->generateDensities(_intervalTime, PlanningMode::HistoricalData);
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

void MainWindow::generateDensities()
{
    // auto gauseFunction = [](unsigned int t, double rPeak1, unsigned int tPeak1, double sPeak1, double rPeak2, unsigned int tPeak2, double sPeak2)
    //     { return rPeak1*exp(-((t-tPeak1)/(2*sPeak1*sPeak1))) +  rPeak2*exp(-((t-tPeak2)/(2*sPeak2*sPeak2))); };
    // int time = 0; // in hours
    // for (auto& node : graph)
    // {

    // }
}

void MainWindow::calculatePath()
{
    auto compare = [](std::pair<int , double> a, std::pair<int , double> b) { return a.second < b.second; };
    std::priority_queue<std::pair<int , double>, std::vector<std::pair<int , double>>, decltype(compare)> openSet(compare);
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
    _mapData.LoadData();
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
    _mapData.LoadData();
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
            //event->accept();
            return true;
        }
    }
    if (object == _scene && e->type() == QGraphicsSceneMouseEvent::GraphicsSceneMousePress)
    {
        auto event = static_cast<QGraphicsSceneMouseEvent*>(e);
        if (event->buttons() & Qt::RightButton)
        {
            //auto localPoint = event->localPos();
            auto widgetPos = event->pos();
            auto viewportSize = _graphicsView->viewport()->size();
            auto viewSize = _graphicsView->size();
            osmscout::GeoCoord coord;
            _scene->projection_->PixelToGeo(event->scenePos().x(), event->scenePos().y(), coord);
            //_scene->projection_->PixelToGeo(localPoint.x() + viewSize.width() - viewportSize.width(),localPoint.y() + viewSize.height() - viewportSize.height(), coord);
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
        /*const auto&*/ _lastRoute = _router->findPathUniversal(398,543,_startTimeLineEdit->text().toInt(),_intervalTime, _planningMode, _algorithm);

    }

    paintMap();

    // _scene->clearMap();
    // _scene->clear();
    // changeMapZoom(1);
    // _scene->paintCurrentTraffic(_graphRef, _pathListRef,_modelingTime,_intervalTime, _planningMode);
    // _scene->paintPath(_graphRef, _lastRoute);
    //scene_->paintAllNodeIndexes(_graphRef);
}

void MainWindow::compare(unsigned int numOfCars)
{
    QRandomGenerator random(1234);
    int size = _graphRef->size();
    std::vector<int> path;
    std::vector<double> travelTimes;
    unsigned int i = 0;
    for (i = 0; i < numOfCars; i++)
    {
        path = _router->findPathUniversal(398,543,_startTimeLineEdit->text().toInt(),_intervalTime, PlanningMode::HistoricalData, _algorithm);
        if(_router->getCongestion())
            break;
        travelTimes.push_back(_router->getTravelTime());
    }
    double avgTravelTimeDefault = std::accumulate(travelTimes.begin(), travelTimes.end(), 0.0)/travelTimes.size();

    _router->pathListConst.clear();
    _router->generateDensities(_intervalTime, PlanningMode::HistoricalData);
    travelTimes.clear();
    for (unsigned int j = 0; j < i; j++)
    {
        path = _router->findPathUniversal(398,543,_startTimeLineEdit->text().toInt(),_intervalTime, PlanningMode::DriverInfluence, _algorithm);
        if(_router->getCongestion())
            break;
        travelTimes.push_back(_router->getTravelTime());
    }
    double avgTravelTimeInfluence = std::accumulate(travelTimes.begin(), travelTimes.end(), 0.0)/travelTimes.size();
    _router->pathListConst.clear();
    _router->generateDensities(_intervalTime, PlanningMode::HistoricalData);

    QMessageBox::information(this, "Title", QString("Default: %1, Influence: %2, NumOfCarsBeforeJam: %3").arg(avgTravelTimeDefault).arg(avgTravelTimeInfluence).arg(i));

}

MainWindow::~MainWindow()
{
}
