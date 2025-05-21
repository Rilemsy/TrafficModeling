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

    QGroupBox* userGroupBox = new QGroupBox(this);
    QGridLayout* userLayout = new QGridLayout;

    QLabel* modelingTimeLabel = new QLabel("Время: " + (QString::number(_modelingTime)),userGroupBox);
    QLabel* timeIntervalLabel = new QLabel("Интервал дискретизации:");
    QLineEdit* timeIntervalLineEdit = new QLineEdit(QString::number(_intervalTime),this);
    QPushButton* increaseTimeButton = new QPushButton("Увеличить время",this);
    QPushButton* decreaseTimeButton = new QPushButton("Уменьшить время",this);
    QHBoxLayout* timeLayout = new QHBoxLayout;
    timeLayout->addWidget(increaseTimeButton);
    timeLayout->addWidget(decreaseTimeButton);
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
    _loadCheckBox = new QCheckBox("Учет влияния построенных маршрутов", this);
    _updateDensitiesCheckBox = new QCheckBox("Обновление загруженности дорог", this);

    QLabel* algorithmNameLabel = new QLabel("Алгоритм:", this);
    _algorithmComboBox = new QComboBox(this);
    _algorithmComboBox->insertItems(0,{"Алгоритм Дейкстры", "A*", "Weighted A*", "Алгоритм Беллмана-Форда"});
    _algorithmComboBox->setCurrentIndex(0);
    QLabel* weigthLabel = new QLabel("Вес алгоритма Weighted A*:");
    _weightLineEdit = new QLineEdit(QString::number(1),this);
    QPushButton* resetButton = new QPushButton("Вернуть дороги с исходному состоянию",this);

    QGroupBox* optionsGroupBox = new QGroupBox("Опции", this);
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
    QLabel* resultLabel = new QLabel("Результат:",this);
    _resultTextEdit = new QTextEdit(this);

    userLayout->addWidget(modelingTimeLabel, 1, 0, 1, 2);
    userLayout->addLayout(timeLayout, 2, 0, 1, 2);
    userLayout->addWidget(timeIntervalLabel, 3, 0);
    userLayout->addWidget(timeIntervalLineEdit, 3, 1);
    userLayout->addWidget(algorithmNameLabel, 5, 0);
    userLayout->addWidget(_algorithmComboBox, 5, 1);
    userLayout->addWidget(weigthLabel, 6, 0);
    userLayout->addWidget(_weightLineEdit, 6, 1);
    userLayout->addWidget(_loadCheckBox, 7, 0, 1, 2);
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

    userGroupBox->setLayout(userLayout);


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

    mainLayout->addWidget(userGroupBox);
    mainLayout->addWidget(_graphicsView, 1);

    _map.openDatabase();
    _args = _map.GetArguments();
    _router->setDatabase(_map.database);
    _scene->setProjection(&_map.projection);
    _pixmap = new QPixmap(static_cast<int>(_args.width),
                          static_cast<int>(_args.height));
    _painter = new QPainter(_pixmap);
    //painter_->setBackground(QBrush(Qt::white));



    connect(resetButton, &QPushButton::clicked, [this]
            {
                _router->setIntervalTime(_intervalTime);
                _router->initDensities(_intervalTime);
            });

    connect(runButton, &QPushButton::clicked, [this, numOfCarsSpinBox]
            {
                _router->setIntervalTime(_intervalTime);
                _router->addRoutes(numOfCarsSpinBox->value(), _weightLineEdit->text().toFloat() , _loadCheckBox->isChecked(),
                                   _updateDensitiesCheckBox->isChecked(), static_cast<Algorithm>(_algorithmComboBox->currentIndex()));
                QMessageBox::information(this, "Информация","Выполнение завершено.");
            });

    connect(timeIntervalLineEdit, &QLineEdit::editingFinished, [this, timeIntervalLineEdit]
            {
                _intervalTime = timeIntervalLineEdit->text().toInt();
                _router->setIntervalTime(_intervalTime);
            });

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
            route = _router->findPath(_startNodeLineEdit->text().toInt(), _targetNodeLineEdit->text().toInt(), _modelingTime,
                _weightLineEdit->text().toFloat() , _loadCheckBox->isChecked(), _updateDensitiesCheckBox->isChecked(), static_cast<Algorithm>(_algorithmComboBox->currentIndex()));
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
            QString result = QString("Время поездки: %1 с.\nВремя выполнения: %2 с.\nКоличество посещенных узлов: %3").arg(route.travelTime).arg(route.execTime).arg(route.visitedNodeCount);
            _resultTextEdit->setText(result);
            paintMap();
        }
        else
            QMessageBox::information(this,"Информация", "Маршрут не найден.");
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
    _nodeListRef = &_router->getNodeList();
    _pathListRef = &_router->getPathList();
    _scene->setNodeList(_nodeListRef);
    _scene->setPathList(_pathListRef);
    _router->initDensities(_intervalTime);
    _router->setIntervalTime(_intervalTime);

    QIntValidator* nodeValidator = new QIntValidator(0, _nodeListRef->size()-1, this);
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

MainWindow::~MainWindow()
{
}
