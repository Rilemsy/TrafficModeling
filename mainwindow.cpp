#include <osmscout/db/Database.h>
#include <osmscout/feature/ConstructionYearFeature.h>
#include <osmscoutmap/MapService.h>
#include <osmscoutmapqt/MapPainterQt.h>

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

#include <queue>
#include <unordered_set>

#include "mainwindow.h"

MainWindow::MainWindow(int argc, char *argv[], double screen, QWidget *parent)
    : QMainWindow(parent),
    MapData_("QMap", argc, argv, screen)
{
    //ui->setupUi(this);

    router_ = new Router();
    QFile file("output.txt");
    if (file.exists())
        file.remove();

    scene_ = new GraphicsScene();
    auto screenGeometry = QApplication::primaryScreen()->geometry();
    //scene_->setSceneRect(0, 0, 800, 600);
    scene_->setSceneRect(0, 0, screenGeometry.width(), screenGeometry.height());

    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);


    graphicsView = new QGraphicsView(scene_, this);
    //setCentralWidget(graphicsView);
    graphicsView->setRenderHint(QPainter::Antialiasing); // Enable smooth rendering
    graphicsView->setDragMode(QGraphicsView::ScrollHandDrag); // Enable drag mode
    graphicsView->viewport()->installEventFilter(this);

    QGroupBox* timeGroupBox = new QGroupBox(this);
    QGridLayout* timeLayout = new QGridLayout;
    QLabel* modelingTimeLabel = new QLabel("Время: " + (QString::number(_modelingTime)),timeGroupBox);
    QLabel* startTimeLabel = new QLabel("Время начала поездки:");
    _startTimeLineEdit = new QLineEdit(QString::number(_modelingTime),this);
    //currentTimeLineEdit->setPlaceholderText("Время начала поездки");
    QLabel* timeIntervalLabel = new QLabel("Интервал времени:");
    QLineEdit* timeIntervalLineEdit = new QLineEdit(QString::number(_intervalTime),this);
    //timeIntervalLineEdit->setPlaceholderText("Интервал времени");  // в минутах
    QPushButton* increaseTimeButton = new QPushButton("Увеличить время",this);

    // modelingTimeLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    // currentTimeLineEdit->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    // timeIntervalLineEdit->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    timeLayout->addWidget(startTimeLabel, 1, 0);
    timeLayout->addWidget(timeIntervalLabel, 2, 0);

    timeLayout->addWidget(_startTimeLineEdit,1,1);
    timeLayout->addWidget(timeIntervalLineEdit,2,1);
    timeLayout->addWidget(modelingTimeLabel,3,0,1,2);
    timeLayout->addWidget(increaseTimeButton,4,0,1,2);
    timeLayout->setColumnStretch(timeLayout->columnCount(),1);
    timeLayout->setRowStretch(timeLayout->rowCount(),1);
    //timeLayout->addStretch(1);

    timeGroupBox->setLayout(timeLayout);

    mainLayout->addWidget(timeGroupBox);
    mainLayout->addWidget(graphicsView,1);

    MapData_.OpenDatabase();
    args_ = MapData_.GetArguments();
    scene_->setProjections(&MapData_.projection);
    pixmap_ = new QPixmap(static_cast<int>(args_.width),
                          static_cast<int>(args_.height));
    painter_ = new QPainter(pixmap_);

    connect(increaseTimeButton, &QPushButton::clicked, [this, modelingTimeLabel]
    {
        _modelingTime += _intervalTime;
        modelingTimeLabel->setText("Время: " + QString::number(_modelingTime));
    });

}

void MainWindow::SetData()
{
    osmscout::MapPainterQt mapPainter(MapData_.styleConfig);
    osmscout::TypeInfoRef buildingType =
        MapData_.database->GetTypeConfig()->GetTypeInfo("building");
    MapData_.LoadData();
    if (mapPainter.DrawMap(MapData_.projection, MapData_.drawParameter,
                           MapData_.data, painter_))
    {
        scene_->setMap(pixmap_);
    }
}

void MainWindow::paintPoint()
{
    osmscout::GeoCoord pointFrom(55.51882, 42.07721);
    osmscout::Distance dist = pointFrom.GetDistance(osmscout::GeoCoord(55.50585, 42.06659));
    router_->LoadDataNodes(args_, dist, pointFrom);
    router_->SetupGraphFromNodes();

    //auto& graph = router_->getGraph();
    _graphRef = &router_->getGraph();
    _pathListRef = &router_->getPathList();
    router_->generateDensities(15);
    scene_->paintDots(_graphRef);


    //const auto& path = router_->findPathAStar(5,98);
    // const auto& path = router_->findPathAStarTime(5,98,_startTimeLineEdit->text().toInt(),_intervalTime);
    // scene_->paintPath(_graphRef, path);
    placeCars(24);
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

void MainWindow::setMapZoom(double zoom)
{
    scene_->clearMap();
    args_.zoom.SetMagnification(args_.zoom.GetMagnification() * zoom);
    MapData_.projection.Set(args_.center, args_.angle.AsRadians(), args_.zoom,
                            args_.dpi, args_.width, args_.height);
    delete painter_;
    delete pixmap_;
    pixmap_ = new QPixmap(static_cast<int>(args_.width),
                          static_cast<int>(args_.height));
    painter_ = new QPainter(pixmap_);
    osmscout::MapPainterQt mapPainter(MapData_.styleConfig);
    MapData_.LoadData();
    if (mapPainter.DrawMap(MapData_.projection, MapData_.drawParameter,
                           MapData_.data, painter_))
    {
        scene_->setMap(pixmap_);
    }
}

bool MainWindow::eventFilter(QObject *object, QEvent *e)
{
    if (object == graphicsView->viewport() && e->type() == QEvent::Wheel)
    {
        auto event = static_cast<QWheelEvent *>(e);
        if (event->modifiers().testFlag(Qt::ControlModifier))
        {
            double scaleZoom = 1.15;
            zoom *= scaleZoom;
            if (event->angleDelta().y() > 0)
            {
                setMapZoom(scaleZoom);
            }
            else if (event->angleDelta().y() < 0)
            {
                setMapZoom(1.0 / scaleZoom);
            }
            //event->accept();
            return true;
        }
    }
    return false;
}

void MainWindow::placeCars(int amount)
{
    QRandomGenerator random(1234);
    int size = _graphRef->size();

    for (int i = 0; i < amount; i++)
    {
        const auto& path = router_->findPathAStarTime(random.bounded(0, size),random.bounded(0, size),_startTimeLineEdit->text().toInt(),_intervalTime);
        scene_->paintPath(_graphRef, path);
    }
    scene_->paintAllPathIndexes(_graphRef, _pathListRef);
    scene_->paintAllNodeIndexes(_graphRef);
}

MainWindow::~MainWindow()
{
}
