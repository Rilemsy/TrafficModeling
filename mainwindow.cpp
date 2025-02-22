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

#include <queue>
#include <unordered_set>

#include "mainwindow.h"

MainWindow::MainWindow(int argc, char *argv[], double screen, QWidget *parent)
    : QMainWindow(parent),
    MapData_("QMap", argc, argv, screen)
{
    //ui->setupUi(this);

    router_ = new Router();

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

    QGroupBox* timeGroupBox = new QGroupBox(this);
    QGridLayout* timeLayout = new QGridLayout;
    QLabel* modelingTimeLabel = new QLabel("Время: " + (QString::number(_modelingTime)),timeGroupBox);
    QLabel* startTimeLabel = new QLabel("Время начала поездки:");
    QLineEdit* currentTimeLineEdit = new QLineEdit(this);
    //currentTimeLineEdit->setPlaceholderText("Время начала поездки");
    QLabel* timeIntervalLabel = new QLabel("Интервал времени:");
    QLineEdit* timeIntervalLineEdit = new QLineEdit(this);
    //timeIntervalLineEdit->setPlaceholderText("Интервал времени");  // в минутах
    QPushButton* increaseTimeLineEdit = new QPushButton("Увеличить время",this);

    // modelingTimeLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    // currentTimeLineEdit->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    // timeIntervalLineEdit->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    timeLayout->addWidget(startTimeLabel, 1, 0);
    timeLayout->addWidget(timeIntervalLabel, 2, 0);


    timeLayout->addWidget(currentTimeLineEdit,1,1);
    timeLayout->addWidget(timeIntervalLineEdit,2,1);
    timeLayout->addWidget(modelingTimeLabel,3,0,1,2);
    timeLayout->addWidget(increaseTimeLineEdit,4,0,1,2);
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
    graphRef = &router_->getGraph();
    pathListRef = &router_->getPathList();
    router_->generateDensities();
    scene_->paintDots(graphRef);


    const auto& path = router_->findPathAStar(5,98);
    scene_->paintPath(graphRef, path);
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

MainWindow::~MainWindow()
{
}
