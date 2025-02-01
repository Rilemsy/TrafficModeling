#include <osmscout/db/Database.h>
#include <osmscout/feature/ConstructionYearFeature.h>
#include <osmscoutmap/MapService.h>
#include <osmscoutmapqt/MapPainterQt.h>

#include <QGuiApplication>
#include <QApplication>
#include <QScreen>

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


    ///scene_->addRect(100, 100, 200, 150, QPen(Qt::black), QBrush(Qt::blue));
    //scene_->addEllipse(300, 200, 100, 100, QPen(Qt::red), QBrush(Qt::green));
    //scene_->addText("Hello, QGraphicsView!");
    //ui->graphicsView->setScene(scene_);
    //ui->graphicsView->viewport()->installEventFilter(this);
    //scene_->setView(ui->graphicsView);

    graphicsView = new QGraphicsView(scene_, this);
    setCentralWidget(graphicsView);
    graphicsView->setRenderHint(QPainter::Antialiasing); // Enable smooth rendering
    graphicsView->setDragMode(QGraphicsView::ScrollHandDrag); // Enable drag mode

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
    osmscout::GeoCoord pointFrom(55.541053, 42.082765);
    osmscout::Distance dist = pointFrom.GetDistance(osmscout::GeoCoord(55.54236, 42.06341));
    router_->LoadDataNodes(args_, dist, pointFrom);
    router_->SetupGraphFromNodes();

    auto& graph = router_->getGraph();
    scene_->paintDots(graph);


}

void MainWindow::generateDensities(const std::vector<Node> &graph)
{
    // auto gauseFunction = [](unsigned int t, double rPeak1, unsigned int tPeak1, double sPeak1, double rPeak2, unsigned int tPeak2, double sPeak2)
    //     { return rPeak1*exp(-((t-tPeak1)/(2*sPeak1*sPeak1))) +  rPeak2*exp(-((t-tPeak2)/(2*sPeak2*sPeak2))); };
    // int time = 0; // in hours
    // for (auto& node : graph)
    // {

    // }
}


MainWindow::~MainWindow()
{
}
