#include <osmscout/db/Database.h>
#include <osmscout/feature/ConstructionYearFeature.h>
#include <osmscoutmap/MapService.h>
#include <osmscoutmapqt/MapPainterQt.h>

#include <QGuiApplication>

#include "mainwindow.h"

MainWindow::MainWindow(int argc, char *argv[], double screen, QWidget *parent)
    : QMainWindow(parent),
    MapData_("QMap", argc, argv, screen)
{
    //ui->setupUi(this);
    scene_ = new GraphicsScene();

    scene_->setSceneRect(0, 0, 800, 600);
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

MainWindow::~MainWindow()
{
}
