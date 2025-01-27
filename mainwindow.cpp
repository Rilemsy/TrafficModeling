#include <osmscout/db/Database.h>
#include <osmscout/feature/ConstructionYearFeature.h>
#include <osmscoutmap/MapService.h>
#include <osmscoutmapqt/MapPainterQt.h>

#include <QGuiApplication>

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char *argv[], double screen, QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    MapData_("QMap", argc, argv, screen)
{
    ui->setupUi(this);
    scene_ = new GraphicsScene();
    //ui->graphicsView->setScene(scene_);
    //ui->graphicsView->viewport()->installEventFilter(this);
    //scene_->setView(ui->graphicsView);
    MapData_.OpenDatabase();
}

MainWindow::~MainWindow()
{
    delete ui;
}
