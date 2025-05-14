#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <osmscout/projection/MercatorProjection.h>

#include <QMainWindow>
#include <QLineEdit>
#include <QListWidget>

#include "DrawMap.h"
#include "GraphicsScene.h"
#include "Router.h"
#include "CustomStructures.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(int argc, char *argv[], double screen, QWidget *parent = nullptr);
    ~MainWindow();

    void    setData();
    void    init();
    void    generateDensities();
    void    calculatePath();
    void    initDensities();
    bool    eventFilter(QObject *object, QEvent *e);
    void    changeMapZoom(double zoomFactor);
    void    placeCars(int amount);
    void    moveMap(osmscout::GeoCoord coord);
    void    paintMap();
    void    runSimulation(unsigned int numOfCars);

private:
    Router*             _router;
    QGraphicsView*      _graphicsView;
    QLineEdit*          _startTimeLineEdit;
    GraphicsScene*      _scene;
    QListWidget*        _optionsList;
    DrawMap         _mapData;
    Arguments           _args;
    QPainter*           _painter;
    QPixmap*            _pixmap;
    std::vector<Node>*  _graphRef;
    std::vector<Path>*  _pathListRef;
    double _zoom = 1;
    PlanningMode        _planningMode = PlanningMode::DriverInfluence;
    Algorithm           _algorithm = Algorithm::AStar;
    Options             _options;
    std::vector<int>    _lastRoute;

    unsigned int    _modelingTime = 0;  // в секундах
    unsigned int    _intervalTime = 30;
    double          _momentTime = _modelingTime;
};
#endif // MAINWINDOW_H
