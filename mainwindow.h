#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLineEdit>

#include "DrawMap.h"
#include "GraphicsScene.h"
#include "Router.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char *argv[], double screen, QWidget *parent = nullptr);
    ~MainWindow();

    void    SetData();
    void    paintPoint();
    void    generateDensities();
    void    calculatePath();
    void    initDensities();
    bool    eventFilter(QObject *object, QEvent *e);
    void    changeMapZoom(double zoomFactor);
    void    placeCars(int amount);



private:
    Router*             router_;
    QGraphicsView*      graphicsView;
    QLineEdit*          _startTimeLineEdit;
    GraphicsScene*      scene_;
    DrawMapDemo         MapData_;
    Arguments           args_;
    QPainter*           painter_;
    QPixmap*            pixmap_;
    std::vector<Node>*  _graphRef;
    std::vector<Path>*  _pathListRef;
    double _zoom = 1;
    PlanningMode        _planningMode = PlanningMode::DriverInfluence;
    Algorithm           _algorithm = Algorithm::AStar;

    unsigned int    _modelingTime = 240;  // в минутах
    double          _intervalTime = 15;
    double          _momentTime = _modelingTime;
};
#endif // MAINWINDOW_H
