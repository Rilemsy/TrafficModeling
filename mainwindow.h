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

    void SetData();
    void paintPoint();
    void generateDensities();
    void calculatePath();
    void initDensities();
    bool eventFilter(QObject *object, QEvent *e);
    void setMapZoom(double zoom);
    void placeCars(int amount);



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
    double zoom = 1;

    unsigned int    _modelingTime = 240;  // в минутах
    double          _intervalTime = 15;
};
#endif // MAINWINDOW_H
