#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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

private:
    Router *router_;
    QGraphicsView *graphicsView;
    GraphicsScene *scene_;
    DrawMapDemo MapData_;
    Arguments args_;
    QPainter *painter_;
    QPixmap *pixmap_;
    std::vector<Node>* graphRef;
    std::vector<Path>* pathListRef;
    double zoom = 1;

    unsigned int _modelingTime = 0;  // в минутах

};
#endif // MAINWINDOW_H
