#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <osmscout/projection/MercatorProjection.h>

#include <QMainWindow>
#include <QLineEdit>
#include <QListWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QTextEdit>


#include "Map.h"
#include "GraphicsScene.h"
#include "Router.h"
#include "CustomStructures.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(int argc, char *argv[], double screen, QWidget *parent = nullptr);
    ~MainWindow();

    void    setMap();
    void    initGraph();
    bool    eventFilter(QObject *object, QEvent *e);
    void    changeMapZoom(double zoomFactor);
    void    moveMap(osmscout::GeoCoord coord);
    void    paintMap();
    void    addRoutes(unsigned int numOfCars);

private:
    Router*             _router;
    QGraphicsView*      _graphicsView;
    GraphicsScene*      _scene;
    Map                 _map;
    Arguments           _args;
    QPainter*           _painter;
    QPixmap*            _pixmap;
    std::vector<Node>*  _nodeListRef;
    std::vector<Path>*  _pathListRef;
    double              _zoom = 1;
    std::vector<int>    _lastRoute;

    QCheckBox*          _showTrafficCheckBox;
    QCheckBox*          _showLastRouteCheckBox;
    QCheckBox*          _showNodesCheckBox;
    QCheckBox*          _showNodesIndexCheckBox;
    QCheckBox*          _updateDensitiesCheckBox;
    QComboBox*          _algorithmComboBox;
    QLineEdit*          _weightLineEdit;
    QLineEdit*          _startNodeLineEdit;
    QLineEdit*          _targetNodeLineEdit;
    QTextEdit*          _resultTextEdit;
    QCheckBox*          _loadCheckBox;

    unsigned int    _modelingTime = 0;  // в секундах
    unsigned int    _intervalTime = 10;
};
#endif // MAINWINDOW_H
