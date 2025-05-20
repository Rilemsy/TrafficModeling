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
    void    initDensities();
    bool    eventFilter(QObject *object, QEvent *e);
    void    changeMapZoom(double zoomFactor);
    Route   findPath(int startNodeIndex, int endNodeIndex, int startTime, bool densityUpdate);
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
    std::vector<Node>*  _graphRef;
    std::vector<Path>*  _pathListRef;
    double              _zoom = 1;
    std::vector<int>    _lastRoute;

    QCheckBox*          _showTrafficCheckBox;
    QCheckBox*          _showLastRouteCheckBox;
    QCheckBox*          _showNodesCheckBox;
    QCheckBox*          _showNodesIndexCheckBox;
    QCheckBox*          _updateDensitiesCheckBox;
    QComboBox*          _weightTypeComboBox;
    QComboBox*          _algorithmComboBox;
    QLineEdit*          _weightLineEdit;
    QLineEdit*          _startNodeLineEdit;
    QLineEdit*          _targetNodeLineEdit;
    QTextEdit*          _resultTextEdit;

    unsigned int    _modelingTime = 0;  // в секундах
    unsigned int    _intervalTime = 15;
};
#endif // MAINWINDOW_H
