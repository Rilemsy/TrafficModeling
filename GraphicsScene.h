#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H

#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <iostream>

#include <osmscout/projection/MercatorProjection.h>

#include "CustomStructures.h"

class GraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit GraphicsScene(QObject *parent = 0) : QGraphicsScene(parent)
    {
        setBackgroundBrush(Qt::white);
    };
    ~GraphicsScene(){};

    void    setMap(QPixmap *value);
    void    setView(QGraphicsView *view) { _graphicsView = view; }
    void    clearMap();
    void    setProjection(osmscout::MercatorProjection *projection)
    {
        _projection = projection;
    }
    void    setGraph(std::vector<Node>*  graph)
    {
        _graphRef = graph;
    }
    void    setPathList(std::vector<Path>*  pathList)
    {
        _pathListRef = pathList;
    }


    void    paintNodes();
    void    paintPath(const std::vector<int>&);
    void    paintAllPathIndexes();
    void    paintAllNodeIndexes();
    void    paintCurrentTraffic(float currentTime, float intervalTime);

    osmscout::MercatorProjection*   _projection;

private:
    QGraphicsPixmapItem             _pixmapItem;
    QGraphicsView*                  _graphicsView;
    std::vector<Node>*              _graphRef;
    std::vector<Path>*              _pathListRef;
    short int                       _dotSize = 16;

};

#endif // GRAPHICSSCENE_H
