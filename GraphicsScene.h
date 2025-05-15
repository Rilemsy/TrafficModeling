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

    void    paintDots(std::vector<Node>* graph);
    void    paintPath(std::vector<Node>* graph, const std::vector<int>&);
    void    paintAllPathIndexes(std::vector<Node>* graph, std::vector<Path>* pathList);
    void    paintAllNodeIndexes(std::vector<Node>* graph);
    void    paintCurrentTraffic(std::vector<Node>* graph, std::vector<Path>* pathList, float currentTime, float intervalTime, PlanningMode mode);

    osmscout::MercatorProjection*   _projection;

private:
    QGraphicsPixmapItem             _pixmapItem;
    QGraphicsView*                  _graphicsView;

};

#endif // GRAPHICSSCENE_H
