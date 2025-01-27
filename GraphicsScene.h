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

class GraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit GraphicsScene(QObject *parent = 0) : QGraphicsScene(parent)
    {
        setBackgroundBrush(Qt::white);
    };
    ~GraphicsScene(){};

    void setMap(QPixmap *value);
    void setView(QGraphicsView *view) { view_ = view; }
    void clearMap();
    void setProjections(osmscout::MercatorProjection *projection)
    {
        projection_ = projection;
    }

private:
    QGraphicsPixmapItem pixmapItem_;
    QGraphicsView *view_;
    osmscout::MercatorProjection *projection_;
};

#endif // GRAPHICSSCENE_H
