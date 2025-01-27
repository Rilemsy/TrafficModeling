#include "GraphicsScene.h"

void GraphicsScene::setMap(QPixmap *value)
{
    pixmapItem_.setPixmap(*value);
    pixmapItem_.setPos(0, 0);
    setBackgroundBrush(Qt::white);
    addItem(&pixmapItem_);
}

void GraphicsScene::clearMap() { QGraphicsScene::removeItem(&pixmapItem_); }
