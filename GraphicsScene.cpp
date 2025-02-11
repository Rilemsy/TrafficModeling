#include "GraphicsScene.h"

#define BRUSH_SIZE 20

void GraphicsScene::setMap(QPixmap *value)
{
    pixmapItem_.setPixmap(*value);
    pixmapItem_.setPos(0, 0);
    setBackgroundBrush(Qt::white);
    addItem(&pixmapItem_);
}

void GraphicsScene::clearMap() { QGraphicsScene::removeItem(&pixmapItem_); }

void GraphicsScene::paintDots(std::vector<Node>* graph)
{
    for (auto node : *graph)
    {
        QColor color;

        color = QColor(0, 0, 204, 200);

        osmscout::Vertex2D dataFirstDot;
        projection_->GeoToPixel(node.point.GetCoord(), dataFirstDot);
        addEllipse(dataFirstDot.GetX() - BRUSH_SIZE / 4, dataFirstDot.GetY() - 5,
                   BRUSH_SIZE / 2, BRUSH_SIZE / 2, QPen(Qt::NoPen), QBrush(color));
    }
}
