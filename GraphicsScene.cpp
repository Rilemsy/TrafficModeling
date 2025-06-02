#include "GraphicsScene.h"

#include <QGraphicsTextItem>
#include <QGraphicsItem>
#include <QMessageBox>

void GraphicsScene::setMap(QPixmap *value)
{
    _pixmapItem.setPixmap(*value);
    _pixmapItem.setPos(0, 0);
    setBackgroundBrush(Qt::white);
    addItem(&_pixmapItem);
}

void GraphicsScene::clearMap() {
    if (_pixmapItem.scene())
        QGraphicsScene::removeItem(&_pixmapItem);
}

void GraphicsScene::paintNodes()
{
    for (const auto& node : *_nodeListRef)
    {
        QColor color;

        color = QColor(0, 0, 204, 200);

        osmscout::Vertex2D dot;
        _projection->GeoToPixel(node.point.GetCoord(), dot);
        addEllipse(dot.GetX() - _dotSize / 2, dot.GetY() - 5, _dotSize, _dotSize, QPen(Qt::NoPen), QBrush(color));
    }
}

void GraphicsScene::paintPath(const std::vector<int>& indexes)
{
    QPair<qreal , qreal > prevNodePixels;
    bool notFirstNodeFlag = false;
    int currentNode = 0;
    double pathLength = 0;
    for (auto i : indexes)
    {
        QColor color;

        color = QColor(204, 0, 0, 200);

        osmscout::Vertex2D dot;
        _projection->GeoToPixel((*_nodeListRef)[i].point.GetCoord(), dot);
        addEllipse(dot.GetX() - _dotSize / 2, dot.GetY() - 5, _dotSize, _dotSize, QPen(Qt::NoPen), QBrush(color));
        QPen pen(QColor(255,0,0));
        pen.setWidth(5);
        if (notFirstNodeFlag)
        {
            addLine(prevNodePixels.first,prevNodePixels.second,dot.GetX() - _dotSize / 2 + _dotSize / 2, dot.GetY() - 5 + _dotSize / 2, pen);
            auto currentPathLength = (*_nodeListRef)[indexes[currentNode]].point.GetCoord().GetDistance((*_nodeListRef)[indexes[currentNode-1]].point.GetCoord()).AsMeter() / 1000.0;
            pathLength += currentPathLength;
        }
        prevNodePixels.first = dot.GetX();
        prevNodePixels.second = dot.GetY() - 5 + _dotSize / 2;
        notFirstNodeFlag = true;
        currentNode++;


    }
}

void GraphicsScene::paintAllPathIndexes()
{
    int i = 0;
    for (const auto& path : *_pathListRef)
    {
        osmscout::Vertex2D firstDot;
        _projection->GeoToPixel((*_nodeListRef)[path.startNodeIndex].point.GetCoord(), firstDot);
        osmscout::Vertex2D secondDot;
        _projection->GeoToPixel((*_nodeListRef)[path.targetNodeIndex].point.GetCoord(), secondDot);
        osmscout::Vertex2D resultDot((firstDot.GetX()+secondDot.GetX())/2, (firstDot.GetY()+secondDot.GetY())/2);

        QGraphicsTextItem *text = addText(QString::number(i));
        if (path.startNodeIndex < path.targetNodeIndex)
        {
            text->setPos(resultDot.GetX() -  35, resultDot.GetY()+ 5 - _dotSize / 2);
        }
        else
            text->setPos(resultDot.GetX(), resultDot.GetY()- 5 + _dotSize / 2);
        i++;
    }
}

void GraphicsScene::paintAllNodeIndexes()
{
    int i = 0;
    for (const auto& node : *_nodeListRef)
    {
        osmscout::Vertex2D dot;
        _projection->GeoToPixel(node.point.GetCoord(), dot);
        QGraphicsTextItem *text = addText(QString::number(i));
        text->setDefaultTextColor(QColor(0,0,0));
        text->setPos(dot.GetX(), dot.GetY()- 5 + _dotSize / 2);
        i++;
    }
}

void GraphicsScene::paintCurrentTraffic(float currentTime, float intervalTime)
{
    for (const auto& path : *_pathListRef)
    {
        QColor color;

        osmscout::Vertex2D firstDot;
        _projection->GeoToPixel((*_nodeListRef)[path.startNodeIndex].point.GetCoord(), firstDot);

        osmscout::Vertex2D secondDot;
        _projection->GeoToPixel((*_nodeListRef)[path.targetNodeIndex].point.GetCoord(), secondDot);
        osmscout::Vertex2D resultDot((firstDot.GetX()+secondDot.GetX())/2, (firstDot.GetY()+secondDot.GetY())/2);

        float density = path.densities[std::floor(currentTime/intervalTime)]/(path.distanceLength.AsMeter()/1000);
        if (density < 50)
            color = QColor(0,255,0);
        else if (density < 80)
            color = QColor(255,255,0);
        else if (density < 120)
            color = QColor(255,128,0);
        else
            color = QColor(255,0,0);

        QPen pen(color);
        pen.setWidth(3);

        addLine(firstDot.GetX(), firstDot.GetY() - 5 + _dotSize / 2, secondDot.GetX(), secondDot.GetY() - 5 + _dotSize / 2, pen);
    }
}
