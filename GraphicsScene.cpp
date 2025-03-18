#include "GraphicsScene.h"

#include <QGraphicsTextItem>
#include <QGraphicsItem>

#include <iomanip>

#include <QMessageBox>

#define BRUSH_SIZE 20

void GraphicsScene::setMap(QPixmap *value)
{
    pixmapItem_.setPixmap(*value);
    pixmapItem_.setPos(0, 0);
    setBackgroundBrush(Qt::white);
    addItem(&pixmapItem_);
}

void GraphicsScene::clearMap() {
    if (pixmapItem_.scene())
        QGraphicsScene::removeItem(&pixmapItem_);
}

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

void GraphicsScene::paintPath(std::vector<Node>* graph, const std::vector<int>& indexes)
{
    QPair<qreal , qreal > prevNodePixels;
    bool notFirstNodeFlag = false;
    int currentNode = 0;
    double pathLength = 0;
    for (auto i : indexes)
    {
        QColor color;

        color = QColor(204, 0, 0, 200);

        osmscout::Vertex2D dataFirstDot;
        projection_->GeoToPixel((*graph)[i].point.GetCoord(), dataFirstDot);
        addEllipse(dataFirstDot.GetX() - BRUSH_SIZE / 4, dataFirstDot.GetY() - 5,
                   BRUSH_SIZE / 2, BRUSH_SIZE / 2, QPen(Qt::NoPen), QBrush(color));
        QPen pen(QColor(255,0,0));
        pen.setWidth(5);
        if (notFirstNodeFlag)
        {
            addLine(prevNodePixels.first,prevNodePixels.second,dataFirstDot.GetX() - BRUSH_SIZE / 4 + BRUSH_SIZE / 4,
                    dataFirstDot.GetY() - 5 + BRUSH_SIZE / 4, pen);
            auto currentPathLength = (*graph)[indexes[currentNode]].point.GetCoord().GetDistance((*graph)[indexes[currentNode-1]].point.GetCoord()).AsMeter() / 1000.0;
            pathLength += currentPathLength;
            //std::cout << "Current pathLength: " << currentPathLength << std::endl;
        }
        prevNodePixels.first = dataFirstDot.GetX() - BRUSH_SIZE / 4 + BRUSH_SIZE / 4;
        prevNodePixels.second = dataFirstDot.GetY() - 5 + BRUSH_SIZE / 4;
        notFirstNodeFlag = true;
        currentNode++;


    }

    std::cout << "\nPath Length: " << pathLength << std::endl;
}

void GraphicsScene::paintAllPathIndexes(std::vector<Node>* graph, std::vector<Path>* pathList)
{
    int i = 0;
    for (const auto& path : *pathList)
    {
        osmscout::Vertex2D dataFirstDot;
        projection_->GeoToPixel((*graph)[path.startNodeIndex].point.GetCoord(), dataFirstDot);
        osmscout::Vertex2D dataSecondDot;
        projection_->GeoToPixel((*graph)[path.targetNodeIndex].point.GetCoord(), dataSecondDot);
        osmscout::Vertex2D dataResultDot((dataFirstDot.GetX()+dataSecondDot.GetX())/2,
                                         (dataFirstDot.GetY()+dataSecondDot.GetY())/2);
        //addText("Text")
        QGraphicsTextItem *text = addText(QString::number(i));
        if (path.startNodeIndex < path.targetNodeIndex)
        {
            text->setPos(dataResultDot.GetX() -  35, dataResultDot.GetY()+ 5 - BRUSH_SIZE / 4);
        }
        else
            text->setPos(dataResultDot.GetX(), dataResultDot.GetY()- 5 + BRUSH_SIZE / 4);
        i++;
    }
}

void GraphicsScene::paintAllNodeIndexes(std::vector<Node>* graph)
{
    int i = 0;
    for (const auto& node : *graph)
    {
        osmscout::Vertex2D dataFirstDot;
        projection_->GeoToPixel(node.point.GetCoord(), dataFirstDot);
        QGraphicsTextItem *text = addText(QString::number(i));
        text->setDefaultTextColor(QColor(255,0,0));
        text->setPos(dataFirstDot.GetX(), dataFirstDot.GetY()- 5 + BRUSH_SIZE / 4);
        i++;
    }
}

void GraphicsScene::paintCurrentTraffic(std::vector<Node>* graph, std::vector<Path>* pathList, float currentTime, float intervalTime, PlanningMode mode)
{
    QPair<qreal , qreal > prevNodePixels;

    for (const auto& path : *pathList)
    {
        QColor color;

        //color = QColor(204, 0, 0, 200);

        osmscout::Vertex2D dataFirstDot;
        projection_->GeoToPixel((*graph)[path.startNodeIndex].point.GetCoord(), dataFirstDot);

        // addEllipse(dataFirstDot.GetX() - BRUSH_SIZE / 4, dataFirstDot.GetY() - 5,
        //            BRUSH_SIZE / 2, BRUSH_SIZE / 2, QPen(Qt::NoPen), QBrush(color));



        osmscout::Vertex2D dataSecondDot;
        projection_->GeoToPixel((*graph)[path.targetNodeIndex].point.GetCoord(), dataSecondDot);
        osmscout::Vertex2D dataResultDot((dataFirstDot.GetX()+dataSecondDot.GetX())/2,
                                         (dataFirstDot.GetY()+dataSecondDot.GetY())/2);


        float density = path.densities[std::floor(currentTime/intervalTime)];
        if (density < 20)
            color = QColor(0,255,0);
        else if (density < 70)
            color = QColor(255,255,0);
        else if (density < 120)
            color = QColor(255,128,0);
        else
            color = QColor(255,0,0);

        if (density >= 120)
        {
            addEllipse(dataResultDot.GetX(), dataResultDot.GetY()- 5 + BRUSH_SIZE / 4,
                       BRUSH_SIZE / 2, BRUSH_SIZE / 2, QPen(Qt::NoPen), QBrush(color));
            if (mode == PlanningMode::OnlyDistance)
            {
                QMessageBox::warning(view_, "Warning", "Density >= 120");
            }
        }
        else
        {
            // QGraphicsTextItem *text = addText(QString::number(density, 'f', 2));
            // text->setDefaultTextColor(QColor(255,0,0));
            // if (path.startNodeIndex < path.targetNodeIndex)
            // {
            //     text->setPos(dataResultDot.GetX() -  35, dataResultDot.GetY()+ 5 - BRUSH_SIZE / 4);
            // }
            // else
            //     text->setPos(dataResultDot.GetX(), dataResultDot.GetY()- 5 + BRUSH_SIZE / 4);
        }
        //text->setPos(dataFirstDot.GetX(), dataFirstDot.GetY()- 5 + BRUSH_SIZE / 4);



        QPen pen(color);
        pen.setWidth(3);

        addLine(dataFirstDot.GetX() - BRUSH_SIZE / 4 + BRUSH_SIZE / 4, dataFirstDot.GetY() - 5 + BRUSH_SIZE / 4,
                dataSecondDot.GetX() - BRUSH_SIZE / 4 + BRUSH_SIZE / 4, dataSecondDot.GetY() - 5 + BRUSH_SIZE / 4, pen);
    }
}
