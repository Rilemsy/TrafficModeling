#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "DrawMap.h"
#include "GraphicsScene.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char *argv[], double screen, QWidget *parent = nullptr);
    ~MainWindow();

    void SetData();

private:
    GraphicsScene *scene_;
    Ui::MainWindow *ui;
    DrawMapDemo MapData_;
    Arguments args_;
    QPainter *painter_;
    QPixmap *pixmap_;
    double zoom = 1;
};
#endif // MAINWINDOW_H
