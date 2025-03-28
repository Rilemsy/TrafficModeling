#include "mainwindow.h"

#include <QScreen>
#include <QGuiApplication>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv,true);
    MainWindow w(argc, argv,
                 QGuiApplication::primaryScreen()->physicalDotsPerInch());;
    w.setWindowState(Qt::WindowMaximized);
    w.SetData();
    w.paintPoint();
    w.show();
    return a.exec();
}
