#include "mainwindow.h"

#include <QScreen>
#include <QGuiApplication>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv,true);
    MainWindow w(argc, argv,
                 QGuiApplication::primaryScreen()->physicalDotsPerInch());;
    w.SetData();
    w.show();
    return a.exec();
}
