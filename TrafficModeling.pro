QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

INCLUDEPATH += "/usr/local/include"

LIBS += -L"/usr/local/lib" -losmscoutd -losmscout_mapd -losmscout_map_qtd

SOURCES += \
    GraphicsScene.cpp \
    Router.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    CustomStructures.h \
    GraphicsScene.h \
    Map.h \
    Router.h \
    mainwindow.h
