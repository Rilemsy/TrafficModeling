TEMPLATE = app
TARGET = name_of_the_app


QT = core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

INCLUDEPATH += "/usr/local/include"

LIBS += -L"/usr/local/lib" -losmscoutd -losmscout_mapd -losmscout_map_qtd -losmscout_client_qtd

HEADERS += \
    DrawMap.h

SOURCES += \
    main.cpp
