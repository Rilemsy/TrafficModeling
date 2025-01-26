#include <DrawMap.h>

#include <iostream>

#include <QApplication>
#include <QPixmap>
#include <QScreen>
#include <QGuiApplication>

#include <osmscout/db/Database.h>

#include <osmscoutmap/MapService.h>

#include <osmscout/feature/ConstructionYearFeature.h>

#include <osmscoutmapqt/MapPainterQt.h>

int main(int argc, char **argv)
{
    QApplication app (argc, argv,true);
    // QPushButton button ("Hello world !");
    // button.show();
    // return app.exec();

    assert(QGuiApplication::primaryScreen());
    DrawMapDemo drawDemo("DrawMapQt", argc, argv,
                         QGuiApplication::primaryScreen()->physicalDotsPerInch());

    if (!drawDemo.OpenDatabase()){
        return 2;
    }

    Arguments args = drawDemo.GetArguments();

    auto *pixmap=new QPixmap(static_cast<int>(args.width),
                               static_cast<int>(args.height));

    auto* painter=new QPainter(pixmap);

    osmscout::MapPainterQt        mapPainter(drawDemo.styleConfig);

    osmscout::TypeInfoRef         buildingType=drawDemo.database->GetTypeConfig()->GetTypeInfo("building");


    if (buildingType!=nullptr) {
        /*
    osmscout::FillStyleProcessorRef constructionProcessor=std::make_shared<ConstructionProcessor>(*db->GetTypeConfig());
    drawParameter.RegisterFillStyleProcessor(buildingType->GetIndex(),
                                             constructionProcessor);*/

        /*
    osmscout::FillStyleProcessorRef addressProcessor=std::make_shared<AddressProcessor>(*db->GetTypeConfig());
    drawParameter.RegisterFillStyleProcessor(buildingType->GetIndex(),
                                          addressProcessor);*/
    }

    drawDemo.LoadData();

    if (mapPainter.DrawMap(drawDemo.projection,
                           drawDemo.drawParameter,
                           drawDemo.data,
                           painter)) {
        if (!pixmap->save(QString::fromStdString(args.output),"PNG",-1)) {
            std::cerr << "Cannot write PNG" << std::endl;
        }
    }

    delete painter;
    delete pixmap;

    return 0;
}
