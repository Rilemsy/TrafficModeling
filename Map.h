#ifndef MAP_H
#define MAP_H

#include <osmscout/db/Database.h>
#include <osmscout/db/BasemapDatabase.h>
#include <osmscout/projection/MercatorProjection.h>
#include <osmscout/cli/CmdLineParsing.h>
#include <osmscoutmap/MapService.h>

#include <iostream>

struct Arguments {
    bool                debug=false;
    double              dpi=96.0;
    osmscout::Bearing   angle;
    bool                renderContourLines=false;
    bool                renderHillShading=false;
    std::string         dbPath;
    std::string         stylePath;

    size_t              width=1920;
    size_t              height=1080;

    osmscout::GeoCoord       center;
    osmscout::Magnification  zoom   {osmscout::Magnification::magClose};
    osmscout::MapParameter::IconMode iconMode{osmscout::MapParameter::IconMode::FixedSizePixmap};
    std::list<std::string>          iconPaths;
    double              radius=1000.0;

    double              fontSize{3.0};
    std::string fontName;
};

enum MapArgParserWindowStyle
{
    ARG_WS_CONSOLE,
    ARG_WS_WINDOW
};

class MapArgumentParser: public osmscout::CmdLineParser
{
private:
    Arguments args;

public:
    MapArgumentParser(const std::string& appName,
                     int argc, char* argv[],
                     double dpi,
                     MapArgParserWindowStyle windowStyle=ARG_WS_CONSOLE)
        : osmscout::CmdLineParser(appName, argc, argv)
    {
        args.dpi = dpi;
        args.debug = false;
        args.fontSize = 3.0;
        args.fontName = "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf";
        AddOption(osmscout::CmdLineDoubleOption([this](const double &value)
                            { args.angle = osmscout::Bearing::Degrees(value); }),
                            "angle", "Rendering angle (in degrees)", false);
        AddOption(osmscout::CmdLineStringOption(
                            [this](const std::string &value)
                            { args.fontName = value; }),
                            "fontName", "Rendering font (" + args.fontName + ")", false);
        AddOption(osmscout::CmdLineStringOption([this](const std::string &value)
                                                { args.iconPaths.push_back(value); }),
                                                "iconPath", "Icon lookup directory", false);
        args.renderContourLines = false;
        args.renderHillShading = false;
        AddPositional(osmscout::CmdLineStringOption(
                            [this](const std::string &value)
                            { args.dbPath = value; }),
                            "databaseDir", "Database directory");
        AddPositional(osmscout::CmdLineStringOption(
                            [this](const std::string &value)
                            { args.stylePath = value; }),
                            "stylesheet", "Map stylesheet");
        if (windowStyle == ARG_WS_CONSOLE)
        {
            AddPositional(osmscout::CmdLineSizeTOption(
                            [this](const size_t &value)
                            { args.width = value; }),
                            "width", "Image width");
            AddPositional(osmscout::CmdLineSizeTOption(
                            [this](const size_t &value)
                            { args.height = value; }),
                            "height", "Image height");
        }
        AddPositional(osmscout::CmdLineGeoCoordOption(
                [this](const osmscout::GeoCoord &coord)
                { args.center = coord; }),
                "lat lon", "Rendering center");
        AddPositional(osmscout::CmdLineDoubleOption([this](const double &value)
                                                    { args.zoom.SetMagnification(value); }),
                      "zoom", "Rendering zoom");
        AddPositional(osmscout::CmdLineDoubleOption([this](const double &value)
                                                    { args.radius = value; }),
                      "radious", "Network radious");
    }
    Arguments GetArguments() const { return args; }
};

class Map
{
public:

    osmscout::DatabaseRef           database;
    osmscout::StyleConfigRef        styleConfig;
    osmscout::MercatorProjection    projection;
    osmscout::MapParameter          drawParameter;
    osmscout::MapData               data;
    osmscout::MapServiceRef         mapService;

    MapArgumentParser argParser;

public:
    Map(const std::string& appName,
                int argc, char* argv[],
                double dpi=96.0,
                MapArgParserWindowStyle windowStyle=ARG_WS_CONSOLE)
        : argParser(appName, argc, argv, dpi, windowStyle)
    {
    }

    bool openDatabase()
    {
        osmscout::CmdLineParseResult argResult=argParser.Parse();
        if (argResult.HasError()) {
            std::cerr << "ERROR: " << argResult.GetErrorDescription() << std::endl;
            std::cout << argParser.GetHelp() << std::endl;
            return false;
        }

        Arguments args=argParser.GetArguments();

        osmscout::DatabaseParameter databaseParameter;
        database=std::make_shared<osmscout::Database>(databaseParameter);

        if (!database->Open(args.dbPath)) {
            std::cerr << "Cannot open db" << std::endl;
            return false;
        }

        mapService=std::make_shared<osmscout::MapService>(database);

        styleConfig = std::make_shared<osmscout::StyleConfig>(database->GetTypeConfig());
        if (!styleConfig->Load(args.stylePath)) {
            std::cerr << "Cannot open style" << std::endl;
            return false;
        }

        if (!args.fontName.empty()) {
            drawParameter.SetFontName(args.fontName);
        }

        drawParameter.SetFontSize(args.fontSize);
        drawParameter.SetRenderSeaLand(true);
        drawParameter.SetRenderUnknowns(false);
        drawParameter.SetRenderBackground(false);
        drawParameter.SetRenderContourLines(args.renderContourLines);
        drawParameter.SetRenderHillShading(args.renderHillShading);
        drawParameter.SetIconMode(args.iconMode);
        drawParameter.SetIconPaths(args.iconPaths);
        drawParameter.SetDebugData(args.debug);
        drawParameter.SetDebugPerformance(args.debug);
        drawParameter.SetLabelLineMinCharCount(15);
        drawParameter.SetLabelLineMaxCharCount(30);
        drawParameter.SetLabelLineFitToArea(true);

        projection.Set(args.center, args.angle.AsRadians(), args.zoom, args.dpi, args.width, args.height);

        return true;
    }

    void loadData()
    {
        std::list<osmscout::TileRef> tiles;
        data.ClearDBData();
        osmscout::AreaSearchParameter searchParameter;

        mapService->LookupTiles(projection,tiles);
        mapService->LoadMissingTileData(searchParameter,*styleConfig,tiles);
        mapService->AddTileDataToMapData(tiles,data);
        mapService->GetGroundTiles(projection,data.groundTiles);

        if (GetArguments().renderHillShading) {
            data.srtmTile=mapService->GetSRTMData(projection);
        }
    }

    Arguments GetArguments() const
    {
        return argParser.GetArguments();
    }
};

#endif // MAP_H
