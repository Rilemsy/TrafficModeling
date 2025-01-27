#ifndef DRAWMAP_H
#define DRAWMAP_H

#include <osmscout/db/Database.h>
#include <osmscout/db/BasemapDatabase.h>

#include <osmscout/projection/MercatorProjection.h>

#include <osmscout/cli/CmdLineParsing.h>

#include <osmscoutmap/MapService.h>

#include <iostream>

struct Arguments {
    bool       debug=false;
    double     dpi=96.0;
    osmscout::Bearing angle;
    bool renderContourLines=true;
    bool renderHillShading=false;
    std::string map;
    std::string style;

    std::string output;

    size_t      width=1920;
    size_t      height=1080;

    std::string basemap;
    std::string srtmDirectory;

    osmscout::GeoCoord       center;
    osmscout::Magnification  zoom{osmscout::Magnification::magClose};
    osmscout::MapParameter::IconMode iconMode{osmscout::MapParameter::IconMode::FixedSizePixmap};
    std::list<std::string> iconPaths;

    double fontSize{3.0};
    std::string fontName = "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf";
};

enum DrawMapArgParserWindowStyle
{
    ARG_WS_CONSOLE,
    ARG_WS_WINDOW
};

class DrawMapArgParser: public osmscout::CmdLineParser
{
private:
    Arguments args;

public:
    DrawMapArgParser(const std::string& appName,
                     int argc, char* argv[],
                     double dpi,
                     DrawMapArgParserWindowStyle windowStyle=ARG_WS_CONSOLE)
        : osmscout::CmdLineParser(appName, argc, argv)
    {
        args.dpi = dpi;
        args.debug = false;
        args.fontSize = 3.0;
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
                          { args.map = value; }),
                      "databaseDir", "Database directory");
        AddPositional(osmscout::CmdLineStringOption(
                          [this](const std::string &value)
                          { args.style = value; }),
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
        AddPositional(
            osmscout::CmdLineGeoCoordOption(
                [this](const osmscout::GeoCoord &coord)
                { args.center = coord; }),
            "lat lon", "Rendering center");
        AddPositional(osmscout::CmdLineDoubleOption([this](const double &value)
                                                    { args.zoom.SetMagnification(value); }),
                      "zoom", "Rendering zoom");
    }
    Arguments GetArguments() const { return args; }
};

class DrawMapDemo
{
public:
    DrawMapArgParser argParser;

    osmscout::DatabaseParameter databaseParameter;
    osmscout::DatabaseRef       database;
    osmscout::MapServiceRef     mapService;
    osmscout::StyleConfigRef    styleConfig;

    osmscout::BasemapDatabaseRef basemapDatabase;

    osmscout::MercatorProjection  projection;
    osmscout::MapParameter        drawParameter;
    osmscout::AreaSearchParameter searchParameter;
    osmscout::MapData             data;

public:
    DrawMapDemo(const std::string& appName,
                int argc, char* argv[],
                double dpi=96.0,
                DrawMapArgParserWindowStyle windowStyle=ARG_WS_CONSOLE)
        : argParser(appName, argc, argv, dpi, windowStyle)
    {

    }

    bool OpenDatabase()
    {
        osmscout::CmdLineParseResult argResult=argParser.Parse();
        if (argResult.HasError()) {
            std::cerr << "ERROR: " << argResult.GetErrorDescription() << std::endl;
            std::cout << argParser.GetHelp() << std::endl;
            return false;
        }

        Arguments args=argParser.GetArguments();

        osmscout::log.Debug(args.debug);

        // if (!args.srtmDirectory.empty()) {
        //     databaseParameter.SetSRTMDirectory(args.srtmDirectory);
        // }

        database=std::make_shared<osmscout::Database>(databaseParameter);

        if (!database->Open(args.map)) {
            std::cerr << "Cannot open db" << std::endl;
            return false;
        }

        mapService=std::make_shared<osmscout::MapService>(database);

        styleConfig = std::make_shared<osmscout::StyleConfig>(database->GetTypeConfig());
        if (!styleConfig->Load(args.style)) {
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

        // TODO: arguments
        drawParameter.SetLabelLineMinCharCount(15);
        drawParameter.SetLabelLineMaxCharCount(30);
        drawParameter.SetLabelLineFitToArea(true);

        projection.Set(args.center,
                       args.angle.AsRadians(),
                       args.zoom,
                       args.dpi,
                       args.width,
                       args.height);

        // if (!args.basemap.empty()) {
        //     basemapDatabase=std::make_shared<osmscout::BasemapDatabase>(osmscout::BasemapDatabaseParameter{});
        //     if (!basemapDatabase->Open(args.basemap)){
        //         std::cerr << "Cannot open base map" << std::endl;
        //         return false;
        //     }
        // }

        return true;
    }

    void LoadData()
    {
        std::list<osmscout::TileRef> tiles;

        assert(database);
        assert(database->IsOpen());
        assert(mapService);
        assert(styleConfig);

        data.ClearDBData();

        mapService->LookupTiles(projection,tiles);
        mapService->LoadMissingTileData(searchParameter,*styleConfig,tiles);
        mapService->AddTileDataToMapData(tiles,data);
        mapService->GetGroundTiles(projection,data.groundTiles);

        if (GetArguments().renderHillShading) {
            data.srtmTile=mapService->GetSRTMData(projection);
        }

        LoadBaseMapTiles(data.baseMapTiles);
    }

    bool LoadBaseMapTiles(std::list<osmscout::GroundTile> &tiles)
    {
        if (!basemapDatabase) {
            return true;
        }

        osmscout::WaterIndexRef waterIndex = basemapDatabase->GetWaterIndex();
        if (!waterIndex) {
            return true;
        }

        osmscout::GeoBox boundingBox(projection.GetDimensions());
        if (!waterIndex->GetRegions(boundingBox,
                                    projection.GetMagnification(),
                                    tiles)) {
            std::cerr << "Failed to read base map tiles" << std::endl;
            return false;
        }

        return true;
    }

    Arguments GetArguments() const
    {
        return argParser.GetArguments();
    }
};

#endif // DRAWMAP_H
