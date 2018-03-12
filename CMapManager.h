#ifndef CMAPMANAGER_H
#define CMAPMANAGER_H
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
//#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarth/TerrainTileModel>
#include <osgEarth/TerrainTileNode>
#include <osgEarth/ElevationQuery>
#include <osgGA/NodeTrackerManipulator>
#include <osg/observer_ptr>
#include <string>

using namespace osgEarth;
using namespace osgEarth::Util;
using std::string;
class CMapManager
{
public:
    enum ErrorCodeMap {
        MAP_REFRMFILE = 1  //读取map文件失败

    };
    explicit CMapManager(string _filePath);
    int readMapNode();
    osgEarth::MapNode *getMapNode() const;
    osg::Node *getEarthNode() const;
protected:
    string m_filePath;
    osg::ref_ptr<osgEarth::MapNode> m_mapNode;
    osg::ref_ptr<osg::Node> m_earthNode;

};

#endif // CMAPMANAGER_H
