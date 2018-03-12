#ifndef CSCENEMANAGER_H
#define CSCENEMANAGER_H
#include "CMapManager.h"
#include <osg/Group>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/Depth>
#include <osgEarth/TerrainTileModel>
#include <osgEarth/TerrainTileNode>
#include <osgEarth/ElevationQuery>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/LinearLineOfSight>
#include <osgEarthUtil/RadialLineOfSight>

using namespace osgEarth;
using namespace osgEarth::Util;
class CSceneManager
{
public:
    enum ErrorCodeScene {
        READDATABASE_ERROR = 1 //读取地球数据失败
    };
    explicit CSceneManager();
    int initScene();
    //1, 从文件中读取地球数据
    int readEarthDataBaseFrmFile();
    osg::Group *getRootScene();
    osgGA::NodeTrackerManipulator *getNodeTrackerManipulator();
    osgEarth::Util::EarthManipulator *getEarthManipulator();
    int addChild(osg::Node* _object);

protected:
    osg::Node *createPlane(osg::Node* node, const GeoPoint& pos, const SpatialReference* mapSRS, double radius, double time);
    osg::AnimationPath *createAnimationPath(const GeoPoint& pos, const SpatialReference* mapSRS, float radius, double looptime);
    //通过给出mapSRS和在几何SRS上的值计算对象在世界坐标系的位置坐标
    osg::Vec3d calculateWorldPos(const SpatialReference* _mapSRS, GeoPoint _Pos);
    //计算某个点（经纬度点）的地面高度
    double calculateHeightForPoint(GeoPoint _point);
    int setNodeTracker(GeoPoint _location);
private:
    osg::ref_ptr<osg::Group> m_root;
    osg::ref_ptr<osg::Group> m_losGroup;
    osg::ref_ptr<osgGA::NodeTrackerManipulator> m_nodeTracker;
    osg::ref_ptr<osgEarth::Util::EarthManipulator> m_earthManip; //观察地球漫游器
    osg::observer_ptr<osg::Node> m_earthNode;
    osg::observer_ptr<osgEarth::MapNode> m_mapNode;
    osg::ref_ptr<osg::Node> m_plane2;
    osg::ref_ptr<osg::Node> m_plane;
    osg::ref_ptr<const SpatialReference> m_mapSRS;
    osg::ref_ptr<const SpatialReference> m_geoSRS;
};

#endif // CSCENEMANAGER_H
