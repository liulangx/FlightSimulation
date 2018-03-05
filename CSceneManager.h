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

    };
    explicit CSceneManager();
    int initScene();
    osg::Group *getRootScene();
    osgGA::NodeTrackerManipulator *getNodeTrackerManipulator();
    int addChild(osg::Node* _object);

protected:
    osg::Node *createPlane(osg::Node* node, const GeoPoint& pos, const SpatialReference* mapSRS, double radius, double time);
    osg::AnimationPath *createAnimationPath(const GeoPoint& pos, const SpatialReference* mapSRS, float radius, double looptime);
private:
    osg::ref_ptr<osg::Group> m_root;
    osg::ref_ptr<osg::Group> m_losGroup;
    osg::ref_ptr<osgGA::NodeTrackerManipulator> m_nodeTracker;

};

#endif // CSCENEMANAGER_H
