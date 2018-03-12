#include "CSceneManager.h"


CSceneManager::CSceneManager()
    : m_root(new osg::Group)
{
    initScene();
}

int CSceneManager::initScene()
{
    if(int err = readEarthDataBaseFrmFile())
    {
        return err;
    }
    m_root = new osg::Group();
    m_root -> addChild( m_earthNode.get() );

    m_plane = osgDB::readNodeFile("/home/liu/osg/data/cessna.osg.-90,0,0.rot");

    double h1 = calculateHeightForPoint(GeoPoint(m_geoSRS.get(), 95.8788086, 29.5076538, 0, ALTMODE_ABSOLUTE));
    GeoPoint location = GeoPoint(m_geoSRS.get(), 95.8788086, 29.5076538,h1 + double(5), ALTMODE_ABSOLUTE);
    m_plane2 = createPlane(m_plane, location, m_mapSRS.get(), 100000, 50);

    m_plane2->setName("plane2");
    m_root->addChild( m_plane2.get() );

    setNodeTracker(location);

    osg::ref_ptr<osg::Group> losGroup = new osg::Group();
    losGroup->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    losGroup->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::ALWAYS, 0, 1, false));
    m_root->addChild(losGroup);

    LinearLineOfSightNode* los = new LinearLineOfSightNode(
                m_mapNode.get(),
                GeoPoint(m_geoSRS.get(), 95.6788086, 29.3076538, 4258.00, ALTMODE_ABSOLUTE),
                GeoPoint(m_geoSRS.get(), 95.8788086, 29.5076538, 5620.11, ALTMODE_ABSOLUTE));
    losGroup->addChild(los);

    LinearLineOfSightEditor* p2peditor = new LinearLineOfSightEditor(los);
    m_root->addChild(p2peditor);

    m_earthManip = new EarthManipulator();
    osgEarth::Viewpoint vp;
    vp.name() = "Mt Ranier";
//    double equatorRadius = geoSRS->getEllipsoid()->getRadiusEquator();
    vp.focalPoint()->set(m_geoSRS.get(),95.8788086, 29.5076538, h1 + (double) 1010 , ALTMODE_ABSOLUTE);
    vp.pitch() = 0.0;
    vp.range() =1000;
    m_earthManip->setHomeViewpoint(vp);
    m_earthManip->setViewpoint(vp);


    return 0;
}

int CSceneManager::readEarthDataBaseFrmFile()
{
    CMapManager* mapManager = new CMapManager("/home/liu/Desktop/osgEarth/osgEarthData/bluemarble.earth");
    m_earthNode = mapManager->getEarthNode();
    m_mapNode = mapManager->getMapNode();
    if(!m_earthNode || !m_mapNode)
    {
       std::cout << "读取地球数据文件失败！";
       return READDATABASE_ERROR;
    }

    m_mapSRS = m_mapNode->getMapSRS();
    m_geoSRS = m_mapSRS->getGeographicSRS();

    return 0;
}

osg::Group *CSceneManager::getRootScene()
{
    return m_root.get();//m_root.get();
}

osgGA::NodeTrackerManipulator *CSceneManager::getNodeTrackerManipulator()
{
    return m_nodeTracker.get();//m_nodeTracker.get();
}

EarthManipulator *CSceneManager::getEarthManipulator()
{
    return m_earthManip.get();
}

int CSceneManager::addChild(osg::Node *_object)
{
    m_root->addChild(_object);
    return 0;
}

osg::Node *CSceneManager::createPlane(osg::Node *node, const GeoPoint &pos, const SpatialReference *mapSRS, double radius, double time)
{
    osg::ref_ptr<osg::MatrixTransform> positioner =  new osg::MatrixTransform;
    positioner->addChild(node);
    osg::ref_ptr<osg::AnimationPath> animationPath = createAnimationPath(pos, mapSRS, radius, time);
    positioner->setUpdateCallback(new osg::AnimationPathCallback(animationPath.get(), 0.0, 1.0));

    return positioner.release();
}

osg::AnimationPath *CSceneManager::createAnimationPath(const GeoPoint &pos, const SpatialReference *mapSRS, float radius, double looptime)
{
    //osg::ref_ptr<osg::AnimationPath> animationPath = new osg::AnimationPath;
    osg::AnimationPath* animationPath = new osg::AnimationPath;
    animationPath->setLoopMode(osg::AnimationPath::LOOP);

    int numSamples = 1000; //40
    double delta = osg::PI * 2.0 / (double) numSamples;
    GeoPoint mapPos = pos.transform(mapSRS);
    osg::Vec3d centerWorld;
    mapPos.toWorld(centerWorld);
    bool isProjeceted = mapSRS->isProjected();
    osg::Vec3d up = isProjeceted ? osg::Vec3d(0,0,1) : centerWorld;
    up.normalize();

    osg::Vec3d side = isProjeceted ? osg::Vec3d(1, 0, 0) : up ^ osg::Vec3d(0, 0, 1);
    double time = 0.0f;
    double time_delta = looptime/ ( double) numSamples;
    osg::Vec3d firstPosition;
    osg::Quat firstRotation;
    for(unsigned int i = 0; i< (unsigned int) numSamples; ++ i)
    {
        double angle = delta * (double) i;
        osg::Quat quat(angle, up);
        osg::Vec3d spoke = up * radius /numSamples * (i+1);//quat * (side * radius/**(i+1)*/);
        osg::Vec3d end = centerWorld + spoke;
        //mapPos.fromWorld(mapSRS, end);


        osg::Quat makeup;
        makeup.makeRotate(osg::Vec3d(0, 0, 1), up);
        //makeup.makeRotate(osg::Vec3d(0,0,1), osg::Vec3d(0,0,1));
        osg::Quat rot = makeup;
        animationPath -> insert(time, osg::AnimationPath::ControlPoint(end, rot));
        if(i == 0)
        {
            firstPosition = end;
            firstRotation = rot;
        }
        time += time_delta;
    }
    animationPath->insert(time, osg::AnimationPath::ControlPoint(firstPosition, firstRotation));
    return animationPath;
}

osg::Vec3d CSceneManager::calculateWorldPos(const SpatialReference *_mapSRS, GeoPoint _Pos)
{
    GeoPoint mapPos = _Pos.transform(_mapSRS);
    osg::Vec3d centerWorld;
    mapPos.toWorld(centerWorld);
    return centerWorld;
}

double CSceneManager::calculateHeightForPoint(GeoPoint _point)
{
    double query_resolution = 0.00000001;
    osgEarth::Map* m_map = m_mapNode->getMap();
    osgEarth::ElevationQuery query(m_map);
    double h = query.getElevation(_point, query_resolution);
    if(h > 0)
        return h;
    else
        return 0;
}

int CSceneManager::setNodeTracker(GeoPoint _location)
{
    osg::Vec3d centerWorld = calculateWorldPos(m_mapSRS, _location);
    //object
    osg::Vec3 ZeroPoint = osg::Vec3(centerWorld.x(), centerWorld.y(), centerWorld.z());
    //eye
    osg::Vec3 HomePoint = ZeroPoint + osg::Vec3(0, 0, 1000)/*osg::Vec3(0, 100, 0)*/;
    //direction
    osg::Vec3 Direction = osg::Vec3(0, -1, 0)/*.normalize()*/;

    m_nodeTracker = new osgGA::NodeTrackerManipulator;
    m_nodeTracker->setHomePosition( HomePoint, ZeroPoint/*osg::Vec3()*/, Direction/*osg::Z_AXIS*/ );
    m_nodeTracker->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
    m_nodeTracker->setRotationMode( osgGA::NodeTrackerManipulator::TRACKBALL );
    m_nodeTracker->setTrackNode( m_plane );

    return 0;

}
