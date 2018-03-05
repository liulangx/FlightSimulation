#include "CSceneManager.h"


CSceneManager::CSceneManager()
    : m_root(new osg::Group)
{
    initScene();
}

int CSceneManager::initScene()
{
    CMapManager* mapManager = new CMapManager("/home/liu/Desktop/osgEarth/osgEarthData/bluemarble.earth");
    osg::Node* earthNode = mapManager->getEarthNode();
    osgEarth::MapNode* mapNode = mapManager->getMapNode();

    m_root = new osg::Group();
    m_root -> addChild( earthNode );
    osg::Group* losGroup = new osg::Group();
    losGroup->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    losGroup->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::ALWAYS, 0, 1, false));
    m_root->addChild(losGroup);

    const SpatialReference* mapSRS = mapNode->getMapSRS();
    const SpatialReference* geoSRS = mapSRS->getGeographicSRS();

    LinearLineOfSightNode* los = new LinearLineOfSightNode(
                mapNode,
                GeoPoint(geoSRS, 95.6788086, 29.3076538, 4258.00, ALTMODE_ABSOLUTE),
                GeoPoint(geoSRS, 95.8788086, 29.5076538, 5620.11, ALTMODE_ABSOLUTE));
    losGroup->addChild(los);

    LinearLineOfSightEditor* p2peditor = new LinearLineOfSightEditor(los);
    m_root->addChild(p2peditor);

    LinearLineOfSightNode* relativeLos = new LinearLineOfSightNode(
                mapNode,
                GeoPoint(geoSRS, 95.2788086, 29.3200000, 10, ALTMODE_RELATIVE),
                GeoPoint(geoSRS, 95.8788086, 29.5076538, 10, ALTMODE_RELATIVE));
    losGroup->addChild(relativeLos);

    LinearLineOfSightEditor* relEditor = new LinearLineOfSightEditor(relativeLos);
    m_root->addChild(relEditor);

    double query_resolution = 0.00000001;
    osgEarth::Map* m_map = mapNode->getMap();
    osgEarth::ElevationQuery query(m_map);
    double h1 = query.getElevation(GeoPoint(mapNode->getMapSRS(), 95.8788086, 29.5076538, 0.0, osgEarth::AltitudeMode::ALTMODE_ABSOLUTE), query_resolution);

    osg::ref_ptr<osg::Node> plane = osgDB::readNodeFile("/home/liu/Desktop/osgEarth/osgEarthData/dumptruck.osgt.5,5,5.scale");
    osg::ref_ptr<osg::Node> plane_d = osgDB::readNodeFile("/home/liu/osg/data/cessna.osg");

    osg::Node* plane1 = createPlane(plane, GeoPoint(geoSRS, 95.8788086, 29.5076538,h1+(double)2000, ALTMODE_ABSOLUTE), mapSRS, 10, 5);
    osg::Node* plane2 = createPlane(plane_d, GeoPoint(geoSRS, 95.8788086, 29.5076538,h1 + double(1000), ALTMODE_ABSOLUTE), mapSRS, 1000, 100);

    m_root->addChild( plane1 );
    plane1->setName("plane1");
    m_root->addChild( plane2 );

    osgEarth::Viewpoint vp;
    vp.name() = "Mt Ranier";
    double equatorRadius = geoSRS->getEllipsoid()->getRadiusEquator();
    vp.focalPoint()->set(geoSRS,95.8788086, 29.5076538, h1 + (double) 1010 , ALTMODE_ABSOLUTE);
    vp.pitch() = 0.0;
    vp.range() =1000;

    GeoPoint pos = GeoPoint(geoSRS, 95.8788086, 29.5076538,h1+(double)2000, ALTMODE_ABSOLUTE);
    GeoPoint mapPos = pos.transform(mapSRS);
    osg::Vec3d centerWorld;
    mapPos.toWorld(centerWorld);
    osg::Vec3 ZeroPoint = osg::Vec3(centerWorld.x(), centerWorld.y(), centerWorld.z());
    osg::Vec3 HomePoint = ZeroPoint - osg::Vec3(0, 100, 0);
    osg::Vec3 Direction = ZeroPoint/*.normalize()*/;

    m_nodeTracker = new osgGA::NodeTrackerManipulator;
    m_nodeTracker->setHomePosition( HomePoint, ZeroPoint/*osg::Vec3()*/, Direction/*osg::Z_AXIS*/ );
    m_nodeTracker->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
    m_nodeTracker->setRotationMode( osgGA::NodeTrackerManipulator::TRACKBALL );
    m_nodeTracker->setTrackNode( plane_d );
}

osg::Group *CSceneManager::getRootScene()
{
    return m_root.get();//m_root.get();
}

osgGA::NodeTrackerManipulator *CSceneManager::getNodeTrackerManipulator()
{
    return m_nodeTracker.get();//m_nodeTracker.get();
}

int CSceneManager::addChild(osg::Node *_object)
{
    m_root->addChild(_object);
    return 0;
}

osg::Node *CSceneManager::createPlane(osg::Node *node, const GeoPoint &pos, const SpatialReference *mapSRS, double radius, double time)
{
    osg::MatrixTransform* positioner =  new osg::MatrixTransform;
    positioner->addChild(node);
    osg::AnimationPath* animationPath = createAnimationPath(pos, mapSRS, radius, time);
    positioner->setUpdateCallback(new osg::AnimationPathCallback(animationPath, 0.0, 1.0));

    return positioner;
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
        osg::Vec3d spoke = quat * (side * radius*(i+1));
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
