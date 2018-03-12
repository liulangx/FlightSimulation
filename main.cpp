#include <osgViewer/Viewer>
#include <osgGA/NodeTrackerManipulator>
#include <osg/ArgumentParser>
#include <osgGA/EventHandler>

#include "CSceneManager.h"

using namespace osgEarth;
using namespace osgEarth::Util;

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer(arguments);
    CSceneManager sceneManager;
    osg::Group* root = sceneManager.getRootScene();
    osgGA::NodeTrackerManipulator* nodeTracker = sceneManager.getNodeTrackerManipulator();
    osgEarth::Util::EarthManipulator* earthManip = sceneManager.getEarthManipulator();

    viewer.setSceneData(root);


    viewer.setUpViewInWindow(0, 0, 600, 400);
    viewer.setCameraManipulator(nodeTracker/*earthManip*/);
    viewer.addEventHandler(new osgGA::EventHandler);

    while(!viewer.done())
    {
        viewer.frame();
    }
    return 0;
}
