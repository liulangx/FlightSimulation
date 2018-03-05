#include <osgViewer/Viewer>
#include <osgGA/NodeTrackerManipulator>
#include <osg/ArgumentParser>

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
    viewer.setSceneData(root);

    viewer.setUpViewInWindow(0, 0, 600, 400);
    viewer.setCameraManipulator(nodeTracker);

    while(!viewer.done())
    {
        viewer.frame();
    }
    return 0;
}
