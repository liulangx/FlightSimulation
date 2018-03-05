#include "CMapManager.h"

CMapManager::CMapManager(std::string _filePath)
    : m_filePath(_filePath)

{
    this->readMapNode();
}

int CMapManager::readMapNode()
{
    m_earthNode = osgDB::readNodeFile(m_filePath/*"/home/liu/Desktop/osgEarth/osgEarthData/bluemarble.earth"*/);
    if(!m_earthNode)
    {
        OE_NOTICE <<"Unable to load earth Model";
        return ErrorCodeMap::MAP_REFRMFILE;
    }

    m_mapNode = osgEarth::MapNode::findMapNode( m_earthNode.get() );


    return 0; //正确读入
}

MapNode *CMapManager::getMapNode() const
{
    return m_mapNode.get();
}

osg::Node *CMapManager::getEarthNode() const
{
    return m_earthNode.get();
}
