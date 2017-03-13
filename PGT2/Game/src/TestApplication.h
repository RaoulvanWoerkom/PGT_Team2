#ifndef __TestApplication_h_
#define __TestApplication_h_

#include "BaseApplication.h"

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

class TestApplication : public BaseApplication
{
public:
	TestApplication();
	virtual ~TestApplication();

protected:
	virtual void createScene();
	virtual void createCamera();
	virtual void createViewports();

	virtual void createFrameListener();
	virtual void destroyScene();
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& fe);

private:
	void defineTerrain(long x, long y);
	void initBlendMaps(Ogre::Terrain* terrain);
	void configureTerrainDefaults(Ogre::Light* light);

	bool mTerrainsImported;
	Ogre::TerrainGroup* mTerrainGroup;
	Ogre::TerrainGlobalOptions* mTerrainGlobals;
	OgreBites::Label* mInfoLabel;

};

#endif // #ifndef __TestApplication_h_