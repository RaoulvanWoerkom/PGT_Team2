#ifndef __TestApplication_h_
#define __TestApplication_h_

#include "BaseApplication.h"

class TestApplication : public BaseApplication
{
public:
	TestApplication();
	virtual ~TestApplication();
	Ogre::SceneNode* ballNode;

protected:
	virtual void createScene();
	virtual void createCamera();
	virtual void createPlane();
	virtual void createLight();
	virtual void createViewports();
	virtual void createSphere();
	virtual void showScore(double score);

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);

};


#endif // #ifndef __TestApplication_h_