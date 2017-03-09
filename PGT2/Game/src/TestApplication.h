#ifndef __TestApplication_h_
#define __TestApplication_h_

#include "BaseApplication.h"

class TestApplication : public BaseApplication
{
public:
	TestApplication();
	virtual ~TestApplication();
	Ogre::SceneNode* ballNode;
	Ogre::Timer* timer;
	int previousTime;

protected:
	virtual void createScene();
	virtual void init();
	virtual void createCamera();
	virtual void createPlane();
	virtual void createLight();
	virtual void createViewports();
	virtual void createSphere();
	virtual void showScore(double score);
	virtual void showTime(double time);

	void GetMeshInformation(const Ogre::MeshPtr mesh,
		size_t &vertex_count,
		Ogre::Vector3* &vertices,
		size_t &index_count,
		unsigned long* &indices,
		const Ogre::Vector3 &position,
		const Ogre::Quaternion &orient,
		const Ogre::Vector3 &scale);

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);
};


#endif // #ifndef __TestApplication_h_

