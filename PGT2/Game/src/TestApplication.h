#ifndef __TestApplication_h_
#define __TestApplication_h_

#include "BaseApplication.h"
#include "RigidBody.h"
#include "ForceGenerator.h"
#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>


class TestApplication : public BaseApplication
{
public:
	TestApplication();
	virtual ~TestApplication();
	RigidBody ballBody;
	Ogre::SceneNode* camNode;
	Ogre::SceneNode* camPitchNode;
	Ogre::Timer* timer;
	double elapsedTime;
	double remainingTime;
	double totalGameTime;
	bool isGameOver;
	ForceRegistry registry;
	Gravity gravity;


protected:
	virtual void createScene();
	virtual void init();
	virtual void initGameOver();
	virtual void restartGame();
	virtual void createCamera();
	virtual void createPlane();
	virtual void createLight();
	virtual void createViewports();


private:

	Ogre::SceneNode* groundNode;
	Ogre::Entity* groundEntity;
	void setCameraTarget(Ogre::SceneNode* node);
	virtual void createSphere();
	virtual void showScore(double score);
	virtual void updateRemainingTime();

	void CheckBallCollision();
	Ogre::Vector3 normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3);
	float clamp(float n, float lower, float upper);
	Ogre::Vector3 closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition);

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
    virtual bool mouseMoved(const OIS::MouseEvent &arg);
};


#endif // #ifndef __TestApplication_h_

