#ifndef __Baller_h_
#define __Baller_h_

#include "BaseApplication.h"
#include "RigidBody.h"
#include "ForceGenerator.h"
#include "CustomCamera.h"
#include "Ball.h"


class Baller : public BaseApplication 
{
public:
	Baller();
	virtual ~Baller();
	Ball ballBody;
	
	Ogre::Timer* timer;
	double elapsedTime;
	double remainingTime;
	double totalGameTime;
	bool isGameOver;
	ForceRegistry registry;
	Gravity gravity;
	CustomCamera camera;

protected:
	virtual void createScene();
	virtual void init();
	virtual void initGameOver();
	virtual void createCamera();
	virtual void restartGame();
	virtual void createPlane();
	virtual void createLight();
	virtual void createViewports();


private:

	Ogre::SceneNode* groundNode;
	Ogre::Entity* groundEntity;
	virtual void createSphere();
	virtual void showScore(double score);
	virtual void updateRemainingTime();

	void Baller::CheckBallCollision();
	Ogre::Vector3 Baller::normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3);
	float Baller::clamp(float n, float lower, float upper);
	Ogre::Vector3 Baller::closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition);

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


#endif // #ifndef __Baller_h_

