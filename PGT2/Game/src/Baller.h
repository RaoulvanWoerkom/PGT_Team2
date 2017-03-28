#ifndef __Baller_h_
#define __Baller_h_

#include "BaseApplication.h"
#include "RigidBody.h"
#include "ForceGenerator.h"
#include "CustomCamera.h"
#include "World.h"
#include "InputManager.h"


class Baller : public BaseApplication 
{
public:
	Baller();
	virtual ~Baller();
	
	InputManager inputManager;
	World world;
	Ogre::Timer* timer;
	double elapsedTime;
	double remainingTime;
	double totalGameTime;
	bool isGameOver;
	
	
	

protected:
	virtual void createScene();
	virtual void init();
	virtual void initGameOver();
	virtual void createCamera();
	virtual void restartGame();
	
	virtual void createViewports();


private:

	
	virtual void showScore(double score);
	virtual void updateRemainingTime();

	void checkBallCollision(Ogre::Entity* otherEntity, Ogre::SceneNode* otherSceneNode);
	Ogre::Vector3 Baller::normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3);
	float Baller::clamp(float n, float lower, float upper);
	Ogre::Vector3 Baller::closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition);
	
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);
    bool mouseMoved(const OIS::MouseEvent &arg);
};

struct VerticeSection 
{
	size_t vertex_count;
	Ogre::Vector3* vertices;
	size_t index_count;
	unsigned long* indices;
};


#endif // #ifndef __Baller_h_

