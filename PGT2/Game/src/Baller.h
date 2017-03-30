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
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);
    bool mouseMoved(const OIS::MouseEvent &arg);
};




#endif // #ifndef __Baller_h_

