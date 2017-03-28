#include "Baller.h"
#include "OgreText.h"
#include <sstream>
#include <iomanip>
#include "BaseApplication.h"
#include <cmath>
#include "Ogre.h"
#include "Helper.h"
#include "Highscore.h"




OgreText *scoreText;
OgreText *timerText;
OgreText *loseText;
const int START_GAME_TIME = 60;


Baller::Baller(void)
{
	
}

Baller::~Baller(void)
{
}


bool Baller::mouseMoved(const OIS::MouseEvent &arg)
{
    if (mTrayMgr->injectMouseMove(arg)) return true;
	world.mouseMoved(arg);
	return true;
}


void Baller::createCamera()
{
	world = World();
	mCamera = mSceneMgr->createCamera("PlayerCam");
	world.createCamera(mCamera, mSceneMgr, mWindow);
}

void Baller::createViewports()
{
	world.createViewports();
}

void Baller::init()
{
	inputManager = InputManager();
	isGameOver = false;
	totalGameTime = START_GAME_TIME;
	elapsedTime = 0.0;
	timer = new Ogre::Timer();
	timer->reset();
	scoreText = new OgreText();
	scoreText->setPos(0.1f, 0.1f);        // Text position, using relative co-ordinates
	scoreText->setCol(1.0f, 1.0f, 1.0f, 0.8f);    // Text colour (Red, Green, Blue, Alpha)
	timerText = new OgreText();
	timerText->setPos(0.4f, 0.1f);        // Text position, using relative co-ordinates
	timerText->setCol(1.0f, 1.0f, 1.0f, 0.8f);    // Text colour (Red, Green, Blue, Alpha)
	
	


	
}

void Baller::initGameOver()
{
	remainingTime = 0;
	isGameOver = true;

	loseText = new OgreText();
	loseText->setText("Your wrecking time is up! \n press 'Y' to restart.");
	loseText->setPos(0.4f, 0.2f);
	loseText->setCol(1.0f, 1.0f, 1.0f, 1.0f);

	Highscore * highscore = new Highscore();
	highscore->addToScoreboard("WreckingBall", 100);	
	
}


void Baller::restartGame()
{
	isGameOver = false;
	totalGameTime = START_GAME_TIME;
	loseText->setText("");
	elapsedTime = 0.0;
	timer->reset();
	world.restartWorld();
	
}

void Baller::createScene()
{
	init();
	world.createLight(mSceneMgr);
	world.createPlane(mSceneMgr);
	world.createSphere(mSceneMgr);
	world.setCameraFollow();

	world.splitVertices();
}





void Baller::showScore(double score)
{
	scoreText->setText("Score: " + static_cast<std::ostringstream*>(&(std::ostringstream() << score))->str());    // Text to be displayed
										  // Now it is possible to use the Ogre::String as parameter too
}

void Baller::updateRemainingTime()
{
	elapsedTime = timer->getMilliseconds() / 1000.0;
	remainingTime = totalGameTime - elapsedTime;

	if (remainingTime <= 0)
	{
		initGameOver();
	}

	stringstream stream;
	stream << std::fixed << std::setprecision(1) << remainingTime; 	// Set number of digits after the decimal point to 1, for the timer display.
	std::string timeRepresentation = stream.str();
	timerText->setText("Time: " + timeRepresentation);
}



bool Baller::keyPressed(const OIS::KeyEvent& ke)
{
	inputManager.keyPressed(ke);
	return BaseApplication::keyPressed(ke);
}

bool Baller::keyReleased(const OIS::KeyEvent& ke)
{
	inputManager.keyReleased(ke);
	return BaseApplication::keyReleased(ke);
}

bool Baller::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	if (!isGameOver)
	{
		updateRemainingTime();
		showScore(1);

		world.update(evt);
		if (InputManager::yDown)
		{
			restartGame();//temporary for testing purposes
		}
	}
	else
	{
		if (InputManager::yDown)
		{
			restartGame();//temporary for testing purposes
		}
	}
	return BaseApplication::frameRenderingQueued(evt);
}




#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
	INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
#else
	int main(int argc, char *argv[])
#endif
	{
		// Create application object
		Baller app;

		try {
			app.go();
		}
		catch (Ogre::Exception& e) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
			MessageBox(NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
			std::cerr << "An exception has occured: " <<
				e.getFullDescription().c_str() << std::endl;
#endif
		}

		return 0;
	}

#ifdef __cplusplus
}
#endif