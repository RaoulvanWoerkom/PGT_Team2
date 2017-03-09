#include "TestApplication.h"
const float moveSpeed = 100;

TestApplication::TestApplication(void)
{
}

TestApplication::~TestApplication(void)
{
}

void TestApplication::createCamera()
{
	mCamera = mSceneMgr->createCamera("PlayerCam");

	mCamera->setPosition(Ogre::Vector3(0, 300, 500));
	mCamera->lookAt(Ogre::Vector3(0, 0, 0));
	mCamera->setNearClipDistance(5);

	mCameraMan = new OgreBites::SdkCameraMan(mCamera);
}

void TestApplication::createViewports()
{
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);

	vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));

	mCamera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) /
		Ogre::Real(vp->getActualHeight()));

}

void TestApplication::createScene()
{
	createLight();
	createPlane();
	createSphere();
}

void TestApplication::createPlane()
{
	// Create ground
	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);

	Ogre::MeshManager::getSingleton().createPlane(
		"ground",
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);

	Ogre::Entity* groundEntity = mSceneMgr->createEntity("ground");
	mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(groundEntity);

	groundEntity->setMaterialName("Examples/Rockwall");
	groundEntity->setCastShadows(false);
}


void TestApplication::createLight()
{
	mSceneMgr->setAmbientLight(Ogre::ColourValue::White);
	mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

	// Directional light
	Ogre::Light* directionalLight = mSceneMgr->createLight("DirectionalLight");
	directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);

	directionalLight->setDiffuseColour(Ogre::ColourValue(.3, .3, .3));
	directionalLight->setSpecularColour(Ogre::ColourValue(.3, .3, .3));

	directionalLight->setDirection(Ogre::Vector3(0, -1, 1));
}

void TestApplication::createSphere()
{
	Ogre::Entity *sphereEntity = mSceneMgr->createEntity("Sphere", "sphere.mesh");
	ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, 100, 0));
	ballNode->attachObject(sphereEntity);
}

bool iDown = false;
bool jDown = false;
bool kDown = false;
bool lDown = false;

bool TestApplication::keyPressed(const OIS::KeyEvent& ke)
{
	switch (ke.key)
	{
		case OIS::KC_I:
			iDown = true;
			break;
		case OIS::KC_J:
			jDown = true;
			break;
		case OIS::KC_K:
			kDown = true;
			break;
		case OIS::KC_L:
			lDown = true;
			break;
		default:
			break;
	}
	return BaseApplication::keyPressed(ke);
}

bool TestApplication::keyReleased(const OIS::KeyEvent& ke)
{
	switch (ke.key)
	{
	case OIS::KC_I:
		iDown = false;
		break;
	case OIS::KC_J:
		jDown = false;
		break;
	case OIS::KC_K:
		kDown = false;
		break;
	case OIS::KC_L:
		lDown = false;
		break;
	default:
		break;
	}

	return BaseApplication::keyReleased(ke);
}


bool TestApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	Ogre::Vector3 movePos = Ogre::Vector3(0, 0, 0);
	if (iDown)
	{
		movePos.z = -moveSpeed;
	}
	if (jDown)
	{
		movePos.x = -moveSpeed;
	}
	if (kDown)
	{
		movePos.z = moveSpeed;
	}
	if (lDown)
	{
		movePos.x = moveSpeed;
	}

	ballNode->translate(movePos * evt.timeSinceLastFrame, Ogre::Node::TS_LOCAL);
	// Ogre::Vector3 currPos = ballNode->getPosition();

	// Ogre::Vector3 newPos = currPos + movePos;
	// ballNode->setPosition(newPos);

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
		TestApplication app;

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