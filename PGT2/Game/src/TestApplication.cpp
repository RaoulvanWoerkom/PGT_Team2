#include "TestApplication.h"
#include "../OgreText.h"
#include <sstream>
#include <iomanip>

const float moveSpeed = 100;
const int totalGameTime = 60;

#include "Ogre.h"
OgreText *timerText;

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

void TestApplication::init()
{
	remainingTime = totalGameTime;
	timer = new Ogre::Timer();
	timer->reset();
	timerText = new OgreText();
	timerText->setPos(0.4f, 0.1f);        // Text position, using relative co-ordinates
	timerText->setCol(1.0f, 1.0f, 1.0f, 0.8f);    // Text colour (Red, Green, Blue, Alpha)
	timerText->setText("Time: 60");
}

void TestApplication::createScene()
{
	init();
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

	size_t vertex_count, index_count;
	Ogre::Vector3* vertices;
	unsigned long* indices;

	GetMeshInformation(sphereEntity->getMesh(), vertex_count, vertices, index_count, indices, ballNode->getPosition(), ballNode->getOrientation(), ballNode->getScale());
	Ogre::Vector3 kaas = vertices[6];

	delete[] vertices;
	delete[] indices;
}

void TestApplication::GetMeshInformation(const Ogre::MeshPtr mesh,
	size_t &vertex_count,
	Ogre::Vector3* &vertices,
	size_t &index_count,
	unsigned long* &indices,
	const Ogre::Vector3 &position,
	const Ogre::Quaternion &orient,
	const Ogre::Vector3 &scale)
{
	bool added_shared = false;
	size_t current_offset = 0;
	size_t shared_offset = 0;
	size_t next_offset = 0;
	size_t index_offset = 0;

	vertex_count = index_count = 0;

	// Calculate how many vertices and indices we're going to need
	for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
	{
		Ogre::SubMesh* submesh = mesh->getSubMesh(i);

		// We only need to add the shared vertices once
		if (submesh->useSharedVertices)
		{
			if (!added_shared)
			{
				vertex_count += mesh->sharedVertexData->vertexCount;
				added_shared = true;
			}
		}
		else
		{
			vertex_count += submesh->vertexData->vertexCount;
		}

		// Add the indices
		index_count += submesh->indexData->indexCount;
	}


	// Allocate space for the vertices and indices
	vertices = new Ogre::Vector3[vertex_count];
	indices = new unsigned long[index_count];

	added_shared = false;

	// Run through the submeshes again, adding the data into the arrays
	for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
	{
		Ogre::SubMesh* submesh = mesh->getSubMesh(i);

		Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

		if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared))
		{
			if (submesh->useSharedVertices)
			{
				added_shared = true;
				shared_offset = current_offset;
			}

			const Ogre::VertexElement* posElem =
				vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

			Ogre::HardwareVertexBufferSharedPtr vbuf =
				vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

			unsigned char* vertex =
				static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

			// There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
			//  as second argument. So make it float, to avoid trouble when Ogre::Real will
			//  be comiled/typedefed as double:
			//      Ogre::Real* pReal;
			float* pReal;

			for (size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
			{
				posElem->baseVertexPointerToElement(vertex, &pReal);

				Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

				vertices[current_offset + j] = (orient * (pt * scale)) + position;
			}

			vbuf->unlock();
			next_offset += vertex_data->vertexCount;
		}


		Ogre::IndexData* index_data = submesh->indexData;
		size_t numTris = index_data->indexCount / 3;
		Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;
		if (ibuf.isNull()) continue; // need to check if index buffer is valid (which will be not if the mesh doesn't have triangles like a pointcloud)

		bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

		unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
		unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


		size_t offset = (submesh->useSharedVertices) ? shared_offset : current_offset;
		size_t index_start = index_data->indexStart;
		size_t last_index = numTris * 3 + index_start;

		if (use32bitindexes)
			for (size_t k = index_start; k < last_index; ++k)
			{
				indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
			}

		else
			for (size_t k = index_start; k < last_index; ++k)
			{
				indices[index_offset++] = static_cast<unsigned long>(pShort[k]) +
					static_cast<unsigned long>(offset);
			}

		ibuf->unlock();
		current_offset = next_offset;
	}
}

void TestApplication::showScore(double score)
{
	OgreText *textItem = new OgreText;
	textItem->setText("Score: " + static_cast<std::ostringstream*>(&(std::ostringstream() << score))->str());    // Text to be displayed
										  // Now it is possible to use the Ogre::String as parameter too
	textItem->setPos(0.1f, 0.1f);        // Text position, using relative co-ordinates
	textItem->setCol(1.0f, 1.0f, 1.0f, 0.5f);    // Text colour (Red, Green, Blue, Alpha)

}


void TestApplication::updateRemainingTime(double elapsedTime)
{
	remainingTime = totalGameTime - elapsedTime;
	stringstream stream;
	stream << std::fixed << std::setprecision(1) << remainingTime; 	// Set number of digits after the decimal point to 1, for the timer display.
	std::string timeRepresentation = stream.str();
	timerText->setText("Time: " + timeRepresentation);
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
	double elapsedTime =  (timer->getMilliseconds() / 1000.0);
	updateRemainingTime(elapsedTime);
	showScore(1);

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