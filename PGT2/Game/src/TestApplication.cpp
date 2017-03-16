#include "TestApplication.h"
#include "../OgreText.h"
#include <sstream>
#include <iomanip>

const float moveSpeed = 100;
const int BALL_SIZE = 100;
const int START_GAME_TIME = 10;

#include "Ogre.h"
OgreText *scoreText;
OgreText *timerText;
Ogre::Light* directionalLight;

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

#include "BaseApplication.h"

TestApplication::TestApplication(void)
{
}

TestApplication::~TestApplication(void)
{
}

void TestApplication::createCamera()
{
	ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, 100, 0));
	// Create the Camera with name PlayerCam
	mCamera = mSceneMgr->createCamera("PlayerCam");

	mCamera->setPosition(Ogre::Vector3(0, 500, 500));
	mCamera->setNearClipDistance(500);
	camNode = ballNode->createChildSceneNode();
	camNode->setPosition(0, 0, 0);
	camPitchNode = camNode->createChildSceneNode();
	camPitchNode->setPosition(0, 50, 500);
	/*mCameraMan = new OgreBites::SdkCameraMan(mCamera);*/
	camPitchNode->attachObject(mCamera);
	mCamera->setAutoTracking(true, ballNode);
	/*mCameraMan->setTarget(ballNode);*/
}
bool TestApplication::mouseMoved(const OIS::MouseEvent &arg)
{
    if (mTrayMgr->injectMouseMove(arg)) return true;
    camNode->yaw(Ogre::Degree(-arg.state.X.rel * 0.25f));
    return true;

	bool infiniteClip =
		mRoot->getRenderSystem()->getCapabilities()->hasCapability(
		Ogre::RSC_INFINITE_FAR_PLANE);

	if (infiniteClip)
		mCamera->setFarClipDistance(0);
	else
		//mCamera->setFarClipDistance(932500000);
		mCamera->setFarClipDistance(1390000);

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

void getTerrainImage(bool flipX, bool flipY, Ogre::Image& img)
{
	img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

	if (flipX)
		img.flipAroundY();
	if (flipY)
		img.flipAroundX();
}

void TestApplication::initGameOver()
{
	remainingTime = 0;
	isGameOver = true;

	OgreText * loseText = new OgreText();
	loseText->setText("PLAY TIME'S OVER B] \n git gud next time");
	loseText->setPos(0.4f, 0.2f);
	loseText->setCol(1.0f, 1.0f, 1.0f, 1.0f);
}

void TestApplication::createScene()
{
	init();
	createLight();
	createPlane();
	createSphere();
}

void TestApplication::createLight()
{
	mSceneMgr->setAmbientLight(Ogre::ColourValue::White);
	mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

	// Directional light
	directionalLight = mSceneMgr->createLight("DirectionalLight");
	directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);

	directionalLight->setDiffuseColour(Ogre::ColourValue(.3f, .3f, .3f));
	directionalLight->setSpecularColour(Ogre::ColourValue(.3f, .3f, .3f));

	directionalLight->setDirection(Ogre::Vector3(0, -1, 1));
}

void TestApplication::createPlane()
{
	// Create ground
	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
	Ogre::MeshManager::getSingleton().createPlane(
		"ground",
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);

	groundEntity = mSceneMgr->createEntity("ground");
	//mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(groundEntity);
	groundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	groundNode->attachObject(groundEntity);

	groundEntity->setMaterialName("Examples/Rockwall");
	groundEntity->setCastShadows(false);
}

void TestApplication::createSphere()
{
	ballNode->setPosition(0, 200, 0);
	Ogre::Entity *sphereEntity = mSceneMgr->createEntity("Sphere", "sphere.mesh");
	ballNode->attachObject(sphereEntity);
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
	scoreText->setText("Score: " + static_cast<std::ostringstream*>(&(std::ostringstream() << score))->str());    // Text to be displayed
										  // Now it is possible to use the Ogre::String as parameter too
}

void TestApplication::updateRemainingTime()
{
	if (isGameOver == false)
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
	updateRemainingTime();
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

void TestApplication::CheckBallCollision(Ogre::SceneNode* node1, Ogre::Entity* entity1)
{

	size_t vertex_count1, index_count1;
	Ogre::Vector3* vertices1;
	unsigned long* indices1;

	GetMeshInformation(entity1->getMesh(), vertex_count1, vertices1, index_count1, indices1, node1->getPosition(), node1->getOrientation(), node1->getScale());



	double shortestLength = 100000000000;
	int chosenIndex = -1;
	Ogre::Vector3 closestHitCoordinates;
	Ogre::Vector3 normalVec = Ogre::Vector3(-1, -1, -1);
	Ogre::Vector3 ballPos = ballNode->getPosition();

	int max = index_count1 - 3;
	for (size_t i = 0; i < max; i += 3)
	{
		long index1 = indices1[i];
		long index2 = indices1[i + 1];
		long index3 = indices1[i + 2];
		Ogre::Vector3 point1 = vertices1[index1];
		Ogre::Vector3 point2 = vertices1[index2];
		Ogre::Vector3 point3 = vertices1[index3];
		normalVec = normalVector(point1, point2, point3);
		Ogre::Vector3 collPoint = closestPointOnTriangle(point1, point2, point3, ballPos);
		double dist = sqrt(pow((ballPos.x - collPoint.x), 2) + pow((ballPos.y - collPoint.y), 2) + pow((ballPos.z - collPoint.z), 2));
		if (dist < BALL_SIZE)
		{
			if (dist < shortestLength)
			{
				shortestLength = dist;
				chosenIndex = i;
				closestHitCoordinates = collPoint;
			}
		}
	}
	if (chosenIndex >= 0)
	{
		double diffDist = BALL_SIZE - shortestLength;
		ballPos += normalVec * diffDist;
		ballNode->setPosition(ballPos);
	}
	delete[] vertices1;
	delete[] indices1;
}

Ogre::Vector3 TestApplication::closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition)
{
	Ogre::Vector3 edge0 = point2 - point1;
	Ogre::Vector3 edge1 = point3 - point1;
	Ogre::Vector3 v0 = point1 - sourcePosition;

	float a = edge0.dotProduct(edge0);
	float b = edge0.dotProduct(edge1);
	float c = edge1.dotProduct(edge1);
	float d = edge0.dotProduct(v0);
	float e = edge1.dotProduct(v0);

	float det = a*c - b*b;
	float s = b*e - c*d;
	float t = b*d - a*e;

	if (s + t < det)
	{
		if (s < 0.f)
		{
			if (t < 0.f)
			{
				if (d < 0.f)
				{
					s = clamp(-d / a, 0.f, 1.f);
					t = 0.f;
				}
				else
				{
					s = 0.f;
					t = clamp(-e / c, 0.f, 1.f);
				}
			}
			else
			{
				s = 0.f;
				t = clamp(-e / c, 0.f, 1.f);
			}
		}
		else if (t < 0.f)
		{
			s = clamp(-d / a, 0.f, 1.f);
			t = 0.f;
		}
		else
		{
			float invDet = 1.f / det;
			s *= invDet;
			t *= invDet;
		}
	}
	else
	{
		if (s < 0.f)
		{
			float tmp0 = b + d;
			float tmp1 = c + e;
			if (tmp1 > tmp0)
			{
				float numer = tmp1 - tmp0;
				float denom = a - 2 * b + c;
				s = clamp(numer / denom, 0.f, 1.f);
				t = 1 - s;
			}
			else
			{
				t = clamp(-e / c, 0.f, 1.f);
				s = 0.f;
			}
		}
		else if (t < 0.f)
		{
			if (a + d > b + e)
			{
				float numer = c + e - b - d;
				float denom = a - 2 * b + c;
				s = clamp(numer / denom, 0.f, 1.f);
				t = 1 - s;
			}
			else
			{
				s = clamp(-e / c, 0.f, 1.f);
				t = 0.f;
			}
		}
		else
		{
			float numer = c + e - b - d;
			float denom = a - 2 * b + c;
			s = clamp(numer / denom, 0.f, 1.f);
			t = 1.f - s;
		}
	}
	Ogre::Vector3 ret = point1 + s * edge0 + t * edge1;
	return ret;
}

float TestApplication::clamp(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}

Ogre::Vector3 TestApplication::normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3)
{
	long e1x = point2.x - point1.x;
	long e1y = point2.y - point1.y;
	long e1z = point2.z - point1.z;

	long e2x = point3.x - point1.x;
	long e2y = point3.y - point1.y;
	long e2z = point3.z - point1.z;

	long nx = e1y*e2z - e1z*e2y;
	long ny = e1z*e2x - e1x*e2z;
	long nz = e1x*e2y - e1y*e2x;

	Ogre::Vector3 ret = Ogre::Vector3(nx, ny, nx);
	ret.normalise();
	return ret;

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