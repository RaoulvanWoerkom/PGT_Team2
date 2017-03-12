#include "TestApplication.h"
#include "../OgreText.h"
#include <sstream>


#include "Ogre.h"
#include "math.h"

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

	groundEntity = mSceneMgr->createEntity("ground");
	//mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(groundEntity);
	groundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	groundNode->attachObject(groundEntity);

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
	ballEntity = mSceneMgr->createEntity("Sphere", "sphere.mesh");

	ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, 100, 0));
	ballNode->attachObject(ballEntity);

	
}

bool iDown = false;
bool jDown = false;
bool kDown = false;
bool lDown = false;
bool yDown = false;
bool hDown = false;
bool TestApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	//showScore(1);

	Ogre::Vector3 movePos = Ogre::Vector3(0, 0, 0);
	
	if (jDown)
		movePos.x = -moveSpeed;
	if (hDown)
		movePos.y = -moveSpeed;
	if (iDown)
		movePos.z = -moveSpeed;
	if (lDown)
		movePos.x = moveSpeed;
	if (yDown)
		movePos.y = moveSpeed;
	if (kDown)
		movePos.z = moveSpeed;

	ballNode->translate(movePos * evt.timeSinceLastFrame, Ogre::Node::TS_LOCAL);
	

	CheckBallCollision(groundNode, groundEntity);


	return BaseApplication::frameRenderingQueued(evt);
}

void TestApplication::CheckBallCollision(Ogre::SceneNode* node1, Ogre::Entity* entity1)
{

	size_t vertex_count1, index_count1;
	Ogre::Vector3* vertices1;
	unsigned long* indices1;

	GetMeshInformation(entity1->getMesh(), vertex_count1, vertices1, index_count1, indices1, node1->getPosition(), node1->getOrientation(), node1->getScale());

	int max = vertex_count1 - 1;
	for (size_t i = 0; i < max; i++)
	{
		Ogre::Vector3 point1 = vertices1[i];
		Ogre::Vector3 point2 = vertices1[i+1];
		Ogre::Vector3 ballPos = ballNode->getPosition();

		std::vector<Ogre::Vector3> collisionCoordinates = FindLineSphereIntersections(point1, point2, ballPos, 99);
		if (collisionCoordinates.size() > 0)
		{
			OutputDebugStringW(L"HITTTTTTTTTTTTTT\n");
		}

	}
	

	delete[] vertices1;
	delete[] indices1;
}

std::vector<Ogre::Vector3> TestApplication::FindLineSphereIntersections(Ogre::Vector3 linePoint0, Ogre::Vector3 linePoint1, Ogre::Vector3 circleCenter, double circleRadius)
{
	std::vector<Ogre::Vector3> ret;
	double cx = circleCenter.x;
	double cy = circleCenter.y;
	double cz = circleCenter.z;

	double px = linePoint0.x;
	double py = linePoint0.y;
	double pz = linePoint0.z;

	double vx = linePoint1.x - px;
	double vy = linePoint1.y - py;
	double vz = linePoint1.z - pz;

	double A = vx * vx + vy * vy + vz * vz;
	double B = 2.0 * (px * vx + py * vy + pz * vz - vx * cx - vy * cy - vz * cz);
	double C = px * px - 2 * px * cx + cx * cx + py * py - 2 * py * cy + cy * cy +
		pz * pz - 2 * pz * cz + cz * cz - circleRadius * circleRadius;

	// discriminant
	double D = B * B - 4 * A * C;

	if (D < 0)
	{
		return ret;
	}

	double t1 = (-B - sqrt(D)) / (2.0 * A);

	Ogre::Vector3 solution1 = Ogre::Vector3(linePoint0.x * (1 - t1) + t1 * linePoint1.x,
		linePoint0.y * (1 - t1) + t1 * linePoint1.y,
		linePoint0.z * (1 - t1) + t1 * linePoint1.z);
	if (D == 0)
	{
		ret.resize(1);
		ret[0] = solution1;
		return ret;
	}

	double t2 = (-B + sqrt(D)) / (2.0 * A);
	Ogre::Vector3 solution2 = Ogre::Vector3(linePoint0.x * (1 - t2) + t2 * linePoint1.x,
		linePoint0.y * (1 - t2) + t2 * linePoint1.y,
		linePoint0.z * (1 - t2) + t2 * linePoint1.z);

	if (abs(t1 - 0.5) < abs(t2 - 0.5))
	{
		ret.resize(2);
		ret[0] = solution1;
		ret[1] = solution2;
		return ret;
	}
	ret.resize(2);
	ret[0] = solution2;
	ret[1] = solution1;
	return ret;
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

	//delete textItem;
}



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
		case OIS::KC_Y:
			yDown = true;
			break;
		case OIS::KC_H:
			hDown = true;
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
	case OIS::KC_Y:
		yDown = false;
		break;
	case OIS::KC_H:
		hDown = false;
		break;
	default:
		break;
	}

	return BaseApplication::keyReleased(ke);
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