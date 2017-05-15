#include "World.h"
#include "MeshGenerator.h"
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreMovablePlane.h>
#include <OgreMeshManager.h>

const Ogre::Real MOVE_SPEED = 10;

const int BALL_SIZE = 100;
const int SECTION_AMOUNT = 10;

size_t World::bodyCount = 0;
std::vector<RigidBody*> World::worldObjects;

Ogre::SceneManager* World::mSceneMgr = NULL;

World::World() :
	resolver(maxContacts * 8)
{
	cData.contactArray = contacts;
	worldObjects = std::vector<RigidBody*>();
	bodyCount = 0;
}

World::~World()
{
}



void World::createLight()
{
	mSceneMgr->setAmbientLight(Ogre::ColourValue::White);
	// Directional light
	directionalLight = mSceneMgr->createLight("DirectionalLight");
	directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);

	directionalLight->setDiffuseColour(Ogre::ColourValue(.3f, .3f, .3f));
	directionalLight->setSpecularColour(Ogre::ColourValue(.3f, .3f, .3f));

	directionalLight->setDirection(Ogre::Vector3(0, -1, 1));
}

void World::createTerrain()
{
	Ogre::Entity* groundEntity = mSceneMgr->createEntity("Plane", "World.mesh");

	Ogre::MaterialPtr m_pMat = groundEntity->getSubEntity(0)->getMaterial()->clone("carrots");
	m_pMat->getTechnique(0)->getPass(0)->setAmbient(0, 1, 0);
	//	m_pMat->getTechnique(0)->getPass(0)->setDiffuse(1, 10, 3, 50);
	m_pMat->getTechnique(0)->getPass(0)->setDiffuse(3, 20, 5, 20);
	groundEntity->setMaterialName(m_pMat->getName());

	Ogre::SceneNode* groundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, -200, 0));
	groundNode->scale(Ogre::Vector3(500, 850, 500));
	groundNode->attachObject(groundEntity);
	groundBody = new RigidBody(groundNode, groundEntity);
	groundBody->setIsAwake(false);

	splitTerrainVertices();
}


void World::createSphere()
{
	Ogre::SceneNode* ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	Ogre::SceneNode* ballCameraNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	ballNode->setPosition(0, 300, 0);


	createSphereMesh("mySphereMesh", 100, 64, 64);
	Ogre::Entity* sphereEntity = mSceneMgr->createEntity("Sphere", "mySphereMesh");

	ballNode->attachObject(sphereEntity);

	ballBody = new Ball(ballNode, ballCameraNode, sphereEntity);
	ballBody->entity->setMaterialName("Ball/Skin");

}

void World::createHouse(Ogre::SceneManager* mSceneMgr) {

	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
		"Test/ColourTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_EMISSIVE);



	Ogre::SceneNode* thisSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	thisSceneNode->setPosition(0, 250, 0);
	MeshGenerator meshG = MeshGenerator(thisSceneNode);
	meshG.createColourCube();


	Ogre::Entity* thisEntity = mSceneMgr->createEntity("cc", "ColourCube");
	thisSceneNode->attachObject(thisEntity);
	//thisEntity->setMaterialName("Ogre/Earring");
	thisEntity->setMaterialName("Ogre/Skin");

	houseBody = new RigidBody(thisSceneNode, thisEntity);
	addRigidBody(houseBody);
}


void World::createBuilding(Ogre::Vector3 pos)
{
	Ogre::SceneNode* buildingNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	buildingNode->setPosition(pos);
	buildingNode->setScale(4, 10, 4);
	int randNum = rand() % (1000000000);
	std::string meshName = "Cube" + std::to_string(randNum);
	Ogre::Entity* buildingEntity = mSceneMgr->createEntity(meshName, "cube.mesh");


	buildingNode->attachObject(buildingEntity);
	Building* buildingBody = new Building(buildingNode, buildingEntity);
	buildingBody->entity->setMaterialName("Building/Wall");
	//buildingBody.setIsAwake(false);

	addRigidBody(buildingBody);
	addObjectVertices(buildingBody);

	Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create("Skin", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	
	Ogre::TextureUnitState* tuisTexture = mat->getTechnique(0)->getPass(0)->createTextureUnitState("DrySkin");

}

void World::createSphereMesh(const std::string& strName, const float r, const int nRings = 16, const int nSegments = 16)
{
	Ogre::MeshPtr pSphere = Ogre::MeshManager::getSingleton().createManual(strName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	Ogre::SubMesh *pSphereVertex = pSphere->createSubMesh();

	pSphere->sharedVertexData = new Ogre::VertexData();
	Ogre::VertexData* vertexData = pSphere->sharedVertexData;

	// define the vertex format
	Ogre::VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
	currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
	// normals
	vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
	currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
	// two dimensional texture coordinates
	vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
	currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

	// allocate the vertex buffer
	vertexData->vertexCount = (nRings + 1) * (nSegments + 1);
	Ogre::HardwareVertexBufferSharedPtr vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);
	float* pVertex = static_cast<float*>(vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

	// allocate index buffer
	pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
	pSphereVertex->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	Ogre::HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

	float fDeltaRingAngle = (Ogre::Math::PI / nRings);
	float fDeltaSegAngle = (2 * Ogre::Math::PI / nSegments);
	unsigned short wVerticeIndex = 0;

	// Generate the group of rings for the sphere
	for (int ring = 0; ring <= nRings; ring++) {
		float r0 = r * sinf(ring * fDeltaRingAngle);
		float y0 = r * cosf(ring * fDeltaRingAngle);

		// Generate the group of segments for the current ring
		for (int seg = 0; seg <= nSegments; seg++) {
			float x0 = r0 * sinf(seg * fDeltaSegAngle);
			float z0 = r0 * cosf(seg * fDeltaSegAngle);

			// Add one vertex to the strip which makes up the sphere
			*pVertex++ = x0;
			*pVertex++ = y0;
			*pVertex++ = z0;

			Ogre::Vector3 vNormal = Ogre::Vector3(x0, y0, z0).normalisedCopy();
			*pVertex++ = vNormal.x;
			*pVertex++ = vNormal.y;
			*pVertex++ = vNormal.z;

			*pVertex++ = (float)seg / (float)nSegments;
			*pVertex++ = (float)ring / (float)nRings;

			if (ring != nRings) {
				// each vertex (except the last) has six indices pointing to it
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex;
				*pIndices++ = wVerticeIndex + nSegments;
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex + 1;
				*pIndices++ = wVerticeIndex;
				wVerticeIndex++;
			}
		}; // end for seg
	} // end for ring

	  // Unlock
	vBuf->unlock();
	iBuf->unlock();
	// Generate face list
	pSphereVertex->useSharedVertices = true;

	// the original code was missing this line:
	pSphere->_setBounds(Ogre::AxisAlignedBox(Ogre::Vector3(-r, -r, -r), Ogre::Vector3(r, r, r)), false);
	pSphere->_setBoundingSphereRadius(r);
	// this line makes clear the mesh is loaded (avoids memory leaks)
	pSphere->load();
}


Ogre::Entity* World::createCustomEntity(Ogre::Vector3* _verticesArr, int* _indicesArr, int _vertexCount, int _indexCount, Ogre::String matName)
{

	//gebasseerd op: https://www.grahamedgecombe.com/blog/2011/08/05/custom-meshes-in-ogre3d en http://www.ogre3d.org/tikiwiki/Generating+A+Mesh

	int randNum = rand() % (1000000000);
	std::string meshName = "CustomMesh" + std::to_string(randNum);
	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(meshName, "General");
	Ogre::SubMesh *subMesh = mesh->createSubMesh();


	/* create the vertex data structure */
	mesh->sharedVertexData = new Ogre::VertexData;
	mesh->sharedVertexData->vertexCount = _vertexCount;

	/* declare how the vertices will be represented */
	Ogre::VertexDeclaration *decl = mesh->sharedVertexData->vertexDeclaration;
	size_t offset = 0;

	/* the first three floats of each vertex represent the position */
	decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
	offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

	/*
	decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
	offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
	*/


	/* create the vertex buffer */
	Ogre::HardwareVertexBufferSharedPtr vertexBuffer =
		Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
			offset, mesh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

	/* lock the buffer so we can get exclusive access to its data */

	float* _vertices = new float[_vertexCount * 3];

	/* populate the buffer with some data */
	for (size_t i = 0; i < _vertexCount; i++)
	{
		Ogre::Vector3 currVertex = _verticesArr[i];
		_vertices[i * 3] = currVertex.x;
		_vertices[i * 3 + 1] = currVertex.y;
		_vertices[i * 3 + 2] = currVertex.z;
		/*
		Ogre::Vector3 vNormal = currVertex.normalisedCopy();

		_vertices[i * 6 + 3] = vNormal.x;
		_vertices[i * 6 + 4] = vNormal.y;
		_vertices[i * 6 + 5] = vNormal.z;
		*/
	}

	vertexBuffer->writeData(0, vertexBuffer->getSizeInBytes(), _verticesArr, true);
	Ogre::VertexBufferBinding* bind = mesh->sharedVertexData->vertexBufferBinding;
	bind->setBinding(0, vertexBuffer);

	/* create the index buffer */
	Ogre::HardwareIndexBufferSharedPtr indexBuffer = Ogre::HardwareBufferManager::getSingleton().
		createIndexBuffer(
			Ogre::HardwareIndexBuffer::IT_16BIT,
			_indexCount,
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);


	uint16_t *_indices = static_cast<uint16_t *>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));

	for (size_t i = 0; i < _indexCount; i++)
	{
		_indices[i] = _indicesArr[i];
	}

	/* unlock the buffer */
	indexBuffer->unlock();


	/* attach the buffers to the mesh */
	mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vertexBuffer);
	subMesh->useSharedVertices = true;
	subMesh->indexData->indexBuffer = indexBuffer;
	subMesh->indexData->indexCount = _indexCount;
	subMesh->indexData->indexStart = 0;

	/* set the bounds of the mesh */
	mesh->_setBounds(Ogre::AxisAlignedBox(-1000, -1000, -1000, 1000, 1000, 1000));

	/* notify the mesh that we're all ready */
	mesh->load();

	/* you can now create an entity/scene node based on your mesh, e.g. */
	Ogre::Entity *entity = mSceneMgr->createEntity(meshName);
	entity->setMaterialName(matName);
	//Ogre::SceneNode *node2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	//node2->setPosition(Ogre::Vector3(0,0,0));
	//node2->attachObject(entity);
	

	
	//RigidBody *newBody = new RigidBody(node2, entity);
	//addRigidBody(newBody);

	return entity;
}


void World::addRigidBody(RigidBody* body)
{
	worldObjects.push_back(body);
	bodyCount++;
}

void World::addObjectVertices(RigidBody* body)
{
	Ogre::Vector3 pos = body->getPosition();
	std::vector<Ogre::Vector2> sectionList = getSections(pos, false);
	Ogre::Vector2 coordinates = sectionList.at(0);

	VerticeSection newSection = vertexSections[(int)coordinates.x][(int)coordinates.y];
	newSection.objects.push_back(body);
	newSection.objectCount++;
	vertexSections[(int)coordinates.x][(int)coordinates.y] = newSection;
}

void World::splitTerrainVertices()
{
	Helper::getMeshInformation(groundBody->entity->getMesh(), terrainVertexCount, terrainVertices, terrainIndexCount, terrainIndices, groundBody->getPosition(), groundBody->getOrientation(), groundBody->node->getScale());

	int max = terrainIndexCount - 3;
	Ogre::Real lowestX = 1000000;
	Ogre::Real heightestX = -1000000;
	Ogre::Real lowestZ = 1000000;
	Ogre::Real heightestZ = -1000000;

	for (size_t i = 0; i < max; i += 3)
	{
		long index1 = terrainIndices[i];
		long index2 = terrainIndices[i + 1];
		long index3 = terrainIndices[i + 2];
		Ogre::Vector3 point1 = terrainVertices[index1];
		Ogre::Vector3 point2 = terrainVertices[index2];
		Ogre::Vector3 point3 = terrainVertices[index3];
		Ogre::Vector3 middlePoint = (point1 + point2 + point3) / 3;
		if (middlePoint.x < lowestX)
		{
			lowestX = middlePoint.x;
		}
		if (middlePoint.x > heightestX)
		{
			heightestX = middlePoint.x;
		}
		if (middlePoint.z < lowestZ)
		{
			lowestZ = middlePoint.z;
		}
		if (middlePoint.z > heightestZ)
		{
			heightestZ = middlePoint.z;
		}

	}
	lowestMapPos = Ogre::Vector2(lowestX, lowestZ);

	Ogre::Vector2 mapSize = Ogre::Vector2(abs(heightestX - lowestX), abs(heightestZ - lowestZ));
	sectionSize = Ogre::Vector2(mapSize.x / SECTION_AMOUNT + 1, mapSize.y / SECTION_AMOUNT + 1);
	for (int y = 0; y < SECTION_AMOUNT; y++)
	{
		for (int x = 0; x < SECTION_AMOUNT; x++)
		{
			VerticeSection newSection = VerticeSection();
			newSection.minPoint = Ogre::Vector2(lowestX + sectionSize.x * x, lowestZ + sectionSize.y * y);
			newSection.maxPoint = Ogre::Vector2(lowestX + sectionSize.x * (x + 1), lowestZ + sectionSize.y * (y + 1));
			newSection.terrainFaces = std::vector<Face>();
			vertexSections[x][y] = newSection;
			//vertexSections[x][y] 

		}
	}




	for (int i = 0; i < max; i += 3)
	{
		long index1 = terrainIndices[i];
		long index2 = terrainIndices[i + 1];
		long index3 = terrainIndices[i + 2];
		Ogre::Vector3 point1 = terrainVertices[index1];
		Ogre::Vector3 point2 = terrainVertices[index2];
		Ogre::Vector3 point3 = terrainVertices[index3];
		Ogre::Vector3 middlePoint = (point1 + point2 + point3) / 3;
		Face face = Face();
		face.point1 = point1;
		face.point2 = point2;
		face.point3 = point3;
		face.normal = Helper::normalVector(point1, point2, point3);

		std::vector<Ogre::Vector2> sectionList = getSections(middlePoint, false);
		for (size_t i = 0; i < sectionList.size(); i++)
		{
			Ogre::Vector2 currSection = sectionList.at(i);
			vertexSections[(int)currSection.x][(int)currSection.y].terrainFaces.push_back(face);
		}


	}



}

std::vector<Ogre::Vector2> World::getSections(std::vector<Ogre::Vector3> positions)
{
	std::vector<Ogre::Vector2> ret = std::vector<Ogre::Vector2>();

	for (size_t i = 0; i < positions.size(); i++)
	{
		Ogre::Vector3 currPos = positions.at(i);
		std::vector<Ogre::Vector2> currSectorList = getSections(currPos);
		for (size_t j = 0; j < currSectorList.size(); j++)
		{
			Ogre::Vector2 currSectionPos = currSectorList.at(j);
			if (!Helper::vectorListContainsVector2(ret, currSectionPos))
			{
				ret.push_back(currSectionPos);
			}
		}
	}
	return ret;
}

std::vector<Ogre::Vector2> World::getSections(Ogre::Vector3* positions, int size)
{
	std::vector<Ogre::Vector2> ret = std::vector<Ogre::Vector2>();

	for (size_t i = 0; i < size; i++)
	{
		Ogre::Vector3 currPos = positions[i];
		std::vector<Ogre::Vector2> currSectorList = getSections(currPos);
		for (size_t j = 0; j < currSectorList.size(); j++)
		{
			Ogre::Vector2 currSectionPos = currSectorList.at(j);
			if (!Helper::vectorListContainsVector2(ret, currSectionPos))
			{
				ret.push_back(currSectionPos);
			}
		}
	}
	return ret;
}

std::vector<Ogre::Vector2> World::getSections(Ogre::Vector3 pos, bool surround)
{
	std::vector<Ogre::Vector2> ret = std::vector<Ogre::Vector2>();
	Ogre::Vector2 point1 = Ogre::Vector2(pos.x, pos.z);

	Ogre::Vector2 adjustedPoint1 = Ogre::Vector2(point1.x + abs(lowestMapPos.x), point1.y + abs(lowestMapPos.y));

	Ogre::Vector2 section1 = Ogre::Vector2(floor(adjustedPoint1.x / sectionSize.x), floor(adjustedPoint1.y / sectionSize.y));

	section1 = Ogre::Vector2(clamp((int)section1.x, 0, SECTION_AMOUNT - 1), clamp((int)section1.y, 0, SECTION_AMOUNT - 1));
	if (!Helper::vectorListContainsVector2(ret, section1))
		ret.push_back(section1);

	if (surround)
	{
		for (int x = -1; x < 2; x++)
		{
			for (int y = -1; y < 2; y++)
			{
				if (x != 0 || y != 0)
				{
					Ogre::Vector2 currVec = Ogre::Vector2(clamp((int)section1.x + x, 0, SECTION_AMOUNT - 1), clamp((int)section1.y + y, 0, SECTION_AMOUNT - 1));
					if (!Helper::vectorListContainsVector2(ret, currVec))
						ret.push_back(currVec);
				}
			}
		}
	}



	return ret;
}



bool World::mouseMoved(const OIS::MouseEvent &arg)
{
	camera.camNode->pitch(Ogre::Degree(-arg.state.Y.rel * 0.25f));
	camera.camNode->yaw(Ogre::Degree(-arg.state.X.rel * 0.25f));
	return true;
}


void World::createCamera(Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow)
{

	camera = CustomCamera(mCamera, mSceneMgr, mWindow);


}

void World::setCameraFollow()
{
	camera.setCameraTarget(ballBody->cameraNode);
}


void World::createViewports()
{
	camera.createViewports();
}

void World::restartWorld()
{
	ballBody->setPosition(Ogre::Vector3(0, 200, 0));
}

void World::update(const Ogre::FrameEvent evt)
{
	Ogre::Vector3 movePos = Ogre::Vector3(0, 0, 0);
	if (InputManager::iDown)
	{
		Ogre::Vector3 direction = camera.camNode->_getDerivedOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
		direction.y = 0;
		direction.normalise();
		direction = direction * MOVE_SPEED; // * speed
		ballBody->addForce(direction);

	}
	if (InputManager::jDown)
	{
		Ogre::Vector3 direction = camera.camNode->_getDerivedOrientation() * Ogre::Vector3::NEGATIVE_UNIT_X;
		direction.y = 0;
		direction.normalise();
		direction = direction * MOVE_SPEED; // * speed
		ballBody->addForce(direction);
	}
	if (InputManager::kDown)
	{
		Ogre::Vector3 direction = camera.camNode->_getDerivedOrientation() * -Ogre::Vector3::NEGATIVE_UNIT_Z;
		direction.y = 0;
		direction.normalise();
		direction = direction * MOVE_SPEED; // * speed
		ballBody->addForce(direction);
	}
	if (InputManager::lDown)
	{
		Ogre::Vector3 direction = camera.camNode->_getDerivedOrientation() * -Ogre::Vector3::NEGATIVE_UNIT_X;
		direction.y = 0;
		direction.normalise();
		direction = direction * MOVE_SPEED; // * speed
		ballBody->addForce(direction);
	}
}

void World::generateContacts()
{
	// Set up the collision data structure
	cData.reset(maxContacts);
	cData.friction = (Ogre::Real)0.9;
	cData.restitution = (Ogre::Real)0.2;
	cData.tolerance = (Ogre::Real)0.1;
	emptySectionObjects();
	populateSections();
	checkBallCollision();
	checkWorldCollision();
	
}

void World::updateObjects(Ogre::Real duration)
{
	for (int i = 0; i < worldObjects.size(); i++)
	{
		RigidBody* currRigidBody = worldObjects[i];
		if (currRigidBody->isDestroyed)
			continue;
		currRigidBody->integrate(duration);
	}

	//ballBody is not in worldObjects because it has unique collision detection
	ballBody->integrate(duration);
}


void World::emptySectionObjects()
{
	for (int y = 0; y < SECTION_AMOUNT; y++)
	{
		for (int x = 0; x < SECTION_AMOUNT; x++)
		{
			VerticeSection currSection = vertexSections[(int)x][(int)y];
			currSection.objects.empty();
			currSection.objectCount = 0;
		}
	}
}

/**
go through all the sections and all objects to check if there is movement. Then do collision check per section. (ball collision is excluded)
*/
void World::populateSections()
{

	//loop trough all objects
	std::vector<RigidBody*>::iterator i = worldObjects.begin();
	while (i != worldObjects.end())
	{
		RigidBody* currBody = (*i);
		bool isDestroyed = currBody->isDestroyed;
		if (isDestroyed)
		{
			delete currBody;
			i = worldObjects.erase(i);
			
		}
		else
		{
			Ogre::Vector3 currPos = currBody->getPosition();
			Ogre::Vector3* boundingBox = currBody->getBoundingBox(); //bounding box is already in world space coordinates
			std::vector<Ogre::Vector2> sectionList = getSections(boundingBox, 8); //holds the sections... i think

			for (size_t i = 0; i < sectionList.size(); i++)
			{
				Ogre::Vector2 currSectionCoor = sectionList.at(i);
				VerticeSection currSection = vertexSections[(int)currSectionCoor.x][(int)currSectionCoor.y];
				currSection.objects.push_back(currBody);
				currSection.objectCount++;
			}
			++i;
		}
	}

}

/**
Checks whether the ball hits an object
*/
void World::checkBallCollision()
{
	Ogre::Vector3 ballPos = ballBody->node->getPosition();
	std::vector<Ogre::Vector2> sectionList = getSections(ballPos, true); //gets all sections surrounding the ball
	double shortestLength = 100000000000;
	int chosenIndex = -1;
	Ogre::Vector3 closestHitCoordinates;
	Ogre::Vector3 normalVec = Ogre::Vector3(-1, -1, -1);
	for (size_t i = 0; i < sectionList.size(); i++) //loop through all sections
	{

		Ogre::Vector2 currSectionCoor = sectionList.at(i);
		VerticeSection currSection = vertexSections[(int)currSectionCoor.x][(int)currSectionCoor.y]; //get section object

		std::vector<Face> terrainFaceList = currSection.terrainFaces;
		for (size_t j = 0; j < terrainFaceList.size(); j++) //loop through all the terrain vertices inside the section object
		{
			Face currFace = terrainFaceList.at(j);
			//collpoint is point on face closest to the ball
			Ogre::Vector3 collPoint = closestPointOnTriangle(currFace.point1, currFace.point2, currFace.point3, ballPos);
			double dist = sqrt(pow((ballPos.x - collPoint.x), 2) + pow((ballPos.y - collPoint.y), 2) + pow((ballPos.z - collPoint.z), 2));

			if (dist < BALL_SIZE && dist < shortestLength) //if collpoint is closes than the radius of the ball TO the ball
			{
				shortestLength = dist;
				chosenIndex = j;
				normalVec = currFace.normal;
				closestHitCoordinates = collPoint;
			}
		}

		for (size_t j = 0; j < currSection.objectCount; j++)
		{
			RigidBody* currBody = currSection.objects.at(j);
			if (!currBody->isDestroyed && currBody->canCollide)
			{
				std::vector<Face> bodyFaceList = currBody->faces;
				for (size_t k = 0; k < currBody->faces.size(); k++)
				{
					Face currFace = bodyFaceList.at(k);
					Ogre::Vector3 collPoint = closestPointOnTriangle(currFace.point1, currFace.point2, currFace.point3, ballPos);
					double dist = sqrt(pow((ballPos.x - collPoint.x), 2) + pow((ballPos.y - collPoint.y), 2) + pow((ballPos.z - collPoint.z), 2));

					

					if (dist < BALL_SIZE && dist < shortestLength)
					{
						if (currBody->hitBoxContainsPoint(collPoint))
						{
							Helper::log("test", ballPos);
						}

						if (currBody->isBreakable)
						{
							//Building* building = dynamic_cast<Building*>(currBody);
							//building->fracture();
							//building->isDestroyed = true;
							//mSceneMgr->destroyEntity(building->entity);
							//break;
						}
					}
				}
			}
		}
	}
	if (chosenIndex >= 0)
	{
		double diffDist = BALL_SIZE - shortestLength;
		addContact(&cData, normalVec, closestHitCoordinates, diffDist, ballBody);
	}
}

void World::checkWorldCollision()
{
	for (size_t i = 0; i < worldObjects.size(); i++)
	{
		RigidBody* currBody = worldObjects[i];
		Ogre::Vector3 currPos = currBody->node->getPosition();
		std::vector<Ogre::Vector2> sectionList = getSections(currPos);
		VerticeSection currSection = vertexSections[(int)sectionList[0].x][(int)sectionList[0].y];



		for (size_t j = 0; j < currSection.objectCount; j++)
		{
			RigidBody* otherBody = currSection.objects[j];
			if (otherBody != currBody)
			{

			}
		}
	}
}

void World::addContact(CollisionData *data, Ogre::Vector3 contactNormal, Ogre::Vector3 contactPoint, Ogre::Real penetration, RigidBody *sphere)
{
	Contact* contact = data->contacts;
	contact->contactNormal = contactNormal;
	contact->penetration = penetration;
	contact->contactPoint = contactPoint;
	contact->setBodyData(sphere, NULL,
		data->friction, data->restitution);

	data->addContacts(1);
}

Ogre::Vector3 World::closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition)
{
	Ogre::Vector3 edge0 = point2 - point1;
	Ogre::Vector3 edge1 = point3 - point1;
	Ogre::Vector3 v0 = point1 - sourcePosition;

	Ogre::Real a = edge0.dotProduct(edge0);
	Ogre::Real b = edge0.dotProduct(edge1);
	Ogre::Real c = edge1.dotProduct(edge1);
	Ogre::Real d = edge0.dotProduct(v0);
	Ogre::Real e = edge1.dotProduct(v0);

	Ogre::Real det = a*c - b*b;
	Ogre::Real s = b*e - c*d;
	Ogre::Real t = b*d - a*e;

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
			Ogre::Real invDet = 1.f / det;
			s *= invDet;
			t *= invDet;
		}
	}
	else
	{
		if (s < 0.f)
		{
			Ogre::Real tmp0 = b + d;
			Ogre::Real tmp1 = c + e;
			if (tmp1 > tmp0)
			{
				Ogre::Real numer = tmp1 - tmp0;
				Ogre::Real denom = a - 2 * b + c;
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
				Ogre::Real numer = c + e - b - d;
				Ogre::Real denom = a - 2 * b + c;
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
			Ogre::Real numer = c + e - b - d;
			Ogre::Real denom = a - 2 * b + c;
			s = clamp(numer / denom, 0.f, 1.f);
			t = 1.f - s;
		}
	}
	Ogre::Vector3 ret = point1 + s * edge0 + t * edge1;
	return ret;
}

Ogre::Real World::clamp(Ogre::Real n, Ogre::Real lower, Ogre::Real upper) {
	return std::max(lower, std::min(n, upper));
}

int World::clamp(int n, int lower, int upper) {
	return std::max(lower, std::min(n, upper));
}


