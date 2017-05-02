#include "World.h"
const Ogre::Real MOVE_SPEED = 10;
const int BALL_SIZE = 100;
const int SECTION_AMOUNT = 50;

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
	mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

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
	ballNode->setPosition(0, 300, -520);

	Ogre::Entity* sphereEntity = mSceneMgr->createEntity("Sphere", "sphere.mesh");

	ballNode->attachObject(sphereEntity);

	ballBody = Ball(ballNode, ballCameraNode, sphereEntity);

	addRigidBody(&ballBody);
}

void World::createBuilding(Ogre::Vector3 pos)
{
	Ogre::SceneNode* buildingNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	buildingNode->setPosition(pos);
	Ogre::Entity* buildingEntity = mSceneMgr->createEntity("Cube", "cube.mesh");
	buildingNode->attachObject(buildingEntity);
	Building buildingBody = Building(buildingNode, buildingEntity);
	//buildingBody.setIsAwake(false);

	addRigidBody(&buildingBody);

}

void World::createMesh(Ogre::Vector3* _verticesArr, int* _indicesArr, int _vertexCount, int _indexCount)
{

	//gebasseerd op: https://www.grahamedgecombe.com/blog/2011/08/05/custom-meshes-in-ogre3d en http://www.ogre3d.org/tikiwiki/Generating+A+Mesh



	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual("CustomMesh", "General");
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
	mesh->_setBounds(Ogre::AxisAlignedBox(-100, -100, -100, 100, 100, 100));

	/* notify the mesh that we're all ready */
	mesh->load();

	/* you can now create an entity/scene node based on your mesh, e.g. */
	
	Ogre::Entity *entity = mSceneMgr->createEntity("CustomEntity", "CustomMesh", "General");
	entity->setMaterialName("YourMaterial", "General");
	Ogre::SceneNode *node2 = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	node2->setPosition(0, 0, 0);
	node2->attachObject(entity);
	

}


void World::addRigidBody(RigidBody* body)
{
	worldObjects.push_back(body);
	bodyCount++;
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
			newSection.vertices = std::vector<Vertex>();
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
		Vertex vertex = Vertex();
		vertex.point1 = point1;
		vertex.point2 = point2;
		vertex.point3 = point3;
		vertex.point3 = point3;
		vertex.normal = Helper::normalVector(point1, point2, point3);
		std::vector<Ogre::Vector3> pointList = std::vector<Ogre::Vector3>();
		pointList.push_back(point1);
		pointList.push_back(point2);
		pointList.push_back(point3);

		std::vector<Ogre::Vector2> sectionList = getSections(middlePoint, false);
		for (size_t i = 0; i < sectionList.size(); i++)
		{
			Ogre::Vector2 currSection = sectionList.at(i);
			vertexSections[(int)currSection.x][(int)currSection.y].vertices.push_back(vertex);
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

std::vector<Ogre::Vector2> World::getSections(Ogre::Vector3 pos, bool surround)
{
	std::vector<Ogre::Vector2> ret = std::vector<Ogre::Vector2>();
	Ogre::Vector2 point1 = Ogre::Vector2(pos.x, pos.z);
	//Ogre::Vector2 point2 = Ogre::Vector2(pos.x - size.x, pos.z + size.y);
	//Ogre::Vector2 point3 = Ogre::Vector2(pos.x + size.x, pos.z - size.y);
	//Ogre::Vector2 point4 = Ogre::Vector2(pos.x + size.x, pos.z + size.y);

	Ogre::Vector2 adjustedPoint1 = Ogre::Vector2(point1.x + abs(lowestMapPos.x), point1.y + abs(lowestMapPos.y));
	//Ogre::Vector2 adjustedPoint2 = Ogre::Vector2(point2.x + abs(lowestMapPos.x), point2.y + abs(lowestMapPos.y));
	//Ogre::Vector2 adjustedPoint3 = Ogre::Vector2(point3.x + abs(lowestMapPos.x), point3.y + abs(lowestMapPos.y));
	//Ogre::Vector2 adjustedPoint4 = Ogre::Vector2(point4.x + abs(lowestMapPos.x), point4.y + abs(lowestMapPos.y));

	Ogre::Vector2 section1 = Ogre::Vector2(floor(adjustedPoint1.x / sectionSize.x), floor(adjustedPoint1.y / sectionSize.y));
	//Ogre::Vector2 section2 = Ogre::Vector2(floor(adjustedPoint2.x / sectionSize.x), floor(adjustedPoint2.y / sectionSize.y));
	//Ogre::Vector2 section3 = Ogre::Vector2(floor(adjustedPoint3.x / sectionSize.x), floor(adjustedPoint3.y / sectionSize.y));
	//Ogre::Vector2 section4 = Ogre::Vector2(floor(adjustedPoint4.x / sectionSize.x), floor(adjustedPoint4.y / sectionSize.y));


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
	
	//section2 = Ogre::Vector2(clamp((int)section2.x, 0, SECTION_AMOUNT - 1), clamp((int)section2.y, 0, SECTION_AMOUNT - 1));
	//section3 = Ogre::Vector2(clamp((int)section3.x, 0, SECTION_AMOUNT - 1), clamp((int)section3.y, 0, SECTION_AMOUNT - 1));
	//section4 = Ogre::Vector2(clamp((int)section4.x, 0, SECTION_AMOUNT - 1), clamp((int)section4.y, 0, SECTION_AMOUNT - 1));

	
	//if (!Helper::vectorListContainsVector2(ret, section2))
	//	ret.push_back(section2);
	//if (!Helper::vectorListContainsVector2(ret, section3))
	//	ret.push_back(section3);
	//if (!Helper::vectorListContainsVector2(ret, section4))
	//	ret.push_back(section4);




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
	camera.setCameraTarget(ballBody.cameraNode);
}


void World::createViewports()
{
	camera.createViewports();
}

void World::restartWorld()
{
	ballBody.setPosition(Ogre::Vector3(0, 200, 0));
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
		ballBody.addForce(direction);

	}
	if (InputManager::jDown)
	{
		Ogre::Vector3 direction = camera.camNode->_getDerivedOrientation() * Ogre::Vector3::NEGATIVE_UNIT_X;
		direction.y = 0;
		direction.normalise();
		direction = direction * MOVE_SPEED; // * speed
		ballBody.addForce(direction);
	}
	if (InputManager::kDown)
	{
		Ogre::Vector3 direction = camera.camNode->_getDerivedOrientation() * -Ogre::Vector3::NEGATIVE_UNIT_Z;
		direction.y = 0;
		direction.normalise();
		direction = direction * MOVE_SPEED; // * speed
		ballBody.addForce(direction);
	}
	if (InputManager::lDown)
	{
		Ogre::Vector3 direction = camera.camNode->_getDerivedOrientation() * -Ogre::Vector3::NEGATIVE_UNIT_X;
		direction.y = 0;
		direction.normalise();
		direction = direction * MOVE_SPEED; // * speed
		ballBody.addForce(direction);
	}
}

void World::generateContacts()
{
	// Set up the collision data structure
	cData.reset(maxContacts);
	cData.friction = (Ogre::Real)0.9;
	cData.restitution = (Ogre::Real)0.2;
	cData.tolerance = (Ogre::Real)0.1;

	checkBallCollision();
}

void World::updateObjects(Ogre::Real duration)
{
	for (int i = 0; i < bodyCount; i++)
	{
		worldObjects[i]->integrate(duration);
	}
}

void World::checkBallCollision()
{
	Ogre::Vector3 ballPos = ballBody.node->getPosition();
	std::vector<Ogre::Vector2> sectionList = getSections(ballPos, true);


	double shortestLength = 100000000000;
	int chosenIndex = -1;
	Ogre::Vector3 closestHitCoordinates;
	Ogre::Vector3 normalVec = Ogre::Vector3(-1, -1, -1);

	for (size_t i = 0; i < sectionList.size(); i++)
	{

		Ogre::Vector2 currSection = sectionList.at(i);
		std::vector<Vertex> vertexList = vertexSections[(int)currSection.x][(int)currSection.y].vertices;
		for (size_t j = 0; j < vertexList.size(); j++)
		{
			Vertex currVertex = vertexList.at(j);
			Ogre::Vector3 collPoint = closestPointOnTriangle(currVertex.point1, currVertex.point2, currVertex.point3, ballPos);
			double dist = sqrt(pow((ballPos.x - collPoint.x), 2) + pow((ballPos.y - collPoint.y), 2) + pow((ballPos.z - collPoint.z), 2));

			if (dist < BALL_SIZE && dist < shortestLength)
			{
				shortestLength = dist;
				chosenIndex = j;
				normalVec = currVertex.normal;
				closestHitCoordinates = collPoint;
			}
		}
	}


	if (chosenIndex >= 0)
	{
		double diffDist = BALL_SIZE - shortestLength;
		ballPos += normalVec * diffDist;
		//ballBody.node->setPosition(ballPos);
		//ballBody.setVelocity(Ogre::Vector3(ballBody.getVelocity().x, 0, ballBody.getVelocity().z));//temporary fix, gotta make contactregistry

		addContact(&cData, normalVec, closestHitCoordinates, diffDist, &ballBody);
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

