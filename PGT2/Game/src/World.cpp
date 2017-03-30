#include "World.h"
const float MOVE_SPEED = 10;
const int BALL_SIZE = 100;

const int SECTION_AMOUNT = 50;
World::World()
{

	worldObjects = std::vector<RigidBody*>();
	bodyCount = 0;
}

void World::createLight(Ogre::SceneManager* mSceneMgr)
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

void World::createTerrain(Ogre::SceneManager* mSceneMgr)
{
	Ogre::Entity* groundEntity = mSceneMgr->createEntity("Plane", "World.mesh");

	Ogre::MaterialPtr m_pMat = groundEntity->getSubEntity(0)->getMaterial()->clone("carrots");
	m_pMat->getTechnique(0)->getPass(0)->setAmbient(0, 1, 0);
	//	m_pMat->getTechnique(0)->getPass(0)->setDiffuse(1, 10, 3, 50);
	m_pMat->getTechnique(0)->getPass(0)->setDiffuse(3, 20, 5, 20);
	groundEntity->setMaterialName(m_pMat->getName());

	Ogre::SceneNode* groundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, -210, 0));
	groundNode->scale(Ogre::Vector3(500, 850, 500));
	groundNode->attachObject(groundEntity);
	groundBody = new RigidBody(groundNode, groundEntity);
	groundBody->setIsAwake(false);
	addRigidBody(groundBody);

	splitTerrainVertices();
}


void World::createSphere(Ogre::SceneManager* mSceneMgr)
{
	Ogre::SceneNode* ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	ballNode->setPosition(0, 300, 0);
	Ogre::Entity* sphereEntity = mSceneMgr->createEntity("Sphere", "sphere.mesh");

	ballNode->attachObject(sphereEntity);

	ballBody = RigidBody(ballNode, sphereEntity);



	registry.add(&ballBody, &gravity);
}

void World::addRigidBody(RigidBody* body)
{
	worldObjects.push_back(body);
	bodyCount++;
}


void World::splitTerrainVertices()
{
	getMeshInformation(groundBody->entity->getMesh(), terrainVertexCount, terrainVertices, terrainIndexCount, terrainIndices, groundBody->getPosition(), groundBody->getOrientation(), groundBody->node->getScale());

	int max = terrainIndexCount - 3;
	float lowestX = 1000000;
	float heightestX = -1000000;
	float lowestZ = 1000000;
	float heightestZ = -1000000;

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
		vertex.normal = normalVector(point1, point2, point3);
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

void World::getMeshInformation(const Ogre::MeshPtr mesh,
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

bool World::mouseMoved(const OIS::MouseEvent &arg)
{
	camera.camNode->yaw(Ogre::Degree(-arg.state.X.rel * 0.25f));
	return true;
}


void World::createCamera(Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow)
{

	camera = CustomCamera(mCamera, mSceneMgr, mWindow);


}

void World::setCameraFollow()
{
	camera.setCameraTarget(ballBody.node);
}


void World::createViewports()
{
	camera.createViewports();
}

void World::restartWorld()
{
	ballBody.setPosition(Ogre::Vector3(0, 200, 0));
}



void World::update(const Ogre::FrameEvent& evt)
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
	float duration = evt.timeSinceLastFrame;
	registry.updateForces(duration);
	ballBody.integrate(duration);


	checkBallCollision();
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
		ballBody.node->setPosition(ballPos);
		ballBody.setVelocity(Ogre::Vector3(ballBody.getVelocity().x, 0, ballBody.getVelocity().z));//temporary fix, gotta make contactregistry
	}
}

Ogre::Vector3 World::closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition)
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

float World::clamp(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}

int World::clamp(int n, int lower, int upper) {
	return std::max(lower, std::min(n, upper));
}

Ogre::Vector3 World::normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3)
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