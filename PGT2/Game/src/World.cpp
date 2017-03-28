#include "World.h"
const float MOVE_SPEED = 10;
const int BALL_SIZE = 100;


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

void World::createPlane(Ogre::SceneManager* mSceneMgr)
{
	groundEntity = mSceneMgr->createEntity("Plane", "World.mesh");

	Ogre::MaterialPtr m_pMat = groundEntity->getSubEntity(0)->getMaterial()->clone("carrots");
	m_pMat->getTechnique(0)->getPass(0)->setAmbient(0, 1, 0);
	//	m_pMat->getTechnique(0)->getPass(0)->setDiffuse(1, 10, 3, 50);
	m_pMat->getTechnique(0)->getPass(0)->setDiffuse(3, 20, 5, 20);
	groundEntity->setMaterialName(m_pMat->getName());

	groundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, -210, 0));
	groundNode->scale(Ogre::Vector3(500, 850, 500));
	groundNode->attachObject(groundEntity);
	RigidBody* groundBody = new RigidBody(groundNode, groundEntity);
	groundBody->setIsAwake(false);
	addRigidBody(groundBody);
}


void World::createSphere(Ogre::SceneManager* mSceneMgr)
{
	Ogre::SceneNode* ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	ballNode->setPosition(0, 300, 0);
	Ogre::Entity* sphereEntity = mSceneMgr->createEntity("Sphere", "sphere.mesh");
	
	ballNode->attachObject(sphereEntity);

	ballBody = RigidBody(ballNode, sphereEntity);

	

	registry.add(&ballBody , &gravity);
}

void World::addRigidBody(RigidBody* body)
{
	worldObjects.push_back(body);
	bodyCount++;
}


void World::splitVertices()
{
	for (unsigned short i = 0; i < bodyCount; ++i)
	{
		size_t vertex_count, index_count;
		Ogre::Vector3* vertices;
		unsigned long* indices;
		RigidBody* body = worldObjects.at(i);
		getMeshInformation(body->entity->getMesh(), vertex_count, vertices, index_count, indices, body->getPosition(), body->getOrientation(), body->node->getScale());

	}
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
	/*
	bool infiniteClip =
		mRoot->getRenderSystem()->getCapabilities()->hasCapability(
		Ogre::RSC_INFINITE_FAR_PLANE);

	if (infiniteClip)
		mCamera->setFarClipDistance(0);
	else
		mCamera->setFarClipDistance(1390000);
		*/

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
	checkBallCollision(groundEntity, groundNode);
}


void World::checkBallCollision(Ogre::Entity* otherEntity, Ogre::SceneNode* otherSceneNode)
{

	size_t vertex_count, index_count;
	Ogre::Vector3* vertices;
	unsigned long* indices;
	getMeshInformation(otherEntity->getMesh(), vertex_count, vertices, index_count, indices, otherSceneNode->getPosition(), otherSceneNode->getOrientation(), otherSceneNode->getScale());

	double shortestLength = 100000000000;
	int chosenIndex = -1;
	Ogre::Vector3 closestHitCoordinates;
	Ogre::Vector3 normalVec = Ogre::Vector3(-1, -1, -1);
	Ogre::Vector3 ballPos = ballBody.node->getPosition();

	int max = index_count - 3;
	for (size_t i = 0; i < max; i += 3)
	{
		long index1 = indices[i];
		long index2 = indices[i + 1];
		long index3 = indices[i + 2];
		Ogre::Vector3 point1 = vertices[index1];
		Ogre::Vector3 point2 = vertices[index2];
		Ogre::Vector3 point3 = vertices[index3];
		
		Ogre::Vector3 collPoint = closestPointOnTriangle(point1, point2, point3, ballPos);
		double dist = sqrt(pow((ballPos.x - collPoint.x), 2) + pow((ballPos.y - collPoint.y), 2) + pow((ballPos.z - collPoint.z), 2));
		if (dist < BALL_SIZE)
		{
			if (dist < shortestLength)
			{
				shortestLength = dist;
				chosenIndex = i;
				normalVec = normalVector(point1, point2, point3);
				closestHitCoordinates = collPoint;
			}
		}
	}
	if (chosenIndex >= 0)
	{
		double diffDist = BALL_SIZE - shortestLength;
		ballPos += normalVec * diffDist;
		//Helper::log("normal", normalVec);
		ballBody.node->setPosition(ballPos);
		ballBody.setVelocity(Ogre::Vector3(ballBody.getVelocity().x, 0, ballBody.getVelocity().z));//temporary fix, gotta make contactregistry
	}
	delete[] vertices;
	delete[] indices;
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