

#include "Helper.h"
#include <stdio.h>

std::string Helper::to_string(int value)
{
	std::ostringstream os;
	os << value;
	return os.str();
}

void Helper::log(std::string name, int number)
{ 
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, "%s: %d\n", charAr, number);
}



void Helper::log(std::string name, Ogre::Real fl)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, " %s: %f\n", charAr, fl);
}

//werkt neit
void Helper::log(std::string name, size_t size)
{
	int number = static_cast<int>(size);
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, " %s: %f\n", charAr, number);
}

//werkt niet meer???
void Helper::log(std::string name, long l)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, " %s: %f\n", charAr, l);
}

void Helper::log(std::string name, Ogre::Vector2 vec)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	Ogre::Real x = vec.x;
	Ogre::Real y = vec.y;
	sprintf(buffer, " %s:  x: %f  y: %f\n", charAr, x, y);
}

void Helper::log(std::string name, Ogre::Vector3 vec)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	Ogre::Real x = vec.x;
	Ogre::Real y = vec.y;
	Ogre::Real z = vec.z;
	sprintf(buffer, " %s:  x: %f  y: %f  z: %f\n", charAr, x, y, z);
}

void Helper::log(std::string name, Ogre::Quaternion qaut)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	Ogre::Real x = qaut.x;
	Ogre::Real y = qaut.y;
	Ogre::Real z = qaut.z;
	Ogre::Real w = qaut.w;
	sprintf(buffer, " %s:  x: %f  y: %f  z: %f, w: %f\n", charAr, x, y, z, w);
}

bool Helper::vectorListContainsVector2(std::vector<Ogre::Vector2> &list, Ogre::Vector2 vec)
{
	for (size_t i = 0; i < list.size(); i++)
	{
		Ogre::Vector2 currVec = list.at(i);
		if (currVec.x == vec.x && currVec.y == vec.y)
		{
			return true;
		}
	}
	return false;
}


void Helper::getMeshInformation(const Ogre::MeshPtr mesh,
	size_t &vertex_count,
	Ogre::Vector3* &vertices,
	size_t &index_count,
	uint32_t* &indices,
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
	indices = new uint32_t[index_count];

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
			//  as second argument. So make it Ogre::Real, to avoid trouble when Ogre::Real will
			//  be comiled/typedefed as double:
			//      Ogre::Real* pReal;
			Ogre::Real* pReal;

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

Ogre::Vector3 Helper::normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3)
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

	Ogre::Vector3 ret = Ogre::Vector3(nx, ny, nz);
	ret.normalise();
	return ret;

}




