/*
 * Copyright (C) 2012, Jonathan Claassens (jclaassens@csir.co.za)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "OgreTools2.h"

#include <ros/ros.h>

#include <OgreSubMesh.h>
#include <resource_retriever/retriever.h>
#include <boost/filesystem.hpp>

#include <ogre_tools/stl_loader.h>

int nameNum = 0;

Ogre::Quaternion CreateFrame(Ogre::Vector3 dir)
{
	Ogre::Matrix3 R;
	
	Ogre::Vector3 up(0, 1, 0);
	Ogre::Vector3 v1;
	
	dir.normalise();
	
	v1 = dir.crossProduct(up);
	if (v1.normalise() < 0.0001)
	{
		if (dir[1] > 0)
		{
			v1[0] = -1; v1[1] = 0; v1[2] = 0;
		}
		else
		{
			v1[0] = 1; v1[1] = 0; v1[2] = 0; 
		}
	}
	
	Ogre::Vector3 v2;
	v2 = v1.crossProduct(dir);
	
	R.SetColumn(0, v1);
	R.SetColumn(1, dir);
	R.SetColumn(2, v2);
	
	return Ogre::Quaternion(R);
}

void Shape::initialisePrimitives()
{
	Procedural::SphereGenerator().setRadius(0.5f).setUTile(.5f).realizeMesh("basicSphere");
	Procedural::ConeGenerator().setRadius(0.5f).setHeight(1.0f).setPosition(0, -0.5, 0).setNumSegBase(24).setNumSegHeight(2).setUTile(.5f).realizeMesh("basicCone");
	Procedural::CylinderGenerator().setHeight(1.0f).setRadius(1.0f).setUTile(.5f).realizeMesh("basicCylinder");
	Procedural::TorusGenerator().setRadius(1.f).setNumSegCircle(32).setSectionRadius(0.1f).setUTile(5).setVTile(0.5f).realizeMesh("basicTorus");
	Procedural::BoxGenerator().setSizeX(1.f).setSizeY(1.f).setSizeZ(1.f).realizeMesh("basicCube");
	
	//Procedural::CapsuleGenerator().setPosition(0, 0.5, 0).setRadius(1.f).setNumRings(4).setNumSegments(5).setHeight(0.5f).realizeMesh("roughCapsule");
	Procedural::CylinderGenerator().setPosition(0, 0.0, 0).setHeight(1.f).setRadius(1.0f).setUTile(.5f).realizeMesh("roughCylinder");
}

Shape::Shape(int shapeNumber, Ogre::SceneManager * sceneMan, Ogre::SceneNode * parentNode, string materialName)
{
	init(shapeNumber, sceneMan, parentNode, materialName);
}

void Shape::init(int shapeNumber, Ogre::SceneManager * sceneMan, Ogre::SceneNode * parentNode, string materialName)
{
	// Copy scene manager pointer
	scene = sceneMan;
	
	// Copy parent scene node
	parent = parentNode;
	
	// Create the child node
	shapeNode = scene->createSceneNode("ShapeNode" + boost::lexical_cast<std::string>(nameNum));
	switch (shapeNumber)
	{
		case Sphere:
			entity = scene->createEntity("basicSphere");
			break;
		case Cone:
			entity = scene->createEntity("basicCone");
			break;
		case Cylinder:
			entity = scene->createEntity("basicCylinder");
			break;
		case Torus:
			entity = scene->createEntity("basicTorus");
			break;
		case Cube:
			entity = scene->createEntity("basicCube");
			break;
		case RoughCylinder:
			entity = scene->createEntity("roughCylinder");
			break;
	}
	
	// Assign material as a copy from Template/White in the surfaces material script
	string matName = "ShapeMat" + boost::lexical_cast<std::string>(nameNum++);
	material = Ogre::MaterialManager::getSingleton().create(matName, 
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	
	Ogre::MaterialPtr templateMat = Ogre::MaterialManager::getSingleton().getByName(materialName);
	templateMat->copyDetailsTo(material);

	entity->setMaterialName(matName);
	
	// Attach the shape to the scene node
	shapeNode->attachObject(entity);
	
	if (parent)
	{
		attachToNode(parent);
	}
}

void Shape::attachToNode(Ogre::SceneNode * _parent)
{
	if (parent)
	{
		parent->removeChild(shapeNode);
	}
	
	parent = _parent;
	parent->addChild(shapeNode);
}

void Shape::detachFromNode()
{
	if (parent)
	{
		parent->removeChild(shapeNode);
		parent = NULL;
	}
}

Shape::Shape(int shapeNumber, Ogre::SceneManager * sceneMan, Ogre::SceneNode * parentNode)
{
	init(shapeNumber, sceneMan, parentNode, string("Template/White"));
}

Shape::~Shape()
{
	// Material is destroyed implicitly
	
	shapeNode->detachObject(entity);
	scene->destroyEntity(entity);
	if (parent)
	{
		parent->removeChild(shapeNode);
	}
	scene->destroySceneNode(shapeNode);
}
	
Ogre::Entity * Shape::getEntity()
{
	return entity;
}

void Shape::setPosition(Ogre::Vector3 position)
{
	shapeNode->setPosition(position);
}

void Shape::setScale(Ogre::Vector3 scale)
{
	shapeNode->setScale(scale);
}

void Shape::setOrientation(Ogre::Quaternion q)
{
	shapeNode->setOrientation(q);
}

void Shape::setColor(Ogre::ColourValue c)
{
	material->setAmbient(c);
	material->setDiffuse(c);
}

void Shape::alignToLineSegment(Ogre::Vector3 x1, Ogre::Vector3 x2, double thickness)
{
	Ogre::Quaternion q = CreateFrame(x2 - x1);
	setPosition(x1);
	setOrientation(q);
	setScale(Ogre::Vector3(thickness, (x2 - x1).normalise(), thickness));
}

void getMeshInformation(const Ogre::MeshPtr mesh,
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
        Ogre::SubMesh* submesh = mesh->getSubMesh( i );
 
        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
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
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
 
        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
 
        if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
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
 
            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
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
        if( ibuf.isNull() ) continue; // need to check if index buffer is valid (which will be not if the mesh doesn't have triangles like a pointcloud)
 
        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);
 
        unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);
 
 
        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;
        size_t index_start = index_data->indexStart;
        size_t last_index = numTris*3 + index_start;
 
        if (use32bitindexes)
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>( offset );
            }
 
        else
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[ index_offset++ ] = static_cast<unsigned long>( pShort[k] ) +
                    static_cast<unsigned long>( offset );
            }
 
        ibuf->unlock();
        current_offset = next_offset;
    }
}

int Shape::getIntersections(vector<Ogre::Vector3> & potentials, Ogre::Ray & ray)
{	
	int i, count;
	
	count = 0;
	// Get the mesh information
	getMeshInformation(entity->getMesh(), vertex_count, vertices, index_count, indices, 
		shapeNode->_getDerivedPosition(), 
		shapeNode->_getDerivedOrientation(), 
		shapeNode->_getDerivedScale());

	// Loop through every triangle in the mesh
	for(i = 0; i < index_count; i += 3)
	{
		//vertices that make up the triangle
		Ogre::Vector3 A = vertices[indices[i]];
		Ogre::Vector3 B = vertices[indices[i+1]];
		Ogre::Vector3 C = vertices[indices[i+2]];

		pair<bool, float> tmp = Ogre::Math::intersects(ray, A, B, C, true, false);

		// We've found an intersection, return this result through the vector
		if (tmp.first)
		{
			count++;
			
			// Determine exact intersection position by treating the triangle as a 
			// plane and finding the intersection point with the ray
			Ogre::Plane plane(A, B, C);
			std::pair< bool, Ogre::Real > tmp2 = Ogre::Math::intersects(ray, plane);
			Ogre::Vector3 pos = ray.getOrigin() + ray.getDirection() * tmp2.second;
			
			potentials.push_back(pos);
		}
	}
	
	delete [] vertices;
	delete [] indices;
	
	return count;
}

int Shape::getExtents(Ogre::Vector3 & minCorner, Ogre::Vector3 & maxCorner)
{
	int i, j;
	
	// Get the mesh information
	getMeshInformation(entity->getMesh(), vertex_count, vertices, index_count, indices, 
		shapeNode->_getDerivedPosition(), 
		shapeNode->_getDerivedOrientation(), 
		shapeNode->_getDerivedScale());

	if (index_count == 0)
	{
		minCorner = Ogre::Vector3::ZERO;
		maxCorner = Ogre::Vector3::ZERO;
		return -1;
	}

	Ogre::Vector3 X = vertices[indices[0]];
	minCorner = X;
	maxCorner = X;

	// Loop through every vertex in the mesh
	for(i = 1; i < index_count; i += 1)
	{
		// Grow extents based on each vertex position
		Ogre::Vector3 X = vertices[indices[i]];

		for (j = 0; j < 3; j++)
		{
			if (X[j] < minCorner[j])
			{
				minCorner[j] = X[j];
			}
			
			if (X[j] > maxCorner[j])
			{
				maxCorner[j] = X[j];
			}
		}
	}
	
	delete [] vertices;
	delete [] indices;
	
	return 0;
}

// Shamelessly borrowed from 'rviz'
class ResourceIOStream : public Assimp::IOStream
{
	public:
	ResourceIOStream(const resource_retriever::MemoryResource& res)
		: res_(res)
		, pos_(res.data.get())
	{}

	~ResourceIOStream()
	{}

	size_t Read(void* buffer, size_t size, size_t count)
	{
		size_t to_read = size * count;
		if (pos_ + to_read > res_.data.get() + res_.size)
		{
			to_read = res_.size - (pos_ - res_.data.get());
		}

		memcpy(buffer, pos_, to_read);
		pos_ += to_read;

		return to_read;
	}

	size_t Write( const void* buffer, size_t size, size_t count) { ROS_BREAK(); return 0; }

	aiReturn Seek( size_t offset, aiOrigin origin)
	{
		uint8_t* new_pos = 0;
		switch (origin)
		{
			case aiOrigin_SET:
				new_pos = res_.data.get() + offset;
				break;
			case aiOrigin_CUR:
				new_pos = pos_ + offset; // TODO is this right?  can offset really not be negative
				break;
			case aiOrigin_END:
				new_pos = res_.data.get() + res_.size - offset; // TODO is this right?
				break;
			default:
			ROS_BREAK();
		}

		if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
		{
			return aiReturn_FAILURE;
		}

		pos_ = new_pos;
		return aiReturn_SUCCESS;
	}

	size_t Tell() const
	{
		return pos_ - res_.data.get();
	}

	size_t FileSize() const
	{
		return res_.size;
	}

	void Flush() 
	{}

	private:
	resource_retriever::MemoryResource res_;
	uint8_t* pos_;
};

class ResourceIOSystem : public Assimp::IOSystem
{
	public:
	ResourceIOSystem()
	{
	}

	~ResourceIOSystem()
	{
	}

	// Check whether a specific file exists
	bool Exists(const char* file) const
	{
		// Ugly -- two retrievals where there should be one (Exists + Open)
		// resource_retriever needs a way of checking for existence
		// TODO: cache this
		resource_retriever::MemoryResource res;
		try
		{
			res = retriever_.get(file);
		}
		catch (resource_retriever::Exception& e)
		{
			return false;
		}

		return true;
	}

	// Get the path delimiter character we'd like to see
	char getOsSeparator() const
	{
		return '/';
	}

	// ... and finally a method to open a custom stream
	Assimp::IOStream* Open(const char* file, const char* mode)
	{
		ROS_ASSERT(mode == std::string("r") || mode == std::string("rb"));

		// Ugly -- two retrievals where there should be one (Exists + Open)
		// resource_retriever needs a way of checking for existence
		resource_retriever::MemoryResource res;
		try
		{
			res = retriever_.get(file);
		}
		catch (resource_retriever::Exception& e)
		{
			return 0;
		}

		return new ResourceIOStream(res);
	}

	void Close(Assimp::IOStream* stream) { delete stream; }

private:
// Heh?  Don't like mutables...
	mutable resource_retriever::Retriever retriever_;
};

// Cribbed from Rviz, after being
// "Mostly stolen from gazebo"
void buildMesh(const aiScene* scene, const aiNode* node, const Ogre::MeshPtr& mesh, Ogre::AxisAlignedBox& aabb, float& radius)
{
	if (!node)
	{
		return;
	}

	aiMatrix4x4 transform = node->mTransformation;
	aiNode *pnode = node->mParent;
	while (pnode)
	{
		// Don't convert to y-up orientation, which is what the root node in
		// Assimp does
		if (pnode->mParent != NULL)
			transform = pnode->mTransformation * transform;
		pnode = pnode->mParent;
	}

	aiMatrix3x3 rotation(transform);
	aiMatrix3x3 inverse_transpose_rotation(rotation);
	inverse_transpose_rotation.Inverse();
	inverse_transpose_rotation.Transpose();

	for (uint32_t i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

		Ogre::SubMesh* submesh = mesh->createSubMesh();
		submesh->useSharedVertices = false;
		submesh->vertexData = new Ogre::VertexData();
		Ogre::VertexData* vertex_data = submesh->vertexData;
		Ogre::VertexDeclaration* vertex_decl = vertex_data->vertexDeclaration;

		size_t offset = 0;
		// positions
		vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
		offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

		// normals
		if (input_mesh->HasNormals())
		{
			vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
			offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
		}

		// texture coordinates (only support 1 for now)
		if (input_mesh->HasTextureCoords(0))
		{
			vertex_decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
			offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
		}

		// todo vertex colors

		// allocate the vertex buffer
		vertex_data->vertexCount = input_mesh->mNumVertices;
		Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertex_decl->getVertexSize(0),
			vertex_data->vertexCount,
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
			false);

		vertex_data->vertexBufferBinding->setBinding(0, vbuf);
		float* vertices = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

		// Add the vertices
		for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
		{
			aiVector3D p = input_mesh->mVertices[j];
			p *= transform;
			*vertices++ = p.x;
			*vertices++ = p.y;
			*vertices++ = p.z;

			Ogre::Vector3 v(p.x, p.y, p.z);
			aabb.merge(v);
			float dist = v.length();
			if (dist > radius)
			{
				radius = dist;
			}

			if (input_mesh->HasNormals())
			{
				aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
				*vertices++ = n.x;
				*vertices++ = n.y;
				*vertices++ = n.z;
			}

			if (input_mesh->HasTextureCoords(0))
			{
				*vertices++ = input_mesh->mTextureCoords[0][j].x;
				*vertices++ = input_mesh->mTextureCoords[0][j].y;
			}
		}

		// calculate index count
		submesh->indexData->indexCount = 0;
		for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
		{
			aiFace& face = input_mesh->mFaces[j];
			submesh->indexData->indexCount += face.mNumIndices;
		}

		// allocate index buffer
		submesh->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
			Ogre::HardwareIndexBuffer::IT_16BIT,
			submesh->indexData->indexCount,
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
			false);

		Ogre::HardwareIndexBufferSharedPtr ibuf = submesh->indexData->indexBuffer;
		uint16_t* indices = static_cast<uint16_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

		// add the indices
		for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
		{
			aiFace& face = input_mesh->mFaces[j];
			for (uint32_t k = 0; k < face.mNumIndices; ++k)
			{
				*indices++ = face.mIndices[k];
			}
		}

		// Unlock
		vbuf->unlock();
		ibuf->unlock();
	}

	for (uint32_t i=0; i < node->mNumChildren; ++i)
	{
		buildMesh(scene, node->mChildren[i], mesh, aabb, radius);
	}
}

void loadTexture(const std::string& resource_path)
{
	if (!Ogre::TextureManager::getSingleton().resourceExists(resource_path))
	{
		resource_retriever::Retriever retriever;
		resource_retriever::MemoryResource res;
		try
		{
			res = retriever.get(resource_path);
		}
		catch (resource_retriever::Exception& e)
		{
			ROS_ERROR("%s", e.what());
		}

		if (res.size != 0)
		{
			Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
			Ogre::Image image;
			std::string extension = boost::filesystem::extension(boost::filesystem::path(resource_path));

			if (extension[0] == '.')
			{
				extension = extension.substr(1, extension.size() - 1);
			}

			try
			{
				image.load(stream, extension);
				Ogre::TextureManager::getSingleton().loadImage(resource_path, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
			}
			catch (Ogre::Exception& e)
			{
				ROS_ERROR("Could not load texture [%s]: %s", resource_path.c_str(), e.what());
			}
		}
	}
}

// Cribbed from Rviz, after being...
// "Mostly cribbed from gazebo"
int loadMaterialsForMesh(const std::string& resource_path, const aiScene* scene, const Ogre::MeshPtr& mesh)
{
	std::vector<Ogre::MaterialPtr> material_lookup;
	
	// No materials?  Use a default 
	if (scene->mNumMaterials == 0)
	{
		return -1;
	}
	else
	{
		for (uint32_t i = 0; i < scene->mNumMaterials; i++)
		{
			std::stringstream ss;
			ss << resource_path << "Material" << i;
			Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ss.str(), "General", true);
			material_lookup.push_back(mat);

			Ogre::Technique* tech = mat->getTechnique(0);
			Ogre::Pass* pass = tech->getPass(0);
			pass->setLightingEnabled(true);

			aiMaterial *amat = scene->mMaterials[i];

			Ogre::ColourValue diffuse(1.0, 1.0, 1.0, 1.0);
			Ogre::ColourValue specular(1.0, 1.0, 1.0, 1.0);
			Ogre::ColourValue ambient(0.5, 0.5, 0.5, 1.0);

			for (uint32_t j=0; j < amat->mNumProperties; j++)
			{
				aiMaterialProperty *prop = amat->mProperties[j];
				std::string propKey = prop->mKey.data;

				if (propKey == "$tex.file")
				{
					aiString texName;
					aiTextureMapping mapping;
					uint32_t uvIndex;
					amat->GetTexture(aiTextureType_DIFFUSE,0, &texName, &mapping, &uvIndex);

					// Assume textures are in paths relative to the mesh
					std::string texture_path = boost::filesystem::path(resource_path).parent_path().string() + "/" + texName.data;
					loadTexture(texture_path);
					Ogre::TextureUnitState* tu = pass->createTextureUnitState();
					tu->setTextureName(texture_path);
				}
				else if (propKey == "$clr.diffuse")
				{
					aiColor3D clr;
					amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
					diffuse = Ogre::ColourValue(clr.r, clr.g, clr.b);
				}
				else if (propKey == "$clr.ambient")
				{
					aiColor3D clr;
					amat->Get(AI_MATKEY_COLOR_AMBIENT, clr);

					// Most of are DAE files don't have ambient color defined
					if (clr.r > 0 && clr.g > 0 && clr.b > 0)
					{
						ambient = Ogre::ColourValue(clr.r, clr.g, clr.b);
					}
				}
				else if (propKey == "$clr.specular")
				{
					aiColor3D clr;
					amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
					specular = Ogre::ColourValue(clr.r, clr.g, clr.b);
				}
				else if (propKey == "$clr.emissive")
				{
					aiColor3D clr;
					amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
					mat->setSelfIllumination(clr.r, clr.g, clr.b);
				}
				else if (propKey == "$clr.opacity")
				{
					float o;
					amat->Get(AI_MATKEY_OPACITY, o);
					diffuse.a = o;
				}
				else if (propKey == "$mat.shininess")
				{
					float s;
					amat->Get(AI_MATKEY_SHININESS, s);
					mat->setShininess(s);
				}
				else if (propKey == "$mat.shadingm")
				{
					int model;
					amat->Get(AI_MATKEY_SHADING_MODEL, model);
					switch(model)
					{
						case aiShadingMode_Flat:
							mat->setShadingMode(Ogre::SO_FLAT);
							break;
						case aiShadingMode_Phong:
							mat->setShadingMode(Ogre::SO_PHONG);
							break;
						case aiShadingMode_Gouraud:
							default:
							mat->setShadingMode(Ogre::SO_GOURAUD);
							break;
					}
				}
			}

			int mode = aiBlendMode_Default;
			amat->Get(AI_MATKEY_BLEND_FUNC, mode);
			switch(mode)
			{
				case aiBlendMode_Additive:
					mat->setSceneBlending(Ogre::SBT_ADD);
					break;
				case aiBlendMode_Default:
				default:
					{
						if (diffuse.a < 0.99)
						{
							pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
						}
						else
						{
							pass->setSceneBlending(Ogre::SBT_REPLACE);
						}
					}
					break;
			}

			mat->setAmbient(ambient);
			mat->setDiffuse(diffuse);
			specular.a = diffuse.a;
			mat->setSpecular(specular);
		}

		for (uint32_t i = 0; i < mesh->getNumSubMeshes(); ++i)
		{
			mesh->getSubMesh(i)->setMaterialName(material_lookup[scene->mMeshes[i]->mMaterialIndex]->getName());
		}
	}
	
	return 0;
}

Ogre::MeshPtr meshFromAssimpScene(const std::string& name, const aiScene* scene, int & gotMaterial)
{
	if (!scene->HasMeshes())
	{
		ROS_ERROR("No meshes found in file [%s]", name.c_str());
		return Ogre::MeshPtr();
	}

	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(name, "General");

	Ogre::AxisAlignedBox aabb(Ogre::AxisAlignedBox::EXTENT_NULL);
	float radius = 0.0f;
	buildMesh(scene, scene->mRootNode, mesh, aabb, radius);

	mesh->_setBounds(aabb);
	mesh->_setBoundingSphereRadius(radius);
	mesh->buildEdgeList();

	gotMaterial = loadMaterialsForMesh(name, scene, mesh);

	mesh->load();

	return mesh;
}

Ogre::MeshPtr loadMesh(const string& path, int & gotMaterial)
{
	// Interpret the path as a standard path and see if that works
	if (Ogre::MeshManager::getSingleton().resourceExists(path))
	{
		return Ogre::MeshManager::getSingleton().getByName(path);
	}
	else
	// It may be a resource_retriever formatted path
	{
			boost::filesystem::path model_path(path);
#if BOOST_FILESYSTEM_VERSION == 3
			std::string ext = model_path.extension().string();
#else
			std::string ext = model_path.extension();
#endif

		// Mesh files are interpretted directly
		if (ext == ".mesh" || ext == ".MESH")
		{
			resource_retriever::Retriever retriever;
			resource_retriever::MemoryResource res;
			try
			{
				res = retriever.get(path);
			}
			catch (resource_retriever::Exception& e)
			{
				ROS_ERROR("%s", e.what());
				return Ogre::MeshPtr();
			}

			// Got nothing, return nothing
			if (res.size == 0)
			{
				return Ogre::MeshPtr();
			}

			// Fetch
			Ogre::MeshSerializer ser;
			Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
			Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(path, "rviz");
			ser.importMesh(stream, mesh.get());
			
			gotMaterial = 1;
			return mesh;
		}
		// Need a special loader for .stl and variant files
		else if (ext == ".stl" || ext == ".STL" || ext == ".stlb" || ext == ".STLB")
		{
			resource_retriever::Retriever retriever;
			resource_retriever::MemoryResource res;
			try
			{
				res = retriever.get(path);
			}
			catch (resource_retriever::Exception& e)
			{
				ROS_ERROR("%s", e.what());
				return Ogre::MeshPtr();
			}

			// Got nothing, return nothing
			if (res.size == 0)
			{
				return Ogre::MeshPtr();
			}

			ogre_tools::STLLoader loader;
			if (!loader.load(res.data.get()))
			{
				ROS_ERROR("Failed to load file [%s]", path.c_str());
				return Ogre::MeshPtr();
			}

			// TODO:  rid ourselves of ogre_tools altogether and determine if a material was found
			gotMaterial = 0;
			return loader.toMesh(path);
		}
		// For everything else, try the Assimp importer
		else
		{
			Assimp::Importer importer;
			importer.SetIOHandler(new ResourceIOSystem());
			const aiScene* scene = importer.ReadFile(path, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
			if (!scene)
			{
				ROS_ERROR("Could not load resource [%s]: %s", path.c_str(), importer.GetErrorString());
				return Ogre::MeshPtr();
			}

			int gotMaterial;
			return meshFromAssimpScene(path, scene, gotMaterial);
		}
	}
	
	return Ogre::MeshPtr();
}
