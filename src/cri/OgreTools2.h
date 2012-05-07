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

#ifndef __OGRETOOLS2
#define __OGRETOOLS2

#include <OgreException.h>
#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreManualObject.h>
#include <OgreEntity.h>
#include <OgreBillboardSet.h>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreWindowEventUtilities.h>
#include <OgreFrameListener.h>
#include <OgreMath.h>
#include <OgreMeshManager.h>
#include <OgreMeshSerializer.h>
#include <OgreLight.h>

#include <Procedural.h>

#include <boost/lexical_cast.hpp>

#include <string>
#include <vector>

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>

using namespace std;

// For creating a frame with the Y-axis point in direction 'dir'
Ogre::Quaternion CreateFrame(Ogre::Vector3 dir);

class Shape
{
	public:
	
	static const int Sphere = 1;
	static const int Cone = 2;
	static const int Cylinder = 3;
	static const int Torus = 4;
	static const int Cube = 5;
	
	static const int RoughCylinder = 6;
	
	Ogre::MaterialPtr material;
	Ogre::SceneManager * scene;
	Ogre::SceneNode * parent;
	Ogre::SceneNode * shapeNode;
	Ogre::Entity * entity;
	
	static void initialisePrimitives();
	void init(int shapeNumber, Ogre::SceneManager * sceneMan, Ogre::SceneNode * parentNode, string materialName);
	Shape(int shapeNumber, Ogre::SceneManager * sceneMan, Ogre::SceneNode * parentNode);
	Shape(int shapeNumber, Ogre::SceneManager * sceneMan, Ogre::SceneNode * parentNode, string materialName);
	~Shape();
	
	Ogre::Entity * getEntity();
	void attachToNode(Ogre::SceneNode * _parent);
	void detachFromNode();
	void setPosition(Ogre::Vector3 position);
	void setScale(Ogre::Vector3 scale);
	void setOrientation(Ogre::Quaternion q);
	void setColor(Ogre::ColourValue c);
	
	void alignToLineSegment(Ogre::Vector3 x1, Ogre::Vector3 x2, double thickness);
	
	// Temporary data for ray contact detection
	size_t vertex_count,index_count;
	Ogre::Vector3 * vertices;
	unsigned long * indices;
	
	int getIntersections(vector<Ogre::Vector3> & potentials, Ogre::Ray & ray);
	int getExtents(Ogre::Vector3 & minCorner, Ogre::Vector3 & maxCorner);
};

void getMeshInformation(const Ogre::MeshPtr mesh,
		size_t &vertex_count,
		Ogre::Vector3* &vertices,
		size_t &index_count,
		unsigned long* &indices,
		const Ogre::Vector3 &position,
		const Ogre::Quaternion &orient,
		const Ogre::Vector3 &scale);

void buildMesh(const aiScene* scene, const aiNode* node, const Ogre::MeshPtr& mesh, Ogre::AxisAlignedBox& aabb, float& radius);
Ogre::MeshPtr loadMesh(const string& path, int & gotMaterial);
void loadTexture(const std::string& resource_path);
Ogre::MeshPtr meshFromAssimpScene(const std::string& name, const aiScene* scene, int & gotMaterial);
int loadMaterialsForMesh(const std::string& resource_path, const aiScene* scene, const Ogre::MeshPtr& mesh);


#endif
