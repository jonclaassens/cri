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

#include "GTextureQuad.h"

GTextureQuad::GTextureQuad(const char * _name) : Gadget(_name)
{
	manual = NULL;
	subGraphicNode = NULL;
	graphicNode = NULL;
	type = Gadget::GADGET_TEXTUREQUAD;
	material.setNull();
	
	// Setup the Python interface
	//addProperty(string("CellXwidth"), PYINT_DOUBLE, &X_step);

}

void GTextureQuad::callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	
	// TODO:  Need to do some numeric checks??
	
	//if (!validateFloats(*msg))
	//{
	//	setStatus(status_levels::Error, "Map", "Message contained invalid floating point values (nans or infs)");
	//	return;
	//}

	// Check frame
	sourceFrame = Frame::findFrame(map->header.frame_id);

	if (sourceFrame < 0)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}
	
	if (!availableFrames[sourceFrame].valid)
	{
		if (visible && showable)
		{
			showable = false;
			
			// Detach all the persisting manual objects.
			parent->removeChild(graphicNode);
		}
		
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}

	if (map->info.width == 0 || map->info.height == 0)
	{
		publishStatus(Gadget::ERROR, "Occupancy map has no area");
		
		return;
	}

	double res = map->info.resolution;
 
	// TODO: to pad or not to pad to |8?
	int width = map->info.width;
	int height = map->info.height;

  //frame_ = msg->header.frame_id;
  //if (frame_.empty())
  //{
    //frame_ = "/map";
  //}

	// Create a texture array
	unsigned int pixelsSize = width * height;
	unsigned char * tempImage = new unsigned char[pixelsSize];
	memset(tempImage, 255, pixelsSize);

	bool mapStatusSet = false;
	unsigned int numPixelsToCopy = pixelsSize;
	
	if (pixelsSize != map->data.size())
	{
		std::stringstream ss;
		ss << "Data size doesn't match width * height: width = " << width
			<< ", height = " << height << ", data size = " << map->data.size();
		
		publishStatus(Gadget::ERROR, ss.str());
		
		mapStatusSet = true;

		// Keep going, but don't read past the end of the data.
		if(map->data.size() < pixelsSize )
		{
			numPixelsToCopy = map->data.size();
		}
	}

	for (unsigned int pixelIndex = 0; pixelIndex < numPixelsToCopy; pixelIndex++)
	{
		unsigned char val;
		
		if(map->data[pixelIndex] == 100)
			val = 0;
		else if(map->data[pixelIndex] == 0)
			val = 255;
		else
			val = 127;

		tempImage[pixelIndex] = val;
	}

	Ogre::DataStreamPtr pixelStream;

	pixelStream.bind(new Ogre::MemoryDataStream(tempImage, pixelsSize));
	
	static int texCount = 0;
	std::stringstream ss;
	ss << "MapTexture" << texCount++;
	
	try
	{
		texture = Ogre::TextureManager::getSingleton().loadRawData(ss.str(), 
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			pixelStream, width, height, Ogre::PF_L8, Ogre::TEX_TYPE_2D,
			0);
	}
	catch(Ogre::RenderingAPIException&)
	{
		Ogre::Image image;
		pixelStream->seek(0);
		float widthA = width;
		float heightA = height;
		
		if (width > height)
		{
			float aspect = heightA / widthA;
			
			widthA = 2048;
			heightA = widthA * aspect;
		}
		else
		{
			float aspect = widthA / heightA;
			
			heightA = 2048;
			widthA = heightA * aspect;
		}

		{
			std::stringstream ss;
			ss << "Map is larger than your graphics card supports.  Downsampled from [" << width << "x" << height << "] to [" << widthA << "x" << heightA << "]";
			publishStatus(Gadget::WARNING, ss.str());
		}

		// ROS_WARN("Failed to create full-size map texture, likely because your graphics card does not support textures of size > 2048.  Downsampling to [%d x %d]...", (int)width, (int)height);

		image.loadRawData(pixelStream, width, height, Ogre::PF_L8);
		image.resize(widthA, heightA, Ogre::Image::FILTER_NEAREST);
		
		ss << "Downsampled";
		
		texture = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
	}

	delete [] tempImage;

	// No material, create one!
	if (material.isNull())
	{
		static int count = 0;
		
		std::stringstream ss;
		
		ss << "OccupancyMapMaterial" << count++;
		
		material = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->setReceiveShadows(false);
		material->getTechnique(0)->setLightingEnabled(false);
		material->setDepthBias(-16.0f, 0.0f);
		material->setCullingMode(Ogre::CULL_NONE);
		material->setDepthWriteEnabled(false);
	}

	Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
	
	Ogre::TextureUnitState* texUnit = NULL;
	
	if (pass->getNumTextureUnitStates() > 0)
	{
		texUnit = pass->getTextureUnitState(0);
	}
	else
	{
		texUnit = pass->createTextureUnitState();
	}

	texUnit->setTextureName(texture->getName());
	texUnit->setTextureFiltering(Ogre::TFO_NONE);

	static int map_count = 0;
	std::stringstream ss2;
	ss2 << "MapObject" << map_count++;
	
	if (!subGraphicNode)
	{
		subGraphicNode = graphicNode->createChildSceneNode();
	}
	
	subGraphicNode->setPosition(Ogre::Vector3(map->info.origin.position.x,
		map->info.origin.position.y,
		map->info.origin.position.z));
		
	subGraphicNode->setOrientation(Ogre::Quaternion(map->info.origin.orientation.w,
		map->info.origin.orientation.x,
		map->info.origin.orientation.y,
		map->info.origin.orientation.z));
	
	if (!manual)
	{
		manual = scene->createManualObject(ss2.str());
		subGraphicNode->attachObject(manual);

		manual->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
		{
			// First triangle
			{
				// Bottom left
				manual->position( 0.0f, 0.0f, 0.0f );
				manual->textureCoord(0.0f, 0.0f);
				manual->normal( 0.0f, 0.0f, 1.0f );

				// Top right
				manual->position( res * width, res * height, 0.0f );
				manual->textureCoord(1.0f, 1.0f);
				manual->normal( 0.0f, 0.0f, 1.0f );

				// Top left
				manual->position( 0.0f, res * height, 0.0f );
				manual->textureCoord(0.0f, 1.0f);
				manual->normal( 0.0f, 0.0f, 1.0f );
			}

			// Second triangle
			{
				// Bottom left
				manual->position( 0.0f, 0.0f, 0.0f );
				manual->textureCoord(0.0f, 0.0f);
				manual->normal( 0.0f, 0.0f, 1.0f );

				// Bottom right
				manual->position( res * width, 0.0f, 0.0f );
				manual->textureCoord(1.0f, 0.0f);
				manual->normal( 0.0f, 0.0f, 1.0f );

				// Top right
				manual->position( res * width, res * height, 0.0f );
				manual->textureCoord(1.0f, 1.0f);
				manual->normal( 0.0f, 0.0f, 1.0f );
			}
		}
		
		manual->end();
	}

	publishStatus(Gadget::OKAY, "Okay");
	
	// Draw under
	//if (draw_under_)
	//{
	//	manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
	//}

}

void GTextureQuad::applyProperties()
{
	// No properties for a textured quad
}

void GTextureQuad::show(bool value)
{	
	// Ensure that we have the ogre scene node which is responsible for the gadget
	if (!graphicNode)
	{
		graphicNode = parent->createChildSceneNode();
	}
	
	// Make the graphic node visible if it can be
	if (visible)
	{
		if (!value)
		{
			if (graphicNode->getParentSceneNode() == parent)
			{
				parent->removeChild(graphicNode);
			}
			visible = false;
		}
	}
	else
	{
		// Show the graphics if there is something available, otherwise set 'showable' false so that
		// the message callback handles attaching the graphics to the appropriate scene node.
		
		if (value)
		{

			if (graphicNode->getParentSceneNode() != parent)
			{ 
				parent->addChild(graphicNode);
			}
				
			visible = true;
			showable = true;
		}
	}
}

void GTextureQuad::subscribe(string _topic)
{
	topic = _topic;
	ogSub = master->n->subscribe<nav_msgs::OccupancyGrid> (_topic.c_str(), 1, &GTextureQuad::callback, this);
}

void GTextureQuad::update()
{
	if (graphicNode)
	{
		if (sourceFrame < 0)
		{
			return;
		}
		
		// Update graphic motion with its frame
		tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
		tf::Vector3 & pos = tran.getOrigin();
		tf::Quaternion rot = tran.getRotation();
		
		graphicNode->setPosition(pos.x(), pos.y(), pos.z());
		graphicNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
	}
}

GTextureQuad::~GTextureQuad()
{
	// Kill the additional scene node and manual object
	if (manual)
	{
		scene->destroyManualObject(manual);
	}
	
	if (graphicNode)
	{
		// Need to remove children from a node before killing it
		parent->removeChild(graphicNode);
		scene->destroySceneNode(graphicNode);
	}
}
