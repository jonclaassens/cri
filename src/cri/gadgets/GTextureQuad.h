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

#ifndef __GTEXTUREQUAD
#define __GTEXTUREQUAD

#include "CRI.h"
#include "IntersectionMath.h"

#include <nav_msgs/OccupancyGrid.h>

#include <Procedural.h>

#include "OgreTools2.h"
#include "GFrame.h"

#include <OGRE/OgreTextureManager.h>

class GTextureQuad : public Gadget
{
	public:
	
	// Parameters of the textured quad
	
	void show(bool value);
	void update();
		
	// Our contribution to this gadget's Python interface
	void applyProperties();
	
	// Callback
	void callback(const nav_msgs::OccupancyGrid::ConstPtr & map);
	
	void subscribe(string _topic);
	
	GTextureQuad(const char * _name);
	~GTextureQuad();
	
	private:
	
	ros::Subscriber ogSub;
	string topic;
	
	Ogre::MaterialPtr material;
	Ogre::TexturePtr texture;
	Ogre::ManualObject * manual;
	Ogre::SceneNode * subGraphicNode;
};

#endif
