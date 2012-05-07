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

#ifndef __GACTIVEMARKER
#define __GACTIVEMARKER

#include "CRI.h"

// To resolve some compatibility issue between ROS and CEGUI/OGRE headers
#ifdef Success
#undef Success
#endif

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cri/MarkerScript.h>

#include <boost/lexical_cast.hpp>

#include <Procedural.h>
#include "MovableText.h"
#include "OgreTools2.h"
#include "ROSTools.h"

class GActiveMarker : public Gadget
{
	public:
	
	GActiveMarker(const char * _name);
	GActiveMarker(string _name);
	void initialize(); 
	
	void update();
	void show(bool value);	
	void applyProperties();
	void getIntersections(vector<Ogre::Vector3> & potentials, vector<int> & frameNos, Ogre::Ray & ray);
	void getGlobalIntersections(vector<Ogre::Vector3> & potentials, vector<int> & frameNos, Ogre::Ray & ray);
	
	virtual ~GActiveMarker();
	
	void purgeShapes();
	void cleanup();
	
	static void markerCallback(const visualization_msgs::Marker& marker);
	static void markerArrayCallback(const visualization_msgs::MarkerArray& markerArray);
	static void scriptCallback(const cri::MarkerScript& markerScript);
	
	int markerId;
	int markerType;
	string markerNamespace;
	string meshResource;
	
	vector<Shape *> shapes;
	
	Ogre::MeshPtr mesh;
	Ogre::ManualObject * manual;
	Ogre::MovableText * text;
	Ogre::Entity * entity;
	Ogre::SceneNode * subGraphicNode;
	Ogre::MaterialPtr material;
	
	// For marking markers as active/inactive
	Ogre::Vector3 minCorner, maxCorner;
	Ogre::ManualObject * boundsRect;
	bool boundsVisible;
	bool boundsChanged;
	void calculateBounds();
	void showBounds(bool state);
	
	// For event scripts
	int previousActiveState;
	virtual void giveMouse(Ogre::Ray & ray, int leftClicked, int rightClicked, int active);
	virtual bool clicked(Ogre::Ray & ray, Ogre::Vector3 cameraPos, double & distance);
	virtual bool clickedInRange(Ogre::Ray & ray);
	
	struct EventScript
	{
		string name;
		string code;
	};
	
	vector<struct EventScript> triggerScripts[6];
};

void markerCallback(const visualization_msgs::Marker& msg);

#endif

