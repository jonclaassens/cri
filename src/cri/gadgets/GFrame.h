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

#ifndef __GFRAME
#define __GFRAME

#include <OgreSubMesh.h>
#include <resource_retriever/retriever.h>
#include <boost/filesystem.hpp>

#include "CRI.h"

// To resolve some compatibility issue between ROS and CEGUI/OGRE headers
#ifdef Success
#undef Success
#endif

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <boost/lexical_cast.hpp>

#include <Procedural.h>
#include "OgreTools2.h"

extern tf::TransformBroadcaster * tfBroadcaster;
extern tf::TransformListener * tfListener;
extern string fixedFrame;

// Parameter for enabling/disabling drawing the frame trees
extern int drawFrameTree;

class Frame
{
	public:
	
	//static void initializeFrames();
	static void initFrames();
	static int findFrame(string name);
	static void updateFrames(CRI * boss);
	
	string name;
	bool gotLast;
	ros::Time lastTime, curTime;
	tf::StampedTransform extrapTransform, lastTransform, curTransform;
	int valid;
	
	Frame(char * _name);
	Frame(string _name);
	Frame(const Frame & frame);
	~Frame();
};

// The global GFrame scale
extern double frameScale;

class GFrame : public Gadget
{
	public:
	
	GFrame(char * _name);
	GFrame(string _name);
	void update();
	void show(bool value);
	void applyProperties();
	
	virtual ~GFrame();
	
	// Arrow heads
	Shape * xCone, * yCone, * zCone;
	double coneScale;
	Ogre::ManualObject * manual;
};



extern tf::TransformListener * tfListener;
extern vector<Frame> availableFrames;

// todo : Fix this 
extern CRI * boss;

#endif
