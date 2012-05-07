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

#include "GPath.h"

GPath::GPath(const char * _name) : Gadget(_name)
{
	type = Gadget::GADGET_PATH;
	
	msgsRecieved = 0;
	manual = NULL;
	
	//billboardWidth = 0.1;
	//addProperty(string("BillboardWidth"), PYINT_DOUBLE, &billboardWidth);
	
	needsApply = true;
}

void GPath::callback(const nav_msgs::Path msg)
{
	int i;
	
	sourceFrame = Frame::findFrame(msg.header.frame_id);

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
			graphicNode->detachObject(manual);
		}
		
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}

	if (manual == NULL)
	{
		return;
	}

	if (msg.poses.size() > 1)
	{
		// Draw the path
		
		manual->beginUpdate(0);
		
		manual->colour(0, 1, 0);

		for (i = 0; i < msg.poses.size(); i++)
		{
			//geometry_msgs::Pose & pose = msg.poses[i].pose;
			manual->position(msg.poses[i].pose.position.x, 
				msg.poses[i].pose.position.y, 
				msg.poses[i].pose.position.z);
		} 
		
		manual->end();
	}
	else
	{
		publishStatus(Gadget::WARNING, "No data");
	}

	if (!showable && visible)
	{
		graphicNode->attachObject(manual);
		showable = true;
	}

	publishStatus(Gadget::OKAY, "Okay");

	msgsRecieved++;
}

void GPath::subscribe(string _topic)
{
	topic = _topic;
	pathSub = master->n->subscribe<nav_msgs::Path> (_topic.c_str(), 1, &GPath::callback, this);
}

void GPath::update()
{
	// If, for some reason, the graphic frame gets corrupt, kill this gadget instantiation
	if (sourceFrame < 0)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		//killMe = true;
		return;
	}
		
	// If gadget frame has gone invalid, the gadget must die
	if (!availableFrames[sourceFrame].valid)
	{
		if (visible && showable)
		{
			showable = false;
			
			graphicNode->detachObject(manual);
		}
		
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}
	
	tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
	tf::Vector3 & pos = tran.getOrigin();
	tf::Quaternion rot = tran.getRotation();
	
	graphicNode->setPosition(pos.x(), pos.y(), pos.z());
	graphicNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
}

void GPath::show(bool value)
{
	if (manual)
	{
		if (value && !visible)
		{
			graphicNode->attachObject(manual);
			visible = true;
		}
		
		if (!value && visible)
		{
			graphicNode->detachObject(manual);
			visible = false;
		}
	}
}

void GPath::applyProperties()
{
	if (manual == NULL)
	{		
		//Ogre::MaterialManager & materialManager = Ogre::MaterialManager::getSingleton();
		
		// If there is no manual object, create one 
		manual = scene->createManualObject(name);
			
		manual->setDynamic(true);
		manual->estimateVertexCount(1000);
		manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

		if (msgsRecieved == 0)
		{
			 manual->position(0, 0, 0);
			 manual->position(0, 0, 0);
		}
		
		manual->end();
		
		// Add the manual object to a separate SceneNode
		graphicNode = parent->createChildSceneNode();
		graphicNode->attachObject(manual);
	}
}

GPath::~GPath()
{
	// Kill the additional scene node and manual object
	if (manual)
	{
		scene->destroyManualObject(manual);
	}
	
	if (graphicNode)
	{
		scene->destroySceneNode(graphicNode);
	}
}
