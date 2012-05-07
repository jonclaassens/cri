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

#include "GQuiver.h"

GQuiver::GQuiver(const char * _name) : Gadget(_name)
{
	type = Gadget::GADGET_QUIVER;
	
	msgsRecieved = 0;
		
	scale = 1;
	addProperty(string("Scale"), PYINT_DOUBLE, &scale);
	
	drawXAxis = true;
	drawYAxis = true;
	drawZAxis = true;
	
	xColour[0] = 255; xColour[1] = 0; xColour[2] = 0;
	yColour[0] = 0; yColour[1] = 255; yColour[2] = 0;
	zColour[0] = 0; zColour[1] = 0; zColour[2] = 255;
	
	//needsApply = true;
}

void GQuiver::removeFirstIn()
{
	if (drawXAxis)
	{
		delete xCones[0];
		xCones.erase(xCones.begin());
		
		delete xCylinders[0];
		xCylinders.erase(xCylinders.begin());
	}
	
	if (drawYAxis)
	{
		delete yCones[0];
		yCones.erase(yCones.begin());
		
		delete yCylinders[0];
		yCylinders.erase(yCylinders.begin());
	}
	
	if (drawZAxis)
	{
		delete zCones[0];
		zCones.erase(zCones.begin());
	
		delete zCylinders[0];
		zCylinders.erase(zCylinders.begin());
	}
	
	graphicNode->removeChild(sceneNodes[0]);
	scene->destroySceneNode(sceneNodes[0]);
	sceneNodes.erase(sceneNodes.begin());
}

void GQuiver::cleanup()
{
	while (sceneNodes.size() > 0)
	{
		removeFirstIn();
	}
}

void GQuiver::addFrameToNode(Ogre::SceneNode * newNode)
{
	if (drawXAxis)
	{
		Shape * xCone = new Shape(Shape::Cone, scene, newNode);
		Shape * xCylinder = new Shape(Shape::RoughCylinder, scene, newNode);
		
		xCone->alignToLineSegment(Ogre::Vector3(0.9, 0, 0),
			Ogre::Vector3(1, 0, 0),
			1);
		xCone->setScale(Ogre::Vector3(0.1, 
			0.2, 
			0.1));
		xCone->setColor(Ogre::ColourValue(xColour[0] / 255.f, xColour[1] / 255.f, xColour[2] / 255.f, 1));
			
		xCylinder->alignToLineSegment(Ogre::Vector3(0, 0, 0),
			Ogre::Vector3(0.9, 0, 0),
			1);
		xCylinder->setScale(Ogre::Vector3(0.01, 
			0.8, 
			0.01));
		xCylinder->setColor(Ogre::ColourValue(xColour[0] / 255.f, xColour[1] / 255.f, xColour[2] / 255.f, 1));
			
		xCones.push_back(xCone);
		xCylinders.push_back(xCylinder);
	}
		
	if (drawYAxis)
	{
		Shape * yCone = new Shape(Shape::Cone, scene, newNode);
		Shape * yCylinder = new Shape(Shape::RoughCylinder, scene, newNode);
		
		yCone->alignToLineSegment(Ogre::Vector3(0, 0.9, 0),
			Ogre::Vector3(0, 1, 0),
			1);
		yCone->setScale(Ogre::Vector3(0.1, 
			0.2, 
			0.1));
		yCone->setColor(Ogre::ColourValue(yColour[0] / 255.f, yColour[1] / 255.f, yColour[2] / 255.f, 1));
			
		yCylinder->alignToLineSegment(Ogre::Vector3(0, 0, 0),
			Ogre::Vector3(0, 0.9, 0),
			1);
		yCylinder->setScale(Ogre::Vector3(0.01, 
			0.8, 
			0.01));
		yCylinder->setColor(Ogre::ColourValue(yColour[0] / 255.f, yColour[1] / 255.f, yColour[2] / 255.f, 1));
			
		yCones.push_back(yCone);
		yCylinders.push_back(yCylinder);
	}
	
	if (drawZAxis)
	{
		Shape * zCone = new Shape(Shape::Cone, scene, newNode);
		Shape * zCylinder = new Shape(Shape::RoughCylinder, scene, newNode);
		
		zCone->alignToLineSegment(Ogre::Vector3(0, 0, 0.9),
			Ogre::Vector3(0, 0, 1),
			1);
		zCone->setScale(Ogre::Vector3(0.1, 
			0.2, 
			0.1));
		zCone->setColor(Ogre::ColourValue(zColour[0] / 255.f, zColour[1] / 255.f, zColour[2] / 255.f, 1));
			
		zCylinder->alignToLineSegment(Ogre::Vector3(0, 0, 0),
			Ogre::Vector3(0, 0, 0.9),
			1);
		zCylinder->setScale(Ogre::Vector3(0.01, 
			0.8, 
			0.01));
		zCylinder->setColor(Ogre::ColourValue(zColour[0] / 255.f, zColour[1] / 255.f, zColour[2] / 255.f, 1));
			
		zCones.push_back(zCone);
		zCylinders.push_back(zCylinder);
	}
}

void GQuiver::odometryCallback(const nav_msgs::Odometry msg)
{
	sourceFrame = Frame::findFrame(msg.header.frame_id);

	if (sourceFrame < 0)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}
	
	if (!availableFrames[sourceFrame].valid)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}

	// Cull axes to keep their number within the limit
	while (sceneNodes.size() > countLimit)
	{
		removeFirstIn();
	}
	
	tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
	tf::Vector3 & pos = tran.getOrigin();
	tf::Quaternion rot = tran.getRotation();
	
	Ogre::Quaternion Qf(rot.w(), rot.x(), rot.y(), rot.z());
	Ogre::Vector3 Xf(pos.x(), pos.y(), pos.z());
	
	Ogre::Quaternion Qe(msg.pose.pose.orientation.w,
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z);
	Ogre::Vector3 Xe(msg.pose.pose.position.x, 
		msg.pose.pose.position.y, 
		msg.pose.pose.position.z);
	
	Ogre::Vector3 X1 = Xf + Qf * Xe;
	Ogre::Quaternion Q1 = Qf * Qe;
	
	if (sceneNodes.size() > 0)
	{
		Ogre::Vector3 X2 = sceneNodes.back()->getPosition();
		Ogre::Quaternion Q2 = sceneNodes.back()->getOrientation();
		
		if ((X1 - X2).normalise() < positionTolerance)
		{
			return;
		}
		
		if (acos((Q2.Inverse() * Q1).w) * 2 < angleTolerance)
		{
			return;
		}
	}
	
	// Create scene node at odo position
	Ogre::SceneNode * newNode = graphicNode->createChildSceneNode();
	newNode->setPosition(X1);
	newNode->setOrientation(Q1);
	newNode->setScale(scale, scale, scale);
	sceneNodes.push_back(newNode);
	
	addFrameToNode(newNode);
	
	publishStatus(Gadget::OKAY, "Okay");

	msgsRecieved++;
}

void GQuiver::poseArrayCallback(const geometry_msgs::PoseArray msg)
{
	sourceFrame = Frame::findFrame(msg.header.frame_id);

	if (sourceFrame < 0)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}
	
	if (!availableFrames[sourceFrame].valid)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}

	// Cull axes to keep their number within the limit
	while (sceneNodes.size() > msg.poses.size())
	{
		removeFirstIn();
	}
	
	while (sceneNodes.size() < msg.poses.size())
	{
		// Create scene node at odo position
		Ogre::SceneNode * newNode = graphicNode->createChildSceneNode();
		sceneNodes.push_back(newNode);
		
		addFrameToNode(newNode);
	}
	
	tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
	tf::Vector3 & pos = tran.getOrigin();
	tf::Quaternion rot = tran.getRotation();
		
	Ogre::Quaternion Qf(rot.w(), rot.x(), rot.y(), rot.z());
	Ogre::Vector3 Xf(pos.x(), pos.y(), pos.z());
	
	for (int i = 0; i < msg.poses.size(); i++)
	{
		Ogre::Quaternion Qe(msg.poses[i].orientation.w,
			msg.poses[i].orientation.x,
			msg.poses[i].orientation.y,
			msg.poses[i].orientation.z);
		Ogre::Vector3 Xe(msg.poses[i].position.x, 
			msg.poses[i].position.y, 
			msg.poses[i].position.z);
		
		Ogre::Vector3 X1 = Xf + Qf * Xe;
		Ogre::Quaternion Q1 = Qf * Qe;
		
		Ogre::SceneNode * node = sceneNodes[i];
		
		node->setPosition(X1);
		node->setOrientation(Q1);
		node->setScale(scale, scale, scale);
		
		if (drawXAxis)
		{
			xCones[i]->setColor(Ogre::ColourValue(xColour[0] / 255.f, xColour[1] / 255.f, xColour[2] / 255.f, 1));
			xCylinders[i]->setColor(Ogre::ColourValue(xColour[0] / 255.f, xColour[1] / 255.f, xColour[2] / 255.f, 1));
		}
		
		if (drawYAxis)
		{
			yCones[i]->setColor(Ogre::ColourValue(yColour[0] / 255.f, yColour[1] / 255.f, yColour[2] / 255.f, 1));
			yCylinders[i]->setColor(Ogre::ColourValue(yColour[0] / 255.f, yColour[1] / 255.f, yColour[2] / 255.f, 1));
		}
		
		if (drawZAxis)
		{
			zCones[i]->setColor(Ogre::ColourValue(zColour[0] / 255.f, zColour[1] / 255.f, zColour[2] / 255.f, 1));
			zCylinders[i]->setColor(Ogre::ColourValue(zColour[0] / 255.f, zColour[1] / 255.f, zColour[2] / 255.f, 1));
		}
			
	}
	
	publishStatus(Gadget::OKAY, "Okay");

	msgsRecieved++;
}

void GQuiver::poseStampedCallback(const geometry_msgs::PoseStamped msg)
{
	sourceFrame = Frame::findFrame(msg.header.frame_id);

	if (sourceFrame < 0)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}
	
	if (!availableFrames[sourceFrame].valid)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}

	Ogre::SceneNode * node;

	// If none exists, create a blank rendered frame
	if (xCones.size() == 0)
	{
		// Create scene node
		node = graphicNode->createChildSceneNode();
		sceneNodes.push_back(node);
		
		addFrameToNode(node);
	}
	else
	{
		node = sceneNodes[0];
	}
	
	tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
	tf::Vector3 & pos = tran.getOrigin();
	tf::Quaternion rot = tran.getRotation();
	
	Ogre::Quaternion Qf(rot.w(), rot.x(), rot.y(), rot.z());
	Ogre::Vector3 Xf(pos.x(), pos.y(), pos.z());
	
	Ogre::Quaternion Qe(msg.pose.orientation.w,
		msg.pose.orientation.x,
		msg.pose.orientation.y,
		msg.pose.orientation.z);
	Ogre::Vector3 Xe(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	
	// Set pose indicator position
	node->setPosition(Xf + Qf * Xe);
	node->setOrientation(Qf * Qe);
	node->setScale(scale, scale, scale);
	
	if (drawXAxis)
	{
		xCones[0]->setColor(Ogre::ColourValue(xColour[0] / 255.f, xColour[1] / 255.f, xColour[2] / 255.f, 1));
		xCylinders[0]->setColor(Ogre::ColourValue(xColour[0] / 255.f, xColour[1] / 255.f, xColour[2] / 255.f, 1));
	}
	
	if (drawYAxis)
	{
		yCones[0]->setColor(Ogre::ColourValue(yColour[0] / 255.f, yColour[1] / 255.f, yColour[2] / 255.f, 1));
		yCylinders[0]->setColor(Ogre::ColourValue(yColour[0] / 255.f, yColour[1] / 255.f, yColour[2] / 255.f, 1));
	}
	
	if (drawZAxis)
	{
		zCones[0]->setColor(Ogre::ColourValue(zColour[0] / 255.f, zColour[1] / 255.f, zColour[2] / 255.f, 1));
		zCylinders[0]->setColor(Ogre::ColourValue(zColour[0] / 255.f, zColour[1] / 255.f, zColour[2] / 255.f, 1));
	}
	
	publishStatus(Gadget::OKAY, "Okay");

	msgsRecieved++;
}

void GQuiver::subscribeOdometry(string _topic)
{
	type = Gadget::GADGET_ODOMETRY;
	
	topic = _topic;
	drawXAxis = true;
	drawYAxis = false;
	drawZAxis = false;
	
	countLimit = 100;
	addProperty(string("CountLimit"), PYINT_INT, &countLimit);
	
	addProperty(string("ArrowColourR"), PYINT_INT, &(xColour[0]));
	addProperty(string("ArrowColourG"), PYINT_INT, &(xColour[1]));
	addProperty(string("ArrowColourB"), PYINT_INT, &(xColour[2]));

	positionTolerance = 0.1;
	angleTolerance = 0.1;
	addProperty(string("PositionTolerance"), PYINT_DOUBLE, &positionTolerance);
	addProperty(string("AngleTolerance"), PYINT_DOUBLE, &angleTolerance);

	sub = master->n->subscribe<nav_msgs::Odometry> (_topic.c_str(), 10, &GQuiver::odometryCallback, this);
}

void GQuiver::subscribePoseStamped(string _topic)
{
	type = Gadget::GADGET_POSESTAMPED;
	
	topic = _topic;
	drawXAxis = true;
	drawYAxis = false;
	drawZAxis = false;
	
	addProperty(string("ArrowColourR"), PYINT_INT, &(xColour[0]));
	addProperty(string("ArrowColourG"), PYINT_INT, &(xColour[1]));
	addProperty(string("ArrowColourB"), PYINT_INT, &(xColour[2]));
	
	sub = master->n->subscribe<geometry_msgs::PoseStamped> (_topic.c_str(), 10, &GQuiver::poseStampedCallback, this);
}

void GQuiver::subscribePoseArray(string _topic)
{
	type = Gadget::GADGET_POSEARRAY;
	
	topic = _topic;
	drawXAxis = true;
	drawYAxis = false;
	drawZAxis = false;
	
	addProperty(string("ArrowColourR"), PYINT_INT, &(xColour[0]));
	addProperty(string("ArrowColourG"), PYINT_INT, &(xColour[1]));
	addProperty(string("ArrowColourB"), PYINT_INT, &(xColour[2]));
	
	sub = master->n->subscribe<geometry_msgs::PoseArray> (_topic.c_str(), 10, &GQuiver::poseArrayCallback, this);
}

void GQuiver::update()
{
	// Kill elements that have no time left to live
	
}

void GQuiver::show(bool value)
{
	// Ensure that we have the ogre scene node which is responsible for the gadget
	if (!graphicNode)
	{
		graphicNode = scene->createSceneNode();
		graphicNode->setPosition(0, 0, 0);
		graphicNode->setOrientation(Ogre::Quaternion::IDENTITY);
	}
	
	// Make the graphic node visible if it can be
	if (visible)
	{
		if (!value)
		{
			parent->removeChild(graphicNode);
			visible = false;
		}
	}
	else
	{
		// Show the graphics if there is something available, otherwise set 'showable' false so that
		// the message callback handles attaching the graphics to the appropriate scene node.
		
		if (value)
		{
			parent->addChild(graphicNode);
			visible = true;
		}
	}
}

void GQuiver::applyProperties()
{
	// No properties for the GridCells gadget
}

GQuiver::~GQuiver()
{
	// Kill the additional scene node and manual object
	cleanup();
	
	if (graphicNode)
	{
		scene->destroySceneNode(graphicNode);
	}
}
