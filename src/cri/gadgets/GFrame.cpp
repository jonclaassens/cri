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

#include "GFrame.h"

string fixedFrame;
tf::TransformListener * tfListener;
tf::TransformBroadcaster * tfBroadcaster;
vector<Frame> availableFrames;
int drawFrameTree;

// For drawing the frame tree
Ogre::SceneNode * treeNode = NULL;
Ogre::ManualObject * treeManual = NULL;
int treeWasVisible = 0;

// The global GFrame scale
double frameScale;

Frame::Frame(char * _name)
{
	name = string(_name);
	gotLast = false;
}

Frame::Frame(string _name)
{
	name = _name;
	gotLast = false;
}

Frame::Frame(const Frame & frame)
{
	valid = frame.valid;
	name = frame.name;
	gotLast = false;
	
	lastTime = frame.lastTime;
	curTime = frame.curTime;
	extrapTransform = frame.extrapTransform;
	lastTransform = frame.lastTransform;
	curTransform = frame.curTransform;
}

Frame::~Frame()
{
}

void Frame::initFrames()
{
	tfListener = new tf::TransformListener();
	tfBroadcaster = new tf::TransformBroadcaster();
}

int Frame::findFrame(string name)
{
	int frameIdx;
	
	for (frameIdx = 0; frameIdx < availableFrames.size(); frameIdx++)
	{
		if (availableFrames[frameIdx].name == name)
		{
			return frameIdx;
		}
	}
	
	return -1;
}

void Frame::updateFrames(CRI * boss)
{
	// Todo: Need to interpolate rotation aswell!!!
	
	bool found;
	vector<string> listFromTF;
	vector<string>::iterator listIt;
	vector<Frame>::iterator frameIt;
	
	tf::StampedTransform transform;
	GFrame * tframe;
	
	tfListener->getFrameStrings(listFromTF);
	
	// Check for the appearance of any new frames...
	for (listIt = listFromTF.begin(); listIt != listFromTF.end(); listIt++)
	{
		found = false;
		for (frameIt = availableFrames.begin(); frameIt != availableFrames.end(); frameIt++)
		{
			if ((*frameIt).name == *listIt)
			{
				found = true;
				break;
			}
		}
		
		if (!found)
		{
			Frame newFrame = Frame(*listIt);
			
			// We assume the frame is valid
			newFrame.valid = true;
			
			availableFrames.push_back(newFrame);
			
			// Add a representative frame
			tframe = new GFrame((char *)(*listIt).c_str());
			boss->addGadget(tframe);
			tframe->applyProperties();
			tframe->show(true);
			tframe->sourceFrame = availableFrames.size() - 1;
		}
	}

	double timeDiff, timeFut;
	ros::Time tTime;
	tf::StampedTransform tTran;
	int framePosition = -1;

	// Update frame locations by extrapolating
	// Todo: second order extrapolation, please
	for (frameIt = availableFrames.begin(); frameIt != availableFrames.end(); frameIt++)
	{
		Frame & tFr = (*frameIt);
		framePosition++;
		
		if (tfListener->canTransform(fixedFrame, tFr.name, ros::Time(0), NULL))
		{
			// Check if the frame has become valid
			if (!tFr.valid)
			{
				tFr.valid = true;
				
				// Add a renewed representative frame
				tframe = new GFrame((char *)tFr.name.c_str());
				boss->addGadget(tframe);
				tframe->applyProperties();
				tframe->show(true);
				tframe->sourceFrame = framePosition;
			}
			
			tfListener->lookupTransform(fixedFrame, tFr.name, ros::Time(0), tTran);
		}
		else
		{
			// TODO: error msg
			
			// The frame is not connected to the fixed frame, or we're extrapolating, either way,
			// the frame is useless.
			tFr.valid = false;
			
			continue;
		}
		
		//cout << "Beer: " << ros::Time::now() << "\n";
		//cout << tTran.stamp_ << "\n";
		
		if (tFr.gotLast == false)
		{
			tFr.curTransform = tTran;
			tFr.curTime = tTime;
			tFr.extrapTransform = tTran;
			tFr.gotLast = true;
		}
		else
		{
			// Do we have a new sample?  If so, update the pair of samples
			if (tTran.stamp_ > tFr.curTime)
			{
				tFr.lastTransform = tFr.curTransform;
				tFr.lastTime = tFr.curTime;
				
				tFr.curTransform = tTran;
				tFr.curTime = tTran.stamp_;
			}
			
			// Calculate the extrapolated position
			timeDiff = (tFr.curTime - tFr.lastTime).toSec();
			timeFut = (ros::Time::now() - tFr.curTime).toSec();
			
			tf::Vector3 & lPos = tFr.lastTransform.getOrigin();
			tf::Quaternion lRot = tFr.lastTransform.getRotation();
			tf::Vector3 & cPos = tFr.curTransform.getOrigin();
			tf::Quaternion cRot = tFr.curTransform.getRotation();
			
			// Todo: No magic numbers
			if (timeDiff > 0.001 && timeFut < 0.5)
			{
				tFr.extrapTransform.setOrigin(tf::Vector3((cPos.x() - lPos.x()) / timeDiff * timeFut + cPos.x(), 
					(cPos.y() - lPos.y()) / timeDiff * timeFut + cPos.y(), 
					(cPos.z() - lPos.z()) / timeDiff * timeFut + cPos.z()));
				tFr.extrapTransform.setRotation(tFr.curTransform.getRotation());
			}
			else
			{
				tFr.extrapTransform.setOrigin(tFr.curTransform.getOrigin());
				tFr.extrapTransform.setRotation(tFr.curTransform.getRotation());
			}
			
		}
		
	}
	
	// If we need to draw the frame tree, do so
	
	if (drawFrameTree)
	{
		tf::StampedTransform endPoint;
		tf::StampedTransform startPoint;
		string startFrameName;
		
		if (treeNode && !treeWasVisible)
		{
			boss->rootSceneNode->addChild(treeNode);
		}
		else if (treeNode == NULL)
		{
			treeNode = boss->rootSceneNode->createChildSceneNode();
		}
		
		if (!treeManual)
		{
			treeManual = boss->scene->createManualObject("TreeManual");
			treeNode->attachObject(treeManual);
			
			treeManual->setDynamic(true);
			treeManual->estimateVertexCount(1000);
			treeManual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
			
		}
		else
		{
			treeManual->beginUpdate(0);
		}
		
		treeManual->colour(Ogre::ColourValue(1, 1, 0, 1));
		
		for (frameIt = availableFrames.begin(); frameIt != availableFrames.end(); frameIt++)
		{
			if ((*frameIt).valid)
			{
				//tfListener->lookupTransform(fixedFrame, tFr.name, ros::Time(0), endPoint);
				tf::Vector3 startPos = (*frameIt).extrapTransform.getOrigin();
				
				tfListener->getParent((*frameIt).name, ros::Time(), startFrameName);
				int fNum = findFrame(startFrameName);
				
				if (fNum >= 0)
				{
					tf::Vector3 endPos = availableFrames[fNum].extrapTransform.getOrigin();
					
					treeManual->position(startPos.x(), startPos.y(), startPos.z());
					treeManual->position(endPos.x(), endPos.y(), endPos.z());
				}
			}
		}
		
		// A dummy to keep things sane, can't have zero length manual object
		treeManual->position(0, 0, 0);
		treeManual->position(0, 0, 0);
		
		treeManual->end();
		
		treeWasVisible = 1;
	}
	else
	{
		if (treeWasVisible)
		{
			boss->rootSceneNode->removeChild(treeNode);
		}
		treeWasVisible = 0;
	}
}

GFrame::GFrame(char * _name) : Gadget(_name)
{
	manual = NULL;
	type = Gadget::GADGET_FRAME;
	visible = false;
	
	xCone = NULL;
	yCone = NULL;
	zCone = NULL;
	
	coneScale = 0.15;
	addProperty(string("ConeScale"), PYINT_DOUBLE, &coneScale);
}

void GFrame::applyProperties()
{
	// If there is no manual object, create one 
	if (manual == NULL)
	{
		manual = scene->createManualObject(name);
		
		manual->setDynamic(true);
		manual->estimateVertexCount(1000);
		manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
	}
	else
	{
		manual->beginUpdate(0);
	}
	
	// z-axis
	// The line
	// Note: Add vertex first, then change the color
	manual->position(0, 0, 0);
	manual->colour(1, 0, 0);
	manual->position(0, 0, 1);
	
	// The arrowhead
	//manual->position(0, 0, 1);
	//manual->position(0, 0.2, 0.8);
	//manual->position(0, 0, 1);
	//manual->position(0, -0.2, 0.8);	
	
	// y-axis
	manual->position(0, 0, 0);
	manual->colour(0, 1, 0);
	manual->position(0, 1, 0);
	
	// The arrowhead
	//manual->position(0, 1, 0);
	//manual->position(0.2, 0.8, 0);
	//manual->position(0, 1, 0);
	//manual->position(-0.2, 0.8, 0);		
	
	// x-axis
	manual->position(0, 0, 0);
	manual->colour(0, 0, 1);
	manual->position(1, 0, 0);
	
	// The arrowhead	
	//manual->position(1, 0, 0);
	//manual->position(0.8, 0.2, 0);
	//manual->position(1, 0, 0);
	//manual->position(0.8, -0.2, 0);	
	
	// End of definition
	manual->end();
	
	// If we haven't already created the arrow head shapes, create them
	if (!xCone)
	{
		xCone = new Shape(Shape::Cone, scene, NULL);
		yCone = new Shape(Shape::Cone, scene, NULL);
		zCone = new Shape(Shape::Cone, scene, NULL);
	}
	
	xCone->setPosition(Ogre::Vector3(1, 0, 0));
	xCone->setOrientation(CreateFrame(Ogre::Vector3(1, 0, 0)));
	xCone->setScale(Ogre::Vector3(coneScale, coneScale * 2, coneScale));
	xCone->setColor(Ogre::ColourValue(0, 0, 1, 1));
		
	yCone->setPosition(Ogre::Vector3(0, 1, 0));
	yCone->setOrientation(CreateFrame(Ogre::Vector3(0, 1, 0)));
	yCone->setScale(Ogre::Vector3(coneScale, coneScale * 2, coneScale));
	yCone->setColor(Ogre::ColourValue(0, 1, 0, 1));
		
	zCone->setPosition(Ogre::Vector3(0, 0, 1));
	zCone->setOrientation(CreateFrame(Ogre::Vector3(0, 0, 1)));
	zCone->setScale(Ogre::Vector3(coneScale, coneScale * 2, coneScale));
	zCone->setColor(Ogre::ColourValue(1, 0, 0, 1));
	
	// Need a main scene node for this gadget
	if (!graphicNode)
	{
		graphicNode = scene->createSceneNode();
		parent->addChild(graphicNode);
	}
	
	// Adjust frame scale based on global property
	graphicNode->setScale(frameScale, frameScale, frameScale);
	
	publishStatus(Gadget::OKAY, "Added frame");
}

void GFrame::show(bool value)
{
	if (value)
	{
		if (!visible)
		{
			// Add the manual object to our separate SceneNode
			graphicNode->attachObject(manual);
			xCone->attachToNode(graphicNode);
			yCone->attachToNode(graphicNode);
			zCone->attachToNode(graphicNode);
			
			visible = true;
		}
	}
	else
	{
		// Remove manual object from the scene
		graphicNode->detachObject(manual);
		xCone->detachFromNode();
		yCone->detachFromNode();
		zCone->detachFromNode();
		
		visible = false;
	}
}

void GFrame::update()
{
	// If, for some reason, the graphic frame gets corrupt, kill this gadget instantiation
	if (sourceFrame < 0)
	{
		killMe = true;
		return;
	}
		
	// If gadget frame has gone invalid, it must die
	if (!availableFrames[sourceFrame].valid)
	{
		killMe = true;
		return;
	}
	
	tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
	tf::Vector3 & pos = tran.getOrigin();
	tf::Quaternion rot = tran.getRotation();
	
	// Move the scene node to move the sketch
	graphicNode->setPosition(pos.x(), pos.y(), pos.z());
	graphicNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
}

GFrame::~GFrame()
{
	// TODO:  Are these being removed often???
	
	// Clean up the graphics if necessary
	if (xCone)
	{
		delete xCone;
		delete yCone;
		delete zCone;
	}
	
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
