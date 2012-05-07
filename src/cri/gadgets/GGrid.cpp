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

#include "GGrid.h"

GGrid::GGrid(const char * _name) : Gadget(_name)
{
	manual = NULL;
	graphicNode = NULL;
	type = Gadget::GADGET_GRID;
	// Set grid defaults
	
	X_start = -30; X_end = 30; X_step = 1;
	Y_start = -30; Y_end = 30; Y_step = 1;
	Z_start = 0; Z_end = 0; Z_step = 1;
	
	X[0] = 0; X[1] = 0; X[2] = 0;
	Xaxis[0] = 1; Xaxis[1] = 0; Xaxis[2] = 0;
	Yaxis[0] = 0; Yaxis[1] = 1; Yaxis[2] = 0;
	Zaxis[0] = 0; Zaxis[1] = 0; Zaxis[2] = 1;
	
	// Grid properties
	addProperty(string("CellXwidth"), PYINT_DOUBLE, &X_step);
	addProperty(string("CellYwidth"), PYINT_DOUBLE, &Y_step);
	addProperty(string("CellZwidth"), PYINT_DOUBLE, &Z_step);
	
	addProperty(string("GridXMin"), PYINT_DOUBLE, &X_start);
	addProperty(string("GridXMax"), PYINT_DOUBLE, &X_end);
	
	addProperty(string("GridYMin"), PYINT_DOUBLE, &Y_start);
	addProperty(string("GridYMax"), PYINT_DOUBLE, &Y_end);
	
	addProperty(string("GridZMin"), PYINT_DOUBLE, &Z_start);
	addProperty(string("GridZMax"), PYINT_DOUBLE, &Z_end);
	
	
	addProperty(string("XCenter"), PYINT_DOUBLE, &X[0]);
	addProperty(string("YCenter"), PYINT_DOUBLE, &X[1]);
	addProperty(string("ZCenter"), PYINT_DOUBLE, &X[2]);
	
	addProperty(string("XAxisXComponent"), PYINT_DOUBLE, &Xaxis[0]);
	addProperty(string("XAxisYComponent"), PYINT_DOUBLE, &Xaxis[1]);
	addProperty(string("XAxisZComponent"), PYINT_DOUBLE, &Xaxis[2]);
	
	addProperty(string("YAxisXComponent"), PYINT_DOUBLE, &Yaxis[0]);
	addProperty(string("YAxisYComponent"), PYINT_DOUBLE, &Yaxis[1]);
	addProperty(string("YAxisZComponent"), PYINT_DOUBLE, &Yaxis[2]);
	
	addProperty(string("ZAxisXComponent"), PYINT_DOUBLE, &Zaxis[0]);
	addProperty(string("ZAxisYComponent"), PYINT_DOUBLE, &Zaxis[1]);
	addProperty(string("ZAxisZComponent"), PYINT_DOUBLE, &Zaxis[2]);
	
	// General properties
	addProperty(string("Frame"), PYINT_STRING, &frameName);
}

void GGrid::applyProperties()
{
	double x, y, z;
	
	// If there is no manual object, create one 
	if (manual == NULL)
	{
		manual = scene->createManualObject(name + "_manual");
		
		manual->setDynamic(true);
		manual->estimateVertexCount(1000);
		manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
	}
	else
	{
		manual->beginUpdate(0);
	}
	
	// Draw the grid
	Ogre::Vector3 center(X[0], X[1], X[2]);
	Ogre::Vector3 XaxisV(Xaxis[0], Xaxis[1], Xaxis[2]);
	Ogre::Vector3 YaxisV(Yaxis[0], Yaxis[1], Yaxis[2]);
	Ogre::Vector3 ZaxisV(Zaxis[0], Zaxis[1], Zaxis[2]);
	manual->colour(0.7, 0.7, 0.7);

	for (z = Z_start; z <= Z_end; z += Z_step)
	{
		for (x = X_start; x <= X_end; x += X_step)
		{
			for (y = Y_start; y < Y_end; y += Y_step)
			{
				if (x != 0 || z != 0)
				{
					manual->position(center + x * XaxisV + y * YaxisV + z * ZaxisV);
					manual->position(center + x * XaxisV + (y + Y_step) * YaxisV + z * ZaxisV);
				}
					
				if (z != Z_end)
				{
					if (x != 0 || z != 0)
					{
						manual->position(center + x * XaxisV + y * YaxisV + z * ZaxisV);
						manual->position(center + x * XaxisV + y * YaxisV + (z + Z_step) * ZaxisV);
					}
				}
				
				if (x != X_end)
				{
					if (y != 0 || z != 0)
					{
						manual->position(center + x * XaxisV + y * YaxisV + z * ZaxisV);
						manual->position(center + (x + X_step) * XaxisV + y * YaxisV + z * ZaxisV);
					}
				}
				
			}
			if (x != X_end)
			{
				if (y != 0 || z != 0)
				{
					manual->position(center + x * XaxisV + Y_end * YaxisV + z * ZaxisV);
					manual->position(center + (x + X_step) * XaxisV + Y_end * YaxisV + z * ZaxisV);
				}
			}
			
			if (z != Z_end)
			{
				if (x != 0 || y != 0)
				{
					manual->position(center + x * XaxisV + Y_end * YaxisV + z * ZaxisV);
					manual->position(center + x * XaxisV + Y_end * YaxisV + (z + Z_step) * ZaxisV);
				}
			}
		} 
	}

	// Draw origin cross lines
	manual->position(center + X_start * XaxisV);
	manual->colour(1.0, 1.0, 1.0);
	manual->position(center + X_end * XaxisV);
	
	manual->position(center + Y_start * YaxisV);
	manual->position(center + Y_end * YaxisV);
	
	manual->position(center + Z_start * ZaxisV);
	manual->position(center + Z_end * ZaxisV);

	// End of manual definition
	manual->end();

	if (!graphicNode)
	{
		graphicNode = scene->createSceneNode();
		parent->addChild(graphicNode);
	}
	
	if (visible && !showable)
	{
		graphicNode->attachObject(manual);
		showable = true;
	}
}

void GGrid::getIntersections(vector<Ogre::Vector3> & potentials, vector<int> & frameNos, Ogre::Ray & ray)
{
	// TODO : untested!
	
	double z;
	
	Ogre::Vector3 center(X[0], X[1], X[2]);
	Ogre::Vector3 XaxisV(Xaxis[0], Xaxis[1], Xaxis[2]);
	Ogre::Vector3 YaxisV(Yaxis[0], Yaxis[1], Yaxis[2]);
	Ogre::Vector3 ZaxisV(Zaxis[0], Zaxis[1], Zaxis[2]);
	
	Ogre::Vector3 gridPos = graphicNode->_getDerivedPosition();
	Ogre::Quaternion gridOrient = graphicNode->_getDerivedOrientation();
	Ogre::Matrix3 gridR;
	gridOrient.ToRotationMatrix(gridR);
	
	XaxisV = gridR * XaxisV;
	YaxisV = gridR * YaxisV;
	ZaxisV = gridR * ZaxisV;
	
	// For every level check for intersection
	for (z = Z_start; z <= Z_end; z += Z_step)
	{
		// Plane is constructed with the normal as the first parameter
		Ogre::Plane plane(ZaxisV, gridPos + gridR * center + ZaxisV * z);
		
		std::pair<bool, Ogre::Real> tmp = ray.intersects(plane);
		
		Ogre::Vector3 pos = ray.getPoint(tmp.second);
		
		double xpos = (pos - gridPos - gridR * center).dotProduct(XaxisV);
		double ypos = (pos - gridPos - gridR * center).dotProduct(YaxisV);
		
		if (xpos >= X_start && xpos <= X_end && ypos >= Y_start && ypos <= Y_end)
		{
			potentials.push_back(gridR.Inverse() * (pos - gridPos));
			frameNos.push_back(sourceFrame);
		}
	}
}

void GGrid::show(bool value)
{	
	// Make the graphic node visible if it can be
	if (visible)
	{
		if (!value)
		{
			graphicNode->detachObject(manual);
			visible = false;
		}
	}
	else
	{
		// Show the graphics if there is something available, otherwise set 'showable' false so that
		// the message callback handles attaching the graphics to the appropriate scene node.
		
		if (value)
		{
			if (manual)
			{
				graphicNode->attachObject(manual);
				
				visible = true;
				showable = true;
			}
			else
			{
				visible = true;
				showable = false;
			}
		}
	}
}

void GGrid::update()
{
	// Check that we're assigned to the correct frame name, and if not correct it
	if (sourceFrame >= 0)
	{
		if (frameName != availableFrames[sourceFrame].name)
		{
			if (frameName == "")
			{
				sourceFrame = -1;
			}
			else
			{
				sourceFrame = Frame::findFrame(frameName);
			}
		}
	}
	else 
	{
		if (frameName != "")
		{
			sourceFrame = Frame::findFrame(frameName);
		}
	}
	
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

GGrid::~GGrid()
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
