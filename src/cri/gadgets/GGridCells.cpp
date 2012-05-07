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

#include "GGridCells.h"

GGridCells::GGridCells(const char * _name) : Gadget(_name)
{
	type = Gadget::GADGET_GRIDCELLS;
	
	msgsRecieved = 0;
	cellsManual = NULL;
	
	//needsApply = true;
}

void GGridCells::callback(const nav_msgs::GridCells msg)
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
			
			graphicNode->detachObject(cellsManual);
		}
		
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}

	if (msg.cells.size() > 0)
	{
		// Is this the first time we're rendering the grid
		if (cellsManual == NULL)
		{
			// Create manual object
			cellsManual = scene->createManualObject(name + "_triangles");

			cellsManual->setDynamic(true);
			cellsManual->estimateVertexCount(1000);
			cellsManual->begin("Surfaces/FlatVertexColour", Ogre::RenderOperation::OT_TRIANGLE_LIST);
		}
		else // If this is not the first time we're rendering the grid cells
		{
			cellsManual->beginUpdate(0);
		}

		// Draw the GridCells gadget

		for (i = 0; i < msg.cells.size(); i++)
		{
			// Center
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2 * 0.9, 
				msg.cells[i].z);	
			cellsManual->colour(0, 1, 0, 1);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
				
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
				
			// Left side
			cellsManual->position(msg.cells[i].x - msg.cell_width/2, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->colour(0, 0.3, 0, 1);	
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
				
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);
				
			// Left side
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->colour(0, 0.3, 0, 1);	
			cellsManual->position(msg.cells[i].x + msg.cell_width/2, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
				
			cellsManual->position(msg.cells[i].x + msg.cell_width/2, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);

			// Top
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);	
			cellsManual->colour(0, 0.3, 0, 1);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
				
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y - msg.cell_height/2, 
				msg.cells[i].z);
				
			// Bottom
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2 * 0.9, 
				msg.cells[i].z);	
			cellsManual->colour(0, 0.3, 0, 1);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
				
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x - msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2, 
				msg.cells[i].z);
			cellsManual->position(msg.cells[i].x + msg.cell_width/2 * 0.9, 
				msg.cells[i].y + msg.cell_height/2 * 0.9, 
				msg.cells[i].z);
		}
		
		cellsManual->end();
		
		if (visible && !showable)
		{
			graphicNode->attachObject(cellsManual);
			showable = true;
		}
		
		publishStatus(Gadget::OKAY, "Okay");
	}
	else
	{
		publishStatus(Gadget::WARNING, "No data");
	}

	msgsRecieved++;
}

void GGridCells::subscribe(string _topic)
{
	topic = _topic;
	gridCellsSub = master->n->subscribe<nav_msgs::GridCells> (_topic.c_str(), 1, &GGridCells::callback, this);
}

void GGridCells::update()
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
			
			graphicNode->detachObject(cellsManual);
		}
		
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}
	
	// Otherwise update graphic motion with its frame
	tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
	tf::Vector3 & pos = tran.getOrigin();
	tf::Quaternion rot = tran.getRotation();
	
	graphicNode->setPosition(pos.x(), pos.y(), pos.z());
	graphicNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
}

void GGridCells::show(bool value)
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
			graphicNode->detachObject(cellsManual);
			visible = false;
		}
	}
	else
	{
		// Show the graphics if there is something available, otherwise set 'showable' false so that
		// the message callback handles attaching the graphics to the appropriate scene node.
		
		if (value)
		{
			if (cellsManual)
			{
				graphicNode->attachObject(cellsManual);
				
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

void GGridCells::applyProperties()
{
	// No properties for the GridCells gadget
}

GGridCells::~GGridCells()
{
	// Kill the additional scene node and manual object
	if (cellsManual)
	{
		scene->destroyManualObject(cellsManual);
	}
	
	if (graphicNode)
	{
		scene->destroySceneNode(graphicNode);
	}
}
