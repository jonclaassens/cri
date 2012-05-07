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

#include "GActiveMarker.h"

CRI * boss;
int textNameNum = 0;

void GActiveMarker::initialize()
{
	type = Gadget::GADGET_MARKER;
	
	text = NULL;
	entity = NULL;
	manual = NULL;
	boundsRect = NULL;
	
	graphicNode = NULL;
	subGraphicNode = NULL;
	
	boundsVisible = false;
	boundsChanged = false;
	
	previousActiveState = 0; 
}

GActiveMarker::GActiveMarker(const char * _name) : Gadget(_name)
{
	initialize();
}

GActiveMarker::GActiveMarker(string _name) : Gadget(_name.c_str())
{
	initialize();
}

void GActiveMarker::update()
{
	switch (markerType)
	{
		case visualization_msgs::Marker::ARROW:
		case visualization_msgs::Marker::CUBE:
		case visualization_msgs::Marker::SPHERE:
		case visualization_msgs::Marker::CYLINDER:
		case visualization_msgs::Marker::LINE_STRIP:
		case visualization_msgs::Marker::LINE_LIST:
		case visualization_msgs::Marker::CUBE_LIST:
		case visualization_msgs::Marker::SPHERE_LIST:
		case visualization_msgs::Marker::POINTS:
		case visualization_msgs::Marker::TEXT_VIEW_FACING:
		case visualization_msgs::Marker::TRIANGLE_LIST:
		case visualization_msgs::Marker::MESH_RESOURCE:
		
			// If the graphic frame is unavailable, kill the graphic
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
	
			graphicNode->setPosition(pos.x(), pos.y(), pos.z());
			graphicNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
			
			// TODO:  This very slow!
			boundsChanged = true;
			break;
			
	}
}

void GActiveMarker::applyProperties()
{
	// Todo!
}

void GActiveMarker::purgeShapes()
{
	vector<Shape *>::iterator shapeIt;
	
	for (shapeIt = shapes.begin(); shapeIt != shapes.end(); shapeIt++)
	{
		delete *shapeIt;
	}
	
	shapes.clear();
}

void GActiveMarker::cleanup()
{
	purgeShapes();
	
	if (subGraphicNode)
	{
		scene->destroySceneNode(subGraphicNode);
		subGraphicNode = NULL;
	}

	if (manual)
	{
		scene->destroyManualObject(manual);
		manual = NULL;
	}

	if (entity)
	{
		scene->destroyEntity(entity);
		entity = NULL;
	}
	
	if (text)
	{
		graphicNode->detachObject(text);
		delete text;
	}
}

void GActiveMarker::markerArrayCallback(const visualization_msgs::MarkerArray& markerArray)
{
	int i;
	
	for (i = 0; i << markerArray.markers.size(); i++)
	{
		markerCallback(markerArray.markers[i]);
	}
}

// TODO:  Don't blend if it is not necessary

void GActiveMarker::markerCallback(const visualization_msgs::Marker& marker)
{
	int frameNo;
	bool found, renew;
	string name;
	GActiveMarker * tAM;
	vector<Gadget *>::iterator gadIt;
	
	switch (marker.action)
	{
		case visualization_msgs::Marker::DELETE:
			{
				// Make an appropriate name based on the namespace and marker ID
				name = marker.ns + boost::lexical_cast<std::string>(marker.id);
				
				found = false;
				for (gadIt = boss->gadgets.begin(); gadIt != boss->gadgets.end(); gadIt++)
				{
					if ((*gadIt)->name == name)
					{
						found = true;
						break;
					}
				}
				
				// No marker, leave without a sound
				if (!found)
				{
					break;
				}
				
				(*gadIt)->killMe = true;
				
				break;
			}
		
		// ROS has ADD and MODIFY have the same value...
		case visualization_msgs::Marker::ADD:
		//case visualization_msgs::Marker::MODIFY:
			
			// Make an appropriate name based on the namespace and marker ID
			name = marker.ns + boost::lexical_cast<std::string>(marker.id);

			// Get the associated frame
			frameNo = Frame::findFrame(marker.header.frame_id);

			// If we can't find the frame, or it is invalid leave
			if (frameNo < 0)
			{
				return;
			}
			if (!availableFrames[frameNo].valid)
			{
				return;
			}

			// Look for an existing Gadget with this name
			found = false;
			for (gadIt = boss->gadgets.begin(); gadIt != boss->gadgets.end(); gadIt++)
			{
				if ((*gadIt)->name == name)
				{
					found = true;
					break;
				}
			}

			if (found)
			{
				// We're modifying an existing marker
				tAM = (GActiveMarker * )*gadIt;
			}
			else // !found
			{
				// Create a new marker
				tAM = new GActiveMarker(name);
				
				// Create the gadget for this marker
				boss->addGadget(tAM);
				
				// Have the active marker create a scene node (graphicNode)
				tAM->show(true);
			}

			// Set the Gadget's frame
			tAM->sourceFrame = frameNo;

			// If the marker's nature changes, free up all its allocated graphic memory
			if (found)
			{
				if (tAM->markerType != marker.type)
				{
					tAM->cleanup();
					
					renew = true;

					tAM->markerType = marker.type;
				}
				else 
				{
					renew = false;
				}
			}
			else 
			{
				renew = true;
				
				tAM->markerType = marker.type;
			}
			
			switch (marker.type)
			{
				case visualization_msgs::Marker::ARROW:
					{
						if (renew)
						{
							Shape * shape;
						
							if (!tAM->subGraphicNode)
							{
								tAM->subGraphicNode = tAM->graphicNode->createChildSceneNode();
							}
						
							shape = new Shape(Shape::Cone, tAM->scene, tAM->subGraphicNode);
							tAM->shapes.push_back(shape);
							
							shape = new Shape(Shape::RoughCylinder, tAM->scene, tAM->subGraphicNode);
							tAM->shapes.push_back(shape);
						}
						
						Ogre::Vector3 startPoint;
						Ogre::Vector3 endPoint;
						
						if (marker.points.size() > 1)
						{
							startPoint[0] = marker.points[0].x;
							startPoint[1] = marker.points[0].y,
							startPoint[2] = marker.points[0].z;
									
							endPoint[0] = marker.points[1].x;
							endPoint[1] = marker.points[1].y;
							endPoint[2]	= marker.points[1].z;
						}
						else
						{
							if (marker.points.size() == 1)
							{
								tAM->publishStatus(Gadget::WARNING, "To few points to specify an arrow direction, 2 needed");
							}
							
							startPoint[0] = 0;
							startPoint[1] = 0,
							startPoint[2] = 0;
									
							endPoint[0] = 1;
							endPoint[1] = 0;
							endPoint[2]	= 0;
						}
							
						tAM->shapes[0]->alignToLineSegment(startPoint + (endPoint - startPoint) * 0.9,
							endPoint,
							1);
						tAM->shapes[0]->setScale(Ogre::Vector3(marker.scale.y, 
							marker.scale.x * 0.2, 
							marker.scale.y));
							
						tAM->shapes[1]->alignToLineSegment(startPoint,
							endPoint,
							1);
						tAM->shapes[1]->setScale(Ogre::Vector3(marker.scale.y * 0.1, 
							marker.scale.x * 0.8, 
							marker.scale.y * 0.1));
						
						// Set shape properties
						
						tAM->shapes[0]->setColor(Ogre::ColourValue(marker.color.r, marker.color.g,
							marker.color.b, marker.color.a));

						tAM->subGraphicNode->setPosition(Ogre::Vector3(marker.pose.position.x, 
							marker.pose.position.y, marker.pose.position.z));
						tAM->subGraphicNode->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
							marker.pose.orientation.x, marker.pose.orientation.y,
							marker.pose.orientation.z));
						
						if (renew)
						{
							tAM->publishStatus(Gadget::OKAY, "Successfully added arrow marker");
						}
						
						break;
					}
					
				case visualization_msgs::Marker::CUBE:
				case visualization_msgs::Marker::SPHERE:
				case visualization_msgs::Marker::CYLINDER:
					{
						if (renew)
						{
							Shape * shape;
												
							switch (marker.type)
							{
								case visualization_msgs::Marker::CUBE:
									shape = new Shape(Shape::Cube, tAM->scene, tAM->graphicNode);
									break;
								case visualization_msgs::Marker::SPHERE:
									shape = new Shape(Shape::Sphere, tAM->scene, tAM->graphicNode);
									break;
								case visualization_msgs::Marker::CYLINDER:
									shape = new Shape(Shape::Cylinder, tAM->scene, tAM->graphicNode);
									break;
							}
							
							tAM->shapes.push_back(shape);
						}
						
						// Set shape properties
						tAM->shapes[0]->setPosition(Ogre::Vector3(marker.pose.position.x, 
							marker.pose.position.y, marker.pose.position.z));
						tAM->shapes[0]->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
							marker.pose.orientation.x, marker.pose.orientation.y,
							marker.pose.orientation.z));
						tAM->shapes[0]->setScale(Ogre::Vector3(marker.scale.x, marker.scale.y,
							marker.scale.z));
						tAM->shapes[0]->setColor(Ogre::ColourValue(marker.color.r, marker.color.g,
							marker.color.b, marker.color.a));
						
						if (renew)
						{	
							switch (marker.type)
							{
								case visualization_msgs::Marker::CUBE:
									tAM->publishStatus(Gadget::OKAY, "Successfully added cube marker");
									break;
								case visualization_msgs::Marker::SPHERE:
									tAM->publishStatus(Gadget::OKAY, "Successfully added sphere marker");
									break;
								case visualization_msgs::Marker::CYLINDER:
									tAM->publishStatus(Gadget::OKAY, "Successfully added cylinder marker");
									break;
							}
						}
						break;
					}
				
				case visualization_msgs::Marker::LINE_STRIP:
					{
						unsigned int N;
						unsigned int i;
						
						if (!tAM->subGraphicNode)
						{
							tAM->subGraphicNode = tAM->graphicNode->createChildSceneNode();
						}
						
						// Line count
						N = marker.points.size() - 1;
						
						if (N < 1)
						{
							tAM->publishStatus(Gadget::ERROR, "Insufficient data to draw lines");
							return;
						}
						
						// If the number of preallocated shapes is too many, cull a few
						if (!renew && tAM->shapes.size() > N)
						{
							while (tAM->shapes.size() > N)
							{
								delete tAM->shapes[0];
								tAM->shapes.erase(tAM->shapes.begin());
							}
						}
						
						// Otherwise, if we have too few shapes preallocated, add a few
						if (tAM->shapes.size() < N)
						{
							while (tAM->shapes.size() < N)
							{
								Shape * shape = new Shape(Shape::RoughCylinder, tAM->scene, tAM->subGraphicNode);
								tAM->shapes.push_back(shape);
							}
						}
						
						// Set shapes' properties
						for (i = 0; i < N; i++)
						{
							tAM->shapes[i]->alignToLineSegment(Ogre::Vector3(marker.points[i].x, 
									marker.points[i].y,
									marker.points[i].z),
								Ogre::Vector3(marker.points[i + 1].x, 
									marker.points[i + 1].y,
									marker.points[i + 1].z),
								marker.scale.x);
						}
						
						// Set the shapes colors based on whether we're getting a list of colors
						// or just a single color...
						if (marker.colors.size() != marker.points.size())
						{
							for (i = 0; i < tAM->shapes.size(); i++)
							{
								tAM->shapes[i]->setColor(Ogre::ColourValue(marker.color.r, marker.color.g,
									marker.color.b, marker.color.a));
							}
						}
						else
						{
							for (i = 0; i < tAM->shapes.size(); i++)
							{
								tAM->shapes[i]->setColor(Ogre::ColourValue(marker.colors[i].r, 
									marker.colors[i].g,
									marker.colors[i].b, 
									marker.colors[i].a));
							}
						}
						
						tAM->subGraphicNode->setPosition(Ogre::Vector3(marker.pose.position.x, 
							marker.pose.position.y, marker.pose.position.z));
						tAM->subGraphicNode->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
							marker.pose.orientation.x, marker.pose.orientation.y,
							marker.pose.orientation.z));
						
						if (renew)
						{
							tAM->publishStatus(Gadget::OKAY, "Successfully added line strip marker");
						}
						
						break;
					}
					
				case visualization_msgs::Marker::LINE_LIST:
					{
						unsigned int N;
						unsigned int i;
						
						if (!tAM->subGraphicNode)
						{
							tAM->subGraphicNode = tAM->graphicNode->createChildSceneNode();
						}
						
						if (marker.points.size() % 2 != 0)
						{
							tAM->publishStatus(Gadget::ERROR, "Need 2xN points to draw N lines");
							return;
						}
						
						// Line count
						N = marker.points.size() / 2;
						
						// If the number of preallocated shapes is too many, cull a few
						if (!renew && tAM->shapes.size() > N)
						{
							while (tAM->shapes.size() > N)
							{
								delete tAM->shapes[0];
								tAM->shapes.erase(tAM->shapes.begin());
							}
						}
						
						// Otherwise, if we have too few shapes preallocated, add a few
						if (tAM->shapes.size() < N)
						{
							while (tAM->shapes.size() < N)
							{
								Shape * shape = new Shape(Shape::RoughCylinder, tAM->scene, tAM->subGraphicNode);
								tAM->shapes.push_back(shape);
							}
						}
						
						
						// Set shapes' properties
						for (i = 0; i < N; i++)
						{
							tAM->shapes[i]->alignToLineSegment(Ogre::Vector3(marker.points[i * 2].x, 
									marker.points[i * 2].y,
									marker.points[i * 2].z),
								Ogre::Vector3(marker.points[i * 2 + 1].x, 
									marker.points[i * 2 + 1].y,
									marker.points[i * 2 + 1].z),
								marker.scale.x);
						}
						
						// Set the shapes colors based on whether we're getting a list of colors
						// or just a single color...
						if (marker.colors.size() != marker.points.size())
						{
							for (i = 0; i < tAM->shapes.size(); i++)
							{
								tAM->shapes[i]->setColor(Ogre::ColourValue(marker.color.r, marker.color.g,
									marker.color.b, marker.color.a));
							}
						}
						else
						{
							for (i = 0; i < tAM->shapes.size(); i++)
							{
								tAM->shapes[i]->setColor(Ogre::ColourValue(marker.colors[i * 2].r, 
									marker.colors[i * 2].g,
									marker.colors[i * 2].b, 
									marker.colors[i * 2].a));
							}
						}
						
						tAM->subGraphicNode->setPosition(Ogre::Vector3(marker.pose.position.x, 
							marker.pose.position.y, marker.pose.position.z));
						tAM->subGraphicNode->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
							marker.pose.orientation.x, marker.pose.orientation.y,
							marker.pose.orientation.z));	
	
						if (renew)
						{
							tAM->publishStatus(Gadget::OKAY, "Successfully added line list marker");
						}
						
						break;
					}
				
				case visualization_msgs::Marker::SPHERE_LIST:
				case visualization_msgs::Marker::CUBE_LIST:
					{
						int shapeType;
						unsigned int i;
						
						if (!tAM->subGraphicNode)
						{
							tAM->subGraphicNode = tAM->graphicNode->createChildSceneNode();
						}
						
						switch (marker.type)
						{
							case visualization_msgs::Marker::SPHERE_LIST:
								shapeType = Shape::Sphere;
								break;
								
							case visualization_msgs::Marker::CUBE_LIST:
								shapeType = Shape::Cube;
								break;
						}
						
						// If the number of preallocated shapes is too many, cull a few
						if (!renew && tAM->shapes.size() > marker.points.size())
						{
							while (tAM->shapes.size() > marker.points.size())
							{
								delete tAM->shapes[0];
								tAM->shapes.erase(tAM->shapes.begin());
							}
						}
						
						// Otherwise, if we have too few shapes preallocated, add a few
						if (tAM->shapes.size() < marker.points.size())
						{
							while (tAM->shapes.size() < marker.points.size())
							{
								Shape * shape = new Shape(shapeType, tAM->scene, tAM->subGraphicNode);
								tAM->shapes.push_back(shape);
							}
						}
						
						// Set shapes' properties
						for (i = 0; i < marker.points.size(); i++)
						{
							tAM->shapes[i]->setPosition(Ogre::Vector3(marker.points[i].x, 
								marker.points[i].y, marker.points[i].z));
								
							tAM->shapes[i]->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
								marker.pose.orientation.x, marker.pose.orientation.y,
								marker.pose.orientation.z));
							
							tAM->shapes[i]->setScale(Ogre::Vector3(marker.scale.x, marker.scale.y,
								marker.scale.z));
						}
						
						// Set the shapes colors based on whether we're getting a list of colors
						// or just a single color...
						if (marker.colors.size() != marker.points.size())
						{
							for (i = 0; i < tAM->shapes.size(); i++)
							{
								tAM->shapes[i]->setColor(Ogre::ColourValue(marker.color.r, marker.color.g,
									marker.color.b, marker.color.a));
							}
						}
						else
						{
							for (i = 0; i < tAM->shapes.size(); i++)
							{
								tAM->shapes[i]->setColor(Ogre::ColourValue(marker.colors[i].r, marker.colors[i].g,
									marker.colors[i].b, marker.colors[i].a));
							}
						}
						
						tAM->subGraphicNode->setPosition(Ogre::Vector3(marker.pose.position.x, 
							marker.pose.position.y, marker.pose.position.z));
						tAM->subGraphicNode->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
							marker.pose.orientation.x, marker.pose.orientation.y,
							marker.pose.orientation.z));
							
						if (renew)
						{
							switch (marker.type)
							{
								case visualization_msgs::Marker::CUBE_LIST:
									tAM->publishStatus(Gadget::OKAY, "Successfully added cube list marker");
									break;
								case visualization_msgs::Marker::SPHERE_LIST:
									tAM->publishStatus(Gadget::OKAY, "Successfully added sphere list marker");
									break;
							}
						}
					}
					
					break;
					
				case visualization_msgs::Marker::POINTS:
					{
						
						unsigned int i;
						unsigned int N;
						
						if (renew)
						{
							tAM->subGraphicNode = tAM->graphicNode->createChildSceneNode();
							
							if (marker.points.size() > 0)
							{
								string materialName = name + "PointMat";

								tAM->material = Ogre::MaterialManager::getSingleton().create(materialName.c_str(), 
									Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
								tAM->material->setPointSize(marker.scale.x);
								Ogre::Technique* pT = tAM->material->getTechnique( 0 );
									if( NULL == pT ) pT = tAM->material->createTechnique();
								Ogre::Pass* pP = pT->getPass( 0 );
								if( NULL == pP ) pP = pT->createPass();

								// set parameters
								pP->setVertexColourTracking(Ogre::TVC_AMBIENT);
								pP->setLightingEnabled(false);
								//pP->setDepthCheckEnabled(true);
								//pP->setDepthWriteEnabled(false);
								//pP->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
							
								tAM->manual = tAM->scene->createManualObject(tAM->name + "_manual");
								
								tAM->subGraphicNode->attachObject(tAM->manual);
								
								tAM->manual->setDynamic(true);
								tAM->manual->estimateVertexCount(1000);
								tAM->manual->begin(materialName.c_str(), Ogre::RenderOperation::OT_POINT_LIST);
							}
							else
							{
								tAM->publishStatus(Gadget::WARNING, "No data in message");
							}
						}
						else
						{
							if (marker.points.size() > 0)
							{
								tAM->material->setPointSize(marker.scale.x);
								tAM->manual->beginUpdate(0);
							}
							else
							{
								tAM->publishStatus(Gadget::WARNING, "No data in message");
							}
						}
						
						N = marker.points.size();
						
						// Setting each point's color independently or not?
						if (marker.colors.size() == marker.points.size())
						{
							for (i = 0; i < N; i++)
							{
								tAM->manual->position(marker.points[i].x,
									marker.points[i].y,
									marker.points[i].z);
								tAM->manual->colour(Ogre::ColourValue(marker.colors[i].r,
									marker.colors[i].g,
									marker.colors[i].b,
									marker.colors[i].a));
							}
						}
						else
						{
							tAM->manual->colour(Ogre::ColourValue(marker.color.r,
									marker.color.g,
									marker.color.b,
									marker.color.a));
									
							for (i = 0; i < N; i++)
							{
								tAM->manual->position(marker.points[i].x,
									marker.points[i].y,
									marker.points[i].z);
							}
						}
						
						tAM->manual->end();
						
						//tAM->subGraphicNode->setScale(Ogre::Vector3(marker.scale.x, marker.scale.y,
						//	marker.scale.z));
						tAM->subGraphicNode->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
							marker.pose.orientation.x, marker.pose.orientation.y,
							marker.pose.orientation.z));
						tAM->subGraphicNode->setPosition(Ogre::Vector3(marker.pose.position.x, 
							marker.pose.position.y, marker.pose.position.z));
					}
					
					if (renew)
					{
						tAM->publishStatus(Gadget::OKAY, "Successfully added points marker");
					}
					
					break;
						
				case visualization_msgs::Marker::TEXT_VIEW_FACING:
					{
						// If we must abandon all of our preallocated data
						if (renew)
						{
							tAM->text = new Ogre::MovableText("TextLabel" + boost::lexical_cast<std::string>(textNameNum),
								marker.text);
							
							// Create host scene nodes
							tAM->subGraphicNode = tAM->graphicNode->createChildSceneNode();
							tAM->subGraphicNode->attachObject(tAM->text);
						}
						
						// Update movable text properties
						tAM->text->setCaption(marker.text);
						tAM->text->setColor(Ogre::ColourValue(marker.color.r, marker.color.g,
									marker.color.b, marker.color.a));
						tAM->subGraphicNode->setPosition(Ogre::Vector3(marker.pose.position.x, 
							marker.pose.position.y, marker.pose.position.z));
						tAM->text->setCharacterHeight(marker.scale.z);
					
						if (renew)
						{
							tAM->publishStatus(Gadget::OKAY, "Successfully added view facing text marker");
						}
					
						break;
					}
				
				case visualization_msgs::Marker::TRIANGLE_LIST:
					{
						unsigned int i;
						
						if (!tAM->subGraphicNode)
						{
							tAM->subGraphicNode = tAM->graphicNode->createChildSceneNode();
						}
						
						// Make sure we 3xN points to make triangles
						if (marker.points.size() % 3 == 0)
						{
							if (marker.points.size() > 0)
							{
								if (renew)
								{			
									tAM->manual = tAM->scene->createManualObject(name + "_manual");
		
									tAM->subGraphicNode->attachObject(tAM->manual);
		
									tAM->manual->setDynamic(true);
									tAM->manual->estimateVertexCount(1000);
									tAM->manual->begin("Surfaces/FlatVertexColour", Ogre::RenderOperation::OT_TRIANGLE_LIST);
								}
								else 
								{
									tAM->manual->beginUpdate(0);
								}
							}
							else 
							{
								tAM->publishStatus(Gadget::WARNING, "No point data");
								return;
							}
						}
						else
						{
							tAM->publishStatus(Gadget::ERROR, "Need 3xN points to draw N triangles");
							return;
						}
	 
						// Handle the cases of colors being populated correctly and not, separately
						if (marker.colors.size() == marker.points.size())
						{
							for (i = 0; i < marker.points.size(); i++)
							{
								tAM->manual->position(marker.points[i].x, 
									marker.points[i].y,
									marker.points[i].z);	
								tAM->manual->colour(Ogre::ColourValue(marker.colors[i].r,
									marker.colors[i].g,
									marker.colors[i].b,
									marker.colors[i].a));
							}
						}
						else
						{
							tAM->manual->colour(Ogre::ColourValue(marker.color.r,
								marker.color.g,
								marker.color.b,
								marker.color.a));
							for (i = 0; i < marker.points.size(); i++)
							{
								tAM->manual->position(marker.points[i].x, 
									marker.points[i].y,
									marker.points[i].z);	
							}
						}
	 
						tAM->manual->end();
						
						tAM->subGraphicNode->setScale(Ogre::Vector3(marker.scale.x, marker.scale.y,
							marker.scale.z));
						tAM->subGraphicNode->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
							marker.pose.orientation.x, marker.pose.orientation.y,
							marker.pose.orientation.z));
						tAM->subGraphicNode->setPosition(Ogre::Vector3(marker.pose.position.x, 
							marker.pose.position.y, marker.pose.position.z));
					
						if (renew)
						{
							tAM->publishStatus(Gadget::OKAY, "Successfully added triangle list marker");
						}
					
						break;
					}
				
				case visualization_msgs::Marker::MESH_RESOURCE:
					// If something has changed free up old resources
					
					if (!renew && (tAM->meshResource != marker.mesh_resource))
					{
						if (tAM->entity)
						{
							// TODO!! :  Need to destroy scene nodes
							
							tAM->scene->destroyEntity(tAM->entity);
							tAM->entity = NULL;
						}
						renew = true;
					}
					
					if (renew)
					{	
						// Load the mesh
						int gotMaterial;
						tAM->mesh = loadMesh(marker.mesh_resource, gotMaterial);
										
						if (tAM->mesh.isNull())
						{
							tAM->publishStatus(Gadget::ERROR, "Failed to load mesh " + marker.mesh_resource);
							return;
						}
						
						// Create Ogre entity to encapsulate it
						tAM->entity = tAM->scene->createEntity((name + "_Ent").c_str(), marker.mesh_resource);
						
						if (!gotMaterial)
						{
							Ogre::MaterialPtr templateMat = Ogre::MaterialManager::getSingleton().getByName("Template/WhiteNoBlend");
							tAM->entity->setMaterial(templateMat);
							
							tAM->publishStatus(Gadget::WARNING, "No material associated with mesh " + marker.mesh_resource);
						}
	
						tAM->meshResource = marker.mesh_resource;
						
						if (!tAM->subGraphicNode)
						{
							tAM->subGraphicNode = tAM->graphicNode->createChildSceneNode();
						}
						
						tAM->subGraphicNode->attachObject(tAM->entity);
					}
					
					// Update scale and position of the mesh
					
					tAM->subGraphicNode->setScale(Ogre::Vector3(marker.scale.x, marker.scale.y,
						marker.scale.z));
					tAM->subGraphicNode->setOrientation(Ogre::Quaternion(marker.pose.orientation.w, 
						marker.pose.orientation.x, marker.pose.orientation.y,
						marker.pose.orientation.z));
					tAM->subGraphicNode->setPosition(Ogre::Vector3(marker.pose.position.x, 
						marker.pose.position.y, marker.pose.position.z));
						
					// Update to ensure the graphic is placed at the correct location
					tAM->update();	
						
					if (renew)
					{
						tAM->publishStatus(Gadget::OKAY, "Successfully added mesh marker");
					}
					
					break;
					
			}
			
			if (tAM->boundsVisible)
			{
				tAM->calculateBounds();
				tAM->boundsChanged = true;
			}
			
			break;
	}	
}



void GActiveMarker::show(bool value)
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

GActiveMarker::~GActiveMarker()
{
	cleanup();
	
	if (graphicNode)
	{
		scene->destroySceneNode(graphicNode);
	}
	
	if (boundsVisible)
	{
		parent->detachObject(boundsRect);
	}
	
	if (boundsRect)
	{
		scene->destroyManualObject(boundsRect);
	}
}
