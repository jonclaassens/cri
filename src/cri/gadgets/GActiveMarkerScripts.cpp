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

using namespace std;

void GActiveMarker::scriptCallback(const cri::MarkerScript& markerScript)
{
	bool found;
	string name;
	GActiveMarker * tAM;
	vector<Gadget *>::iterator gadIt;
	
	// If the id is 0 and the namespace is "", run the code
	if (markerScript.id == -1) // && markerScript.ns.size() == 0)
	{
		if (markerScript.filename.size())
		{
			string path;
			path = parseROSFilename(markerScript.filename);
			
			if (path.size() == 0)
			{
				boss->publishStatus(Gadget::ERROR, "Script executer: Cannot determine script path (given " + markerScript.filename + ")");
				return;
			}
			
			if (boss->pyQtMarkerExec->execute(path.c_str()) != 0)
			{
				boss->publishStatus(Gadget::ERROR, "Script executer: Could not execute script file " + path);
			}	
		}
		else if (markerScript.code.size())
		{
			boss->pyQtMarkerExec->executeString(markerScript.code.c_str());
		}
		else
		{
			boss->publishStatus(Gadget::ERROR, "Script executer: No script file name or code given");
		}

		return;
	}
	
	// Make an appropriate name based on the namespace and marker ID
	name = markerScript.ns + boost::lexical_cast<std::string>(markerScript.id);
				
	found = false;
	for (gadIt = boss->gadgets.begin(); gadIt != boss->gadgets.end(); gadIt++)
	{
		if ((*gadIt)->name == name)
		{
			found = true;
			break;
		}
	}
				
	if (!found)
	{
		boss->publishStatus(Gadget::ERROR, "Marker scripting: Marker " + name + " not found");
		
		// TODO: Need to publish some kind of failure here!!!
		return;
	}
	
	tAM = (GActiveMarker *)*gadIt;
	
	// Message checking
	if (markerScript.trigger_event_type > 5)
	{
		boss->publishStatus(Gadget::ERROR, "Marker scripting: Invalid event code");
		
		return;
	}
	
	// Find the script if it has already been added
	vector<struct EventScript> & eventVector = tAM->triggerScripts[markerScript.trigger_event_type];
	vector<struct EventScript>::iterator scriptIt;
	
	found = false;
	for (scriptIt = eventVector.begin(); scriptIt != eventVector.end(); scriptIt++)
	{
		if ((*scriptIt).name == markerScript.script_name)
		{
			found = true;
			break;
		}
	}
	
	switch (markerScript.action)
	{
		case cri::MarkerScript::ADD_OR_MODIFY:
			{
				string code;
				
				if (markerScript.filename.size())
				{
					string path;
					path = parseROSFilename(markerScript.filename);
					
					if (path.size() == 0)
					{
						boss->publishStatus(Gadget::ERROR, "Marker scripting: For marker " + 
							name + ", cannot determine script path (given " + 
							markerScript.filename + ")");
						return;
					}
					
					code = getTextFromFile(path);
					
					if (code.size() == 0)
					{
						boss->publishStatus(Gadget::ERROR, "Marker scripting: For marker " +
							name + ", cannot open script file " + path);
						return;
					}			
				}
				else if (markerScript.code.size())
				{
					code = markerScript.code;
				}
				else
				{
					boss->publishStatus(Gadget::ERROR, "Marker scripting: For marker " +
							name + ", no script file name or code given\n");
				}
				
				if (found)
				{
					(*scriptIt).code = code;
				}
				else
				{
					struct EventScript tempScript;
					
					tempScript.name = markerScript.script_name;
					tempScript.code = code;
					
					eventVector.push_back(tempScript);
				}
				
				tAM->publishStatus(Gadget::OKAY, "Script added");
				
				break;
			}
		case cri::MarkerScript::DELETE:
			{
				if (found)
				{
					eventVector.erase(scriptIt);
					
					tAM->publishStatus(Gadget::OKAY, "Script removed");
				}
				else
				{
					boss->publishStatus(Gadget::ERROR, "Marker scripting: For marker " +
						name + ", no script with name " + markerScript.script_name + " to delete");
				}
				
				break;
			}
	}
}

void GActiveMarker::giveMouse(Ogre::Ray & ray, int leftClicked, int rightClicked, int active)
{
	vector<Ogre::Vector3> potentials;
	
	if (active)
	{
		showBounds(true);
		
		// Execute left click scripts
		if (leftClicked)
		{
			vector<struct EventScript> & eventVector = triggerScripts[cri::MarkerScript::LEFT_CLICK];
			vector<struct EventScript>::iterator scriptIt;
		
			for (scriptIt = eventVector.begin(); scriptIt != eventVector.end(); scriptIt++)
			{
				master->pyQtMarkerExec->executeString((*scriptIt).code.c_str());
			}
		}
		
		// Execute activation scripts
		if (!previousActiveState)
		{
			vector<struct EventScript> & eventVector = triggerScripts[cri::MarkerScript::BECOME_ACTIVE];
			vector<struct EventScript>::iterator scriptIt;
		
			for (scriptIt = eventVector.begin(); scriptIt != eventVector.end(); scriptIt++)
			{
				master->pyQtMarkerExec->executeString((*scriptIt).code.c_str());
			}
			
			previousActiveState = 1;
		}
	}
	else
	{
		// Execute deactivation scripts
		if (previousActiveState)
		{
			vector<struct EventScript> & eventVector = triggerScripts[cri::MarkerScript::BECOME_INACTIVE];
			vector<struct EventScript>::iterator scriptIt;
		
			for (scriptIt = eventVector.begin(); scriptIt != eventVector.end(); scriptIt++)
			{
				master->pyQtMarkerExec->executeString((*scriptIt).code.c_str());
			}
			
			previousActiveState = 0;
		}
		
		showBounds(false);
	}
}

void GActiveMarker::getIntersections(vector<Ogre::Vector3> & potentials, vector<int> & frameNos, Ogre::Ray & ray)
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
			{
				vector<Shape *>::iterator shapeIt;
				
				// Get the mesh's present location
				tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
				tf::Vector3 & pos = tran.getOrigin();
				tf::Quaternion rot = tran.getRotation();
				Ogre::Quaternion R(rot.w(), rot.x(), rot.y(), rot.z());
				Ogre::Quaternion Rinv = R.Inverse();
				
				for (shapeIt = shapes.begin(); shapeIt != shapes.end(); shapeIt++)
				{ 
					int count = (*shapeIt)->getIntersections(potentials, ray);
					
					for (int i = 0; i < count; i++)
					{
						potentials[potentials.size() - count + i] =
							Rinv * (potentials[potentials.size() - count + i] - Ogre::Vector3(pos[0], pos[1], pos[2]));
						frameNos.push_back(sourceFrame);
					}
				}
				break;
			}
		case visualization_msgs::Marker::MESH_RESOURCE:
			{
				size_t vertex_count,index_count;
				Ogre::Vector3 * vertices;
				unsigned long * indices;
				unsigned int i;
				
				// Get the mesh information
				getMeshInformation(entity->getMesh(), vertex_count, vertices, index_count, indices, 
					subGraphicNode->_getDerivedPosition(), 
					subGraphicNode->_getDerivedOrientation(), 
					subGraphicNode->_getDerivedScale());

				// Get the mesh's present location
				tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
				tf::Vector3 & pos = tran.getOrigin();
				tf::Quaternion rot = tran.getRotation();
				Ogre::Quaternion R(rot.w(), rot.x(), rot.y(), rot.z());
				Ogre::Quaternion Rinv = R.Inverse();

				// Loop through every triangle in the mesh
				for(i = 0; i < index_count; i += 3)
				{
					//vertices that make up the triangle
					Ogre::Vector3 A = vertices[indices[i]];
					Ogre::Vector3 B = vertices[indices[i+1]];
					Ogre::Vector3 C = vertices[indices[i+2]];

					pair<bool, float> tmp = Ogre::Math::intersects(ray, A, B, C, true, false);

					// We've found an intersection, return this result through the vector
					if (tmp.first)
					{
						// Determine exact intersection position by treating the triangle as a 
						// plane and finding the intersection point with the ray
						Ogre::Plane plane(A, B, C);
						std::pair< bool, Ogre::Real > tmp2 = Ogre::Math::intersects(ray, plane);
						Ogre::Vector3 point = ray.getOrigin() + ray.getDirection() * tmp2.second;
						
						// Send back point adjusted for frame position
						potentials.push_back(Rinv * (point - Ogre::Vector3(pos[0], pos[1], pos[2])));
						frameNos.push_back(sourceFrame);
					}
				}
				
				delete [] vertices;
				delete [] indices;
				
				break;
			}
	}
}

void GActiveMarker::getGlobalIntersections(vector<Ogre::Vector3> & potentials, vector<int> & frameNos, Ogre::Ray & ray)
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
			{
				vector<Shape *>::iterator shapeIt;
				
				for (shapeIt = shapes.begin(); shapeIt != shapes.end(); shapeIt++)
				{ 
					int count = (*shapeIt)->getIntersections(potentials, ray);
					
					for (int i = 0; i < count; i++)
					{
						frameNos.push_back(sourceFrame);
					}
				}
				break;
			}
		case visualization_msgs::Marker::MESH_RESOURCE:
			{
				size_t vertex_count,index_count;
				Ogre::Vector3 * vertices;
				unsigned long * indices;
				unsigned int i;
				
				// Get the mesh information
				getMeshInformation(entity->getMesh(), vertex_count, vertices, index_count, indices, 
					subGraphicNode->_getDerivedPosition(), 
					subGraphicNode->_getDerivedOrientation(), 
					subGraphicNode->_getDerivedScale());

				// Loop through every triangle in the mesh
				for(i = 0; i < index_count; i += 3)
				{
					//vertices that make up the triangle
					Ogre::Vector3 A = vertices[indices[i]];
					Ogre::Vector3 B = vertices[indices[i+1]];
					Ogre::Vector3 C = vertices[indices[i+2]];

					pair<bool, float> tmp = Ogre::Math::intersects(ray, A, B, C, true, false);

					// We've found an intersection, return this result through the 'potentials' vector
					if (tmp.first)
					{
						// Determine exact intersection position by treating the triangle as a 
						// plane and finding the intersection point with the ray
						Ogre::Plane plane(A, B, C);
						std::pair< bool, Ogre::Real > tmp2 = Ogre::Math::intersects(ray, plane);
						Ogre::Vector3 pos = ray.getOrigin() + ray.getDirection() * tmp2.second;
						
						potentials.push_back(pos);
						frameNos.push_back(sourceFrame);
					}
				}
				
				delete [] vertices;
				delete [] indices;
				
				break;
			}
	}
}

bool GActiveMarker::clicked(Ogre::Ray & ray, Ogre::Vector3 cameraPos, double & distance)
{
	vector<Ogre::Vector3> potentials;
	vector<int> dummy;
	
	getGlobalIntersections(potentials, dummy, ray);
	
	if (potentials.size() == 0)
	{
		return false;
	}
	
	vector<Ogre::Vector3>::iterator potIt;
	double tempDistance;
	
	potIt = potentials.begin();
	distance = (*potIt - cameraPos).normalise();
	for (; potIt != potentials.end(); potIt++)
	{
		tempDistance = (*potIt - cameraPos).normalise();
		if (tempDistance < distance)
		{
			distance = tempDistance;
		}
	}
	
	return true;
}

bool GActiveMarker::clickedInRange(Ogre::Ray & ray)
{
	vector<Ogre::Vector3> potentials;
	vector<int> dummy;
	
	getGlobalIntersections(potentials, dummy, ray);
	
	return potentials.size() > 0;
}

void GActiveMarker::calculateBounds()
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
			{
				Ogre::Vector3 tempMinCorner, tempMaxCorner;
				vector<Shape *>::iterator shapeIt;
				int first = 1;
				
				for (shapeIt = shapes.begin(); shapeIt != shapes.end(); shapeIt++)
				{
					if ((*shapeIt)->getExtents(tempMinCorner, tempMaxCorner) >= 0)
					{
						if (first)
						{
							minCorner = tempMinCorner;
							maxCorner = tempMaxCorner;
							first = 0;
						}
						else
						{
							for (int j = 0; j < 3; j++)
							{
								if (tempMinCorner[j] < minCorner[j])
								{
									minCorner[j] = tempMinCorner[j];
								}
								
								if (tempMaxCorner[j] > maxCorner[j])
								{
									maxCorner[j] = tempMaxCorner[j];
								}
							}
						}
					}
				}
				
				// Nothing from the shapes?  Make the limits infinitesimal
				if (first)
				{
					minCorner = Ogre::Vector3::ZERO;
					maxCorner = Ogre::Vector3::ZERO;
				}
				break;
			}
		//case visualization_msgs::Marker::POINTS:
		//case visualization_msgs::Marker::TEXT_VIEW_FACING:
		//case visualization_msgs::Marker::TRIANGLE_LIST:
		case visualization_msgs::Marker::MESH_RESOURCE:
			{
				size_t vertex_count,index_count;
				Ogre::Vector3 * vertices;
				unsigned long * indices;
				unsigned int i;
				
				// Get the mesh information
				getMeshInformation(entity->getMesh(), vertex_count, vertices, index_count, indices, 
					subGraphicNode->_getDerivedPosition(), 
					subGraphicNode->_getDerivedOrientation(), 
					subGraphicNode->_getDerivedScale());

				// If there is no vertex data, return an infinitesimal space
				if (index_count == 0)
				{
					minCorner = Ogre::Vector3::ZERO;
					maxCorner = Ogre::Vector3::ZERO;
				}
				else
				{
					Ogre::Vector3 X = vertices[indices[0]];
					minCorner = X;
					maxCorner = X;

					// Loop through every vertex in the mesh
					for (i = 1; i < index_count; i++)
					{
						// Grow extents based on each vertex position
						X = vertices[indices[i]];

						for (int j = 0; j < 3; j++)
						{
							if (X[j] < minCorner[j])
							{
								minCorner[j] = X[j];
							}
							
							if (X[j] > maxCorner[j])
							{
								maxCorner[j] = X[j];
							}
						}
					}
				}
				
				delete [] vertices;
				delete [] indices;
				
				break;
			}
		default:
			{
				minCorner = Ogre::Vector3::ZERO;
				maxCorner = Ogre::Vector3::ZERO;
				break;
			}
	}
}

void GActiveMarker::showBounds(bool state)
{	
	// If the command doesn't change anything, ignore
	if ((boundsVisible && state && !boundsChanged) || (!boundsVisible && !state))
	{
		return;
	}
	
	if (state)
	{
		calculateBounds();
		
		// If the extents nearly zero ignore
		if ((maxCorner - minCorner).normalise() < 0.01)
		{
			return;
		}
		
		// If there is no manual object, create one 
		if (!boundsRect)
		{
			boundsRect = scene->createManualObject(name + "_bounds");
			
			boundsRect->setDynamic(true);
			boundsRect->estimateVertexCount(1000);
			boundsRect->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
		}
		else
		{
			boundsRect->beginUpdate(0);
		}
		
		// Adjust for graphicNode positon
		//Ogre::Vector3 pos = graphicNode->getPosition();
		//Ogre::Quaternion Rinv = (graphicNode->getOrientation()).Inverse();
		
		//Ogre::Vector3 minCornerAdj = Rinv * (minCorner - pos);
		//Ogre::Vector3 maxCornerAdj = Rinv * (maxCorner - pos);
		
		// Bottom face
		boundsRect->position(minCorner);
		
		// Choose colour based on whether the object has scripts attached to it
		// Green - active with scripts
		// White - no scripts
		
		bool gotScripts = false;
		for (int i = 0; i < 6; i++)
		{
			if (triggerScripts[i].size() > 0) 
			{
				gotScripts = true;
				break;
			}
		}
		
		if (gotScripts)
		{
			boundsRect->colour(Ogre::ColourValue(0, 1, 0, 1));
		}
		else
		{
			boundsRect->colour(Ogre::ColourValue(1, 1, 1, 1));
		}
		
		// More of the bottom face
		boundsRect->position(maxCorner[0], minCorner[1], minCorner[2]);
		
		boundsRect->position(maxCorner[0], minCorner[1], minCorner[2]);
		boundsRect->position(maxCorner[0], maxCorner[1], minCorner[2]);
		
		boundsRect->position(maxCorner[0], maxCorner[1], minCorner[2]);
		boundsRect->position(minCorner[0], maxCorner[1], minCorner[2]);
		
		boundsRect->position(minCorner[0], maxCorner[1], minCorner[2]);
		boundsRect->position(minCorner[0], minCorner[1], minCorner[2]);
		
		// Top face
		boundsRect->position(minCorner[0], minCorner[1], maxCorner[2]);
		boundsRect->position(maxCorner[0], minCorner[1], maxCorner[2]);
		
		boundsRect->position(maxCorner[0], minCorner[1], maxCorner[2]);
		boundsRect->position(maxCorner[0], maxCorner[1], maxCorner[2]);
		
		boundsRect->position(maxCorner[0], maxCorner[1], maxCorner[2]);
		boundsRect->position(minCorner[0], maxCorner[1], maxCorner[2]);
		
		boundsRect->position(minCorner[0], maxCorner[1], maxCorner[2]);
		boundsRect->position(minCorner[0], minCorner[1], maxCorner[2]);
		
		// Remaining edges
		boundsRect->position(minCorner[0], minCorner[1], minCorner[2]);
		boundsRect->position(minCorner[0], minCorner[1], maxCorner[2]);
		
		boundsRect->position(maxCorner[0], minCorner[1], minCorner[2]);
		boundsRect->position(maxCorner[0], minCorner[1], maxCorner[2]);
		
		boundsRect->position(maxCorner[0], maxCorner[1], minCorner[2]);
		boundsRect->position(maxCorner[0], maxCorner[1], maxCorner[2]);
		
		boundsRect->position(minCorner[0], maxCorner[1], minCorner[2]);
		boundsRect->position(minCorner[0], maxCorner[1], maxCorner[2]);
		
		boundsRect->end();
		
		if (!boundsVisible)
		{
			parent->attachObject(boundsRect);
			boundsVisible = true;
		}
		
		boundsChanged = false;
	}
	else
	{
		if (boundsRect && boundsVisible)
		{
			parent->detachObject(boundsRect);
		}
		
		boundsVisible = false;
		boundsChanged = false;
	}
}



