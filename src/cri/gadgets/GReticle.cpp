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

#include "GReticle.h"

GReticle::GReticle(const char * _name) : Gadget(_name)
{
	manual = NULL;
	type = Gadget::GADGET_RETICLE_TRAN;
	
	addProperty(string("TargetType"), PYINT_INT, &targetType);
	addProperty(string("TargetName"), PYINT_STRING, &targetName);
	
	addProperty(string("X"), PYINT_DOUBLE, &relativePos[0]);
	addProperty(string("Y"), PYINT_DOUBLE, &relativePos[1]);
	addProperty(string("Z"), PYINT_DOUBLE, &relativePos[2]);
	
	// Direction properties
	selectionDiscMode = 0;
	addProperty(string("DirectionMode"), PYINT_INT, &selectionDiscMode);
	
	azimuth = 0;
	zenith = 0;
	roll = 0;
	
	addProperty(string("Azimuth"), PYINT_DOUBLE, &azimuth);
	addProperty(string("Zenith"), PYINT_DOUBLE, &zenith);
	addProperty(string("Roll"), PYINT_DOUBLE, &roll);

	gimbalMode = Tori;
	addProperty(string("GimbalMode"), PYINT_INT, &gimbalMode);

	azimuthSphere = NULL;
	zenithSphere = NULL;
	rollSphere = NULL;
	
	azimuthTorus = NULL;
	zenithTorus = NULL;
	rollTorus = NULL;
	
	azimuthCone = NULL;
	zenithCone = NULL;
	rollCone = NULL;
	lateralCone = NULL;
	
	xCone1 = NULL;
	xCone2 = NULL;
	
	axisConeDist[0] = 1.5;
	axisConeDist[1] = 1.5;
	axisConeDist[2] = 1.5;
	
	axisConeScale[0] = 0.25;
	axisConeScale[1] = 0.25;
	axisConeScale[2] = 0.25;
	
	axisCylinderDist[0] = 0.3;
	axisCylinderDist[1] = 0.3;
	axisCylinderDist[2] = 0.3;
	
	axisCylinderRScale[0] = 0.05;
	axisCylinderRScale[1] = 0.05;
	axisCylinderRScale[2] = 0.05;
	
	axisCylinderHScale[0] = 0.55;
	axisCylinderHScale[1] = 0.55;
	axisCylinderHScale[2] = 0.55;
	
	yCone1 = NULL;
	yCone2 = NULL;
	
	zCone1 = NULL;
	zCone2 = NULL;
	
	azimuthConeDist = 0.6;
	zenithConeDist = 0.6;
	rollConeDist = 0.6;

	au[0] = 1; au[1] = 0; au[2] = 0;
	av[0] = 0; av[1] = 1; av[2] = 0;
	
	// TF broadcast properties
	broadcastOn = 0;
	TFFrame = "/" + name;
	
	addProperty(string("BroadcastOn"), PYINT_INT, &broadcastOn);
	addProperty(string("TFFrame"), PYINT_STRING, &TFFrame);
	
	// Selection properties
	drawAxes = 1;
	axesXTic = 0.2;
	axesYTic = 0.2;
	axesZTic = 0.2;
	notchHeight = 0.05;
	axesNode = NULL;
	axesManual = NULL;
	bigNotchCount = 5;
	xText = NULL;
	yText = NULL;
	zText = NULL;
	xTextNode = NULL;
	yTextNode = NULL;
	zTextNode = NULL;
	
	frameX = Ogre::Vector3::ZERO;
	frameR = Ogre::Quaternion(1, 0, 0, 0);
	
	inflation = 1;
	addProperty(string("Inflation"), PYINT_DOUBLE, &inflation);
	
	// General properties
	addProperty(string("Frame"), PYINT_STRING, &frameName);
}

bool GReticle::clickedTorus(int number, Ogre::Ray ray, vector<Ogre::Vector3> & potentials)
{
	switch (number)
	{
		case 0:
			azimuthTorus->getIntersections(potentials, ray);
			break;
		case 1:
			zenithTorus->getIntersections(potentials, ray);
			break;		
		case 2:
			rollTorus->getIntersections(potentials, ray);
			break;
	}
	
	return potentials.size() > 0;
}

bool GReticle::clickedAxis(int number, Ogre::Ray ray, vector<Ogre::Vector3> & potentials)
{
	switch (number)
	{
		case 0:
			xCone1->getIntersections(potentials, ray);
			xCone2->getIntersections(potentials, ray);
			break;
		case 1:
			yCone1->getIntersections(potentials, ray);
			yCone2->getIntersections(potentials, ray);
			break;		
		case 2:
			zCone1->getIntersections(potentials, ray);
			zCone2->getIntersections(potentials, ray);
			break;
	}
				
	return potentials.size() > 0;
}

double GReticle::distToClosest(Ogre::Vector3 origin, vector<Ogre::Vector3> & potentials)
{
	double dist, temp;
	vector<Ogre::Vector3>::iterator potIt;
	
	if (potentials.size() == 0)
	{
		return -1;
	}
	
	potIt = potentials.begin();
	dist = (*potIt - origin).normalise();
	potIt++;
	
	for (; potIt != potentials.end(); potIt++)
	{
		temp = (*potIt - origin).normalise();
		
		if (temp < dist)
		{
			dist = temp;
		}
	}

	return dist;
}

void GReticle::giveMouse(Ogre::Ray & ray, int leftClicked, int rightClicked, int active)
{
	if (selectionDiscMode < 4 && selectionDiscMode > 0)
	{
		Ogre::Vector3 uv;
		Ogre::Vector3 vv;
		Ogre::Vector3 center = graphicNode->_getDerivedPosition(); //(relativePos[0], relativePos[1], relativePos[2]);
		
		Ogre::Quaternion Q;
		Q = graphicNode->_getDerivedOrientation();
		Ogre::Matrix3 R;
		Q.ToRotationMatrix(R);
		
		switch (selectionDiscMode)
		{
			case 1:
				uv = R * av;
				vv = R * au;
				break;
			case 2:
				uv = R * zv;
				vv = R * zu;
				break;
			case 3:
				uv = R * ru;
				vv = R * rv;
				break;
		}
		
		Ogre::Plane plane(uv.crossProduct(vv), center);
		std::pair<bool, Ogre::Real> tmp = ray.intersects(plane);
		
		Ogre::Vector3 pos = ray.getPoint(tmp.second);
		
		center = pos - center;
		
		double ud = center.dotProduct(uv);
		double vd = center.dotProduct(vv);
		
		switch (selectionDiscMode)
		{
			case 1:
			
				if (gimbalMode == Tori)
				{
					double angle = atan2(ud, vd);
					
					if (firstPass)
					{
						firstPass = 0;
						startingAzimuth = angle;
					}
					
					// It's a relative rotation, account for started position
					
					azimuth = relAzimuth + angle - startingAzimuth;
				}
				break;
			case 2:
				if (gimbalMode == Tori)
				{
					double angle = atan2(ud, vd);
					
					if (firstPass)
					{
						firstPass = 0;
						startingZenith = angle;
					}
					
					// It's a relative rotation, account for started position
					
					zenith = relZenith + angle - startingZenith;
				}
				break;
			case 3:
				if (gimbalMode == Tori)
				{
					double angle = atan2(ud, vd);
					
					if (firstPass)
					{
						firstPass = 0;
						startingRoll = angle;
					}
					
					// It's a relative rotation, account for started position
					
					roll = relRoll + angle - startingRoll;
				}
				
				//roll = atan2(ud, vd);
				break;
		}
		
		if (!active || leftClicked)
		{	
			selectionDiscMode = 7;
		}
		
		// Tell CRI to refresh this gadget
		needsApply = true;
	}
	else if (selectionDiscMode > 6)
	{
		if (active)
		{
			if (!wasActive)
			{
				// Hide the gimbal graphics
			
				if (gimbalMode == Spheres)
				{
					azimuthSphere->attachToNode(graphicNode);
					zenithSphere->attachToNode(graphicNode);
					rollSphere->attachToNode(graphicNode);
				}
				else if (gimbalMode == Tori)
				{
					
					rollTorus->attachToNode(graphicNode);
					azimuthTorus->attachToNode(graphicNode);
					zenithTorus->attachToNode(graphicNode);
				}
				xCone1->attachToNode(graphicNode);
				xCone2->attachToNode(graphicNode);
				yCone1->attachToNode(graphicNode);
				yCone2->attachToNode(graphicNode);
				zCone1->attachToNode(graphicNode);
				zCone2->attachToNode(graphicNode);
				
				parent->addChild(axesNode);
				
				wasActive = 1;
				
				return;
			}
			else if (leftClicked)
			{
				bool alreadyFound = false;
				double minDistance, tempDist;
				vector<Ogre::Vector3> tempPots;
				
				wasActive = 1;
				
				if (gimbalMode == Spheres)
				{	
					// Clicked the azimuth sphere?
					Ogre::Sphere asphere(Ogre::Vector3(relativePos[0], relativePos[1], relativePos[2]) + 
						1.1 * (cos(azimuth) * au + sin(azimuth) * av), 0.15);
					std::pair<bool, Ogre::Real> atmp = ray.intersects(asphere);
					
					if (atmp.first)
					{
						needsApply = true;
						selectionDiscMode = 1;
						alreadyFound = true;
					}
					
					// Clicked the zenith sphere?
					Ogre::Sphere zsphere(Ogre::Vector3(relativePos[0], relativePos[1], relativePos[2]) + 
						1.1 * (cos(zenith) * zu + sin(zenith) * zv), 0.15);
					std::pair<bool, Ogre::Real> ztmp = ray.intersects(zsphere);
					
					if (ztmp.first)
					{	
						needsApply = true;
						
						if (alreadyFound)
						{
							if (atmp.second < minDistance)
							{
								minDistance = atmp.second;
								selectionDiscMode = 2;
							}
						}
						else
						{
							alreadyFound = true;
							minDistance = atmp.second;
							selectionDiscMode = 2;
						}
					}
					
					// Clicked the roll sphere?
					Ogre::Sphere rsphere(Ogre::Vector3(relativePos[0], relativePos[1], relativePos[2]) + 
						1.1 * (sin(roll) * ru + cos(roll) * rv), 0.15);
					std::pair<bool, Ogre::Real> rtmp = ray.intersects(rsphere);
					
					if (rtmp.first)
					{
						needsApply = true;
						
						if (alreadyFound)
						{
							if (atmp.second < minDistance)
							{
								minDistance = atmp.second;
								selectionDiscMode = 3;
							}
						}
						else
						{
							alreadyFound = true;
							minDistance = atmp.second;
							selectionDiscMode = 3;
						}
					}
				}
				else if (gimbalMode == Tori)
				{
					
					if (clickedTorus(0, ray, tempPots)) 
					{
						needsApply = true;
						
						tempDist = distToClosest(ray.getOrigin(), tempPots);
						
						if (alreadyFound)
						{
							if (tempDist < minDistance)
							{
								relAzimuth = azimuth;
								selectionDiscMode = 1;
								firstPass = 1;
								minDistance = tempDist;
							}
						}
						else
						{
							relAzimuth = azimuth;
							selectionDiscMode = 1;
							firstPass = 1;
							minDistance = tempDist;
							alreadyFound = true;
						}
					}
					
					tempPots.clear();
					
					if (clickedTorus(1, ray, tempPots)) 
					{
						needsApply = true;
						
						tempDist = distToClosest(ray.getOrigin(), tempPots);
						
						if (alreadyFound)
						{
							if (tempDist < minDistance)
							{
								relZenith = zenith;
								selectionDiscMode = 2;
								firstPass = 1;
								minDistance = tempDist;
							}
						}
						else
						{
							relZenith = zenith;
							selectionDiscMode = 2;
							firstPass = 1;
							minDistance = tempDist;
							alreadyFound = true;
						}
					}
					
					tempPots.clear();
					
					if (clickedTorus(2, ray, tempPots)) 
					{
						needsApply = true;
						
						tempDist = distToClosest(ray.getOrigin(), tempPots);
						
						if (alreadyFound)
						{
							if (tempDist < minDistance)
							{
								relRoll = roll;
								selectionDiscMode = 3;
								firstPass = 1;
								minDistance = tempDist;
							}
						}
						else
						{
							relRoll = roll;
							selectionDiscMode = 3;
							firstPass = 1;
							minDistance = tempDist;
							alreadyFound = true;
						}
					}
					tempPots.clear();
				}
				
				// Clicked the arrows?
	
				if (clickedAxis(0, ray, tempPots)) 
				{
					needsApply = true;
						
					tempDist = distToClosest(ray.getOrigin(), tempPots);
					
					if (alreadyFound)
					{
						if (tempDist < minDistance)
						{
							selectionDiscMode = 4;
							minDistance = tempDist;
						}
					}
					else
					{
						selectionDiscMode = 4;
						minDistance = tempDist;
						alreadyFound = true;
					}
				}
				tempPots.clear();
				
				if (clickedAxis(1, ray, tempPots))
				{
					needsApply = true;
						
					tempDist = distToClosest(ray.getOrigin(), tempPots);
					
					if (alreadyFound)
					{
						if (tempDist < minDistance)
						{
							selectionDiscMode = 5;
							minDistance = tempDist;
						}
					}
					else
					{
						selectionDiscMode = 5;
						minDistance = tempDist;
						alreadyFound = true;
					}
				}
				tempPots.clear();
				
				if (clickedAxis(2, ray, tempPots))
				{
					needsApply = true;
						
					tempDist = distToClosest(ray.getOrigin(), tempPots);
					
					if (alreadyFound)
					{
						if (tempDist < minDistance)
						{
							selectionDiscMode = 6;
							minDistance = tempDist;
						}
					}
					else
					{
						selectionDiscMode = 6;
						minDistance = tempDist;
						alreadyFound = true;
					}
				}
				tempPots.clear();
			}
		}  // !active
		else if (wasActive && !active)
		{
			// Hide the gimbal graphics
			
			if (gimbalMode == Spheres)
			{
				azimuthSphere->detachFromNode();
				zenithSphere->detachFromNode();
				rollSphere->detachFromNode();
			}
			else if (gimbalMode == Tori)
			{
				azimuthTorus->detachFromNode();
				zenithTorus->detachFromNode();
				rollTorus->detachFromNode();
			}
			xCone1->detachFromNode();
			xCone2->detachFromNode();
			yCone1->detachFromNode();
			yCone2->detachFromNode();
			zCone1->detachFromNode();
			zCone2->detachFromNode();
			
			parent->removeChild(axesNode);
			
			wasActive = 0;
		}
		
	}
	else if (selectionDiscMode > 3 && selectionDiscMode < 7)
	{
		// TODO : Broken in different frame for eye(3)
		
		std::pair<Ogre::Vector3, Ogre::Vector3> intRes;
		
		Ogre::Matrix3 Rnode;
		Ogre::Quaternion Qnode = graphicNode->_getDerivedOrientation();
		Qnode.ToRotationMatrix(Rnode);
		Ogre::Vector3 Xorigin = graphicNode->_getDerivedPosition();
		
		switch (selectionDiscMode)
		{
			case 4:
				{					
					intRes = shortestBetweenLines(Xorigin, Rnode.GetColumn(0),
						ray.getOrigin(), ray.getDirection());
					
					Ogre::Vector3 res = intRes.first - Rnode.GetColumn(0) * (axisConeDist[0] - axisConeScale[0]/2) * frameScale;
					
					res = Rnode.Inverse() * (res - frameX);
					relativePos[0] = res[0]; relativePos[1] = res[1]; relativePos[2] = res[2];
				}
				break;
			case 5:
				{
					intRes = shortestBetweenLines(Xorigin, Rnode.GetColumn(1),
						ray.getOrigin(), ray.getDirection());
					
					Ogre::Vector3 res = intRes.first - Rnode.GetColumn(1) * (axisConeDist[1] - axisConeScale[1]/2) * frameScale;
					res = Rnode.Inverse() * (res - frameX);
					relativePos[0] = res[0]; relativePos[1] = res[1]; relativePos[2] = res[2];
				}
				break;
			case 6:
				{
					intRes = shortestBetweenLines(Xorigin, Rnode.GetColumn(2),
						ray.getOrigin(), ray.getDirection());
					
					Ogre::Vector3 res = intRes.first - Rnode.GetColumn(2) * (axisConeDist[2] - axisConeScale[2]/2) * frameScale;
					res = Rnode.Inverse() * (res - frameX);
					relativePos[0] = res[0]; relativePos[1] = res[1]; relativePos[2] = res[2];
				}
				break;
		}
		
		if (leftClicked || !active)
		{
			selectionDiscMode = 7;
		}
		
		needsApply = true;
	}
}

bool GReticle::clicked(Ogre::Ray & ray, Ogre::Vector3 cameraPos, double & distance)
{
	// Clicked a sphere around the frame?
	
	// Convert the relative position of the gimbal to world coordinates
	Ogre::Vector3 position = graphicNode->convertLocalToWorldPosition(Ogre::Vector3::ZERO);
	
	Ogre::Sphere sphere(position, frameScale);
	
	std::pair<bool, Ogre::Real> tmp = ray.intersects(sphere);
	
	if (tmp.first)
	{
		distance = (position - cameraPos).normalise(); 
		return true;
	}
	else
	{
		distance = -1;
		return false;
	}
}

bool GReticle::clickedInRange(Ogre::Ray & ray)
{
	// Clicked a larger sphere around the frame?
	
	// Convert the relative position of the gimbal to world coordinates
	Ogre::Vector3 position = graphicNode->convertLocalToWorldPosition(Ogre::Vector3::ZERO);
	
	Ogre::Sphere sphere(position, frameScale * 1.7);
		
	std::pair<bool, Ogre::Real> tmp = ray.intersects(sphere);
	
	if (tmp.first)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void GReticle::applyProperties()
{
	// If there is no manual object, create one 
	if (manual == NULL)
	{
		manual = scene->createManualObject(name);
		
		manual->setDynamic(true);
		manual->estimateVertexCount(1000);
		manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
		
		graphicNode->attachObject(manual);
	}
	else //Otherwise, modify the current one
	{
		manual->beginUpdate(0);
	}
	
	// Are we a graduated interactive marker?
	if (selectionDiscMode > 0)
	{
		double theta, sa, ca;
		
		// Draw axes
		if (!xCone1)
		{	
			xCone1 = new Shape(Shape::Cone, scene, graphicNode);
			xCone1->setPosition(Ogre::Vector3(axisConeDist[0], 0, 0));
			xCone1->setOrientation(CreateFrame(Ogre::Vector3(1, 0, 0)));
			xCone1->setScale(Ogre::Vector3(axisConeScale[0], axisConeScale[0], axisConeScale[0]));
			xCone1->setColor(Ogre::ColourValue(0, 0, 1, 1));
			
			xCone2 = new Shape(Shape::Cone, scene, graphicNode);
			xCone2->setPosition(Ogre::Vector3(axisConeDist[0] - axisConeScale[0], 0, 0));
			xCone2->setOrientation(CreateFrame(Ogre::Vector3(1, 0, 0)));
			xCone2->setScale(Ogre::Vector3(axisConeScale[0], -axisConeScale[0], axisConeScale[0]));
			xCone2->setColor(Ogre::ColourValue(0, 0, 1, 1));
			
			yCone1 = new Shape(Shape::Cone, scene, graphicNode);
			yCone1->setPosition(Ogre::Vector3(0, axisConeDist[1], 0));
			yCone1->setOrientation(CreateFrame(Ogre::Vector3(0, 1, 0)));
			yCone1->setScale(Ogre::Vector3(axisConeScale[1], axisConeScale[1], axisConeScale[1]));
			yCone1->setColor(Ogre::ColourValue(0, 0, 1, 1));
			
			yCone2 = new Shape(Shape::Cone, scene, graphicNode);
			yCone2->setPosition(Ogre::Vector3(0, axisConeDist[1] - axisConeScale[1], 0));
			yCone2->setOrientation(CreateFrame(Ogre::Vector3(0, 1, 0)));
			yCone2->setScale(Ogre::Vector3(axisConeScale[1], -axisConeScale[1], axisConeScale[1]));
			yCone2->setColor(Ogre::ColourValue(0, 0, 1, 1));
			
			zCone1 = new Shape(Shape::Cone, scene, graphicNode);
			zCone1->setPosition(Ogre::Vector3(0, 0, axisConeDist[2]));
			zCone1->setOrientation(CreateFrame(Ogre::Vector3(0, 0, 1)));
			zCone1->setScale(Ogre::Vector3(axisConeScale[2], axisConeScale[2], axisConeScale[2]));
			zCone1->setColor(Ogre::ColourValue(0, 0, 1, 1));
			
			zCone2 = new Shape(Shape::Cone, scene, graphicNode);
			zCone2->setPosition(Ogre::Vector3(0, 0, axisConeDist[2] - axisConeScale[2]));
			zCone2->setOrientation(CreateFrame(Ogre::Vector3(0, 0, 1)));
			zCone2->setScale(Ogre::Vector3(axisConeScale[2], -axisConeScale[2], axisConeScale[2]));
			zCone2->setColor(Ogre::ColourValue(0, 0, 1, 1));
			
		}
		
		Ogre::Vector3 adir;
		
		// Azimuth graphics
		{
			adir = cos(azimuth) * au + sin(azimuth) * av;
			
			if (gimbalMode == Spheres)
			{
				// Disc
				for (theta = 0; theta < (2 * M_PI - 0.21); theta += 0.2)
				{
					manual->position(cos(theta) * au + sin(theta) * av);
					manual->position(cos(theta + 0.2) * au + sin(theta + 0.2) * av);
				}
				manual->position(cos(theta - 0.21) * au + sin(theta - 0.21) * av);
				manual->position(au);
			}
			
			// Azimuth arrow head
			
			// Only show the arrow head for azimuth if we are selecting it
			if (selectionDiscMode == 1)
			{
				if (!azimuthCone)
				{
					azimuthCone = new Shape(Shape::Cone, scene, graphicNode);
				}
				
				azimuthCone->setPosition(1 * adir);
				azimuthCone->setOrientation(CreateFrame(adir));
				azimuthCone->setScale(Ogre::Vector3(0.15, 0.15, 0.15));
				azimuthCone->setColor(Ogre::ColourValue(0, 1, 1, 1));
			}
			else if (azimuthCone)
			{
				delete azimuthCone;
				
				azimuthCone = NULL;
			}
			
			// Rotation sphere
			if (gimbalMode == Spheres)
			{
				if (azimuthTorus != NULL)
				{
					delete azimuthTorus;
					azimuthTorus = NULL;
				}
				
				if (!azimuthSphere)
				{
					azimuthSphere = new Shape(Shape::Sphere, scene, graphicNode);
				}
				
				azimuthSphere->setPosition(1.1 * adir);
				azimuthSphere->setOrientation(Ogre::Quaternion(1, 0, 0, 0));
				azimuthSphere->setScale(Ogre::Vector3(0.15, 0.15, 0.15));
				
				if (selectionDiscMode == 1)
				{
					azimuthSphere->setColor(Ogre::ColourValue(1, 1, 1, 1));
				}
				else
				{
					azimuthSphere->setColor(Ogre::ColourValue(0, 1, 1, 1));
				}
			}
			else if (gimbalMode == Tori)
			{
				if (azimuthSphere != NULL)
				{
					delete azimuthSphere;
					azimuthSphere = NULL;
				}
				
				if (!azimuthTorus)
				{
					azimuthTorus = new Shape(Shape::Torus, scene, graphicNode, "Template/Carpet");
				}
				
				azimuthTorus->setPosition(Ogre::Vector3(0, 0, 0));
				
				Ogre::Matrix3 R;
				R.FromEulerAnglesXYZ(Ogre::Radian(0.0),
					Ogre::Radian(0.0), 
					Ogre::Radian(azimuth));
				Ogre::Matrix3 Rflatten;
				Rflatten.FromEulerAnglesXYZ(Ogre::Radian(M_PI/2),
					Ogre::Radian(0.0), 
					Ogre::Radian(0.0));
				Ogre::Quaternion Q(R * Rflatten);
	
				azimuthTorus->setOrientation(Q);
				azimuthTorus->setScale(Ogre::Vector3(1, 1, 1));
				if (selectionDiscMode == 1)
				{
					azimuthTorus->setColor(Ogre::ColourValue(1, 1, 1, 1));
				}
				else
				{
					azimuthTorus->setColor(Ogre::ColourValue(0, 1, 1, 1));
				}
			}
		}
		
		Ogre::Vector3 xdir;
		
		// Zenith graphics
		{
			zu[0] = cos(azimuth); zu[1] = sin(azimuth); zu[2] = 0; 
			zv[0] = 0; zv[1] = 0; zv[2] = 1;
			
			xdir = cos(zenith) * zu + sin(zenith) * zv;
			
			// X-Arrow line
			manual->position(0, 0, 0);
			manual->colour(0, 0, 1);
			manual->position(xdir);
			
			// X-Arrow head
			if (!zenithCone)
			{
				zenithCone = new Shape(Shape::Cone, scene, graphicNode);
			}
		
			zenithCone->setPosition(1 * xdir);
			zenithCone->setOrientation(CreateFrame(xdir));
			zenithCone->setScale(Ogre::Vector3(0.15, 0.15, 0.15));
			zenithCone->setColor(Ogre::ColourValue(0, 0, 1, 1));
			
			// Rotation gimbal graphics
			if (gimbalMode == Spheres)
			{
				if (zenithTorus != NULL)
				{
					delete zenithTorus;
					zenithTorus = NULL;
				}
				
				if (!zenithSphere)
				{
					zenithSphere = new Shape(Shape::Sphere, scene, graphicNode);
				}
				
				zenithSphere->setPosition(1.1 * xdir);
				zenithSphere->setOrientation(Ogre::Quaternion(1, 0, 0, 0));
				zenithSphere->setScale(Ogre::Vector3(0.15, 0.15, 0.15));
				
				if (selectionDiscMode == 2)
				{
					zenithSphere->setColor(Ogre::ColourValue(1, 1, 1, 1));
				}
				else
				{
					zenithSphere->setColor(Ogre::ColourValue(0, 0, 1, 1));
				}
			}
			else if (gimbalMode == Tori)
			{
				if (zenithSphere != NULL)
				{
					delete zenithSphere;
					zenithSphere = NULL;
				}
				
				if (!zenithTorus)
				{
					zenithTorus = new Shape(Shape::Torus, scene, graphicNode, "Template/Chessboard");
				}
				
				zenithTorus->setPosition(Ogre::Vector3(0, 0, 0));
				
				Ogre::Matrix3 R;
				R.FromEulerAnglesXYZ(Ogre::Radian(0),
					Ogre::Radian(0), 
					Ogre::Radian(azimuth));
					
				Ogre::Matrix3 R2;
				R2.FromEulerAnglesXYZ(Ogre::Radian(0.0),
					Ogre::Radian(-zenith), 
					Ogre::Radian(0.0));
					
				Ogre::Quaternion Q(R * R2);
	
				zenithTorus->setOrientation(Q);
				zenithTorus->setScale(Ogre::Vector3(1.1, 1.1, 1.1));
				
				if (selectionDiscMode == 2)
				{
					zenithTorus->setColor(Ogre::ColourValue(1, 1, 1, 1));
				}
				else
				{
					zenithTorus->setColor(Ogre::ColourValue(0, 0, 1, 1));
				}
			}

		}
		
		Ogre::Vector3 zdir, ydir;
		
		// Roll graphics
		{
			// Roll plane vectors
			ru = zu * cos(zenith + M_PI/2) + zv * sin(zenith + M_PI/2);
			rv = xdir.crossProduct(ru);
		
			zdir = sin(roll) * ru + cos(roll) * rv;
			ydir = zdir.crossProduct(xdir);
			
			// Z-axis - Arrow line
			manual->position(0, 0, 0);
			manual->colour(1, 0, 0);
			manual->position(zdir);
			
			// Y-axis - Arrow line
			manual->position(0, 0, 0);
			manual->colour(0, 1, 0);
			manual->position(ydir);
			
			// Z-Arrow head
			if (!rollCone)
			{
				rollCone = new Shape(Shape::Cone, scene, graphicNode);
			}
			
			rollCone->setPosition(zdir);
			rollCone->setOrientation(CreateFrame(zdir));
			rollCone->setScale(Ogre::Vector3(0.15, 0.15, 0.15));
			rollCone->setColor(Ogre::ColourValue(1, 0, 0, 1));
			
			// Y-Arrow head
			if (!lateralCone)
			{
				lateralCone = new Shape(Shape::Cone, scene, graphicNode);
			}
			
			lateralCone->setPosition(ydir);
			lateralCone->setOrientation(CreateFrame(ydir));
			lateralCone->setScale(Ogre::Vector3(0.15, 0.15, 0.15));
			lateralCone->setColor(Ogre::ColourValue(0, 1, 0, 1));
			
			// Rotation gimbal graphics
			if (gimbalMode == Spheres)
			{
				if (rollTorus != NULL)
				{
					delete rollTorus;
					rollTorus = NULL;
				}
				
				if (!rollSphere)
				{
					rollSphere = new Shape(Shape::Sphere, scene, graphicNode);
				}
			
				rollSphere->setPosition(1.1 * zdir);
				rollSphere->setOrientation(Ogre::Quaternion(1, 0, 0, 0));
				rollSphere->setScale(Ogre::Vector3(0.15, 0.15, 0.15));
			
				if (selectionDiscMode == 3)
				{
					rollSphere->setColor(Ogre::ColourValue(1, 1, 1, 1));
				}
				else
				{
					rollSphere->setColor(Ogre::ColourValue(1, 0, 0, 1));
				}
			}
			else if (gimbalMode == Tori)
			{
				if (rollSphere != NULL)
				{
					delete rollSphere;
					rollSphere = NULL;
				}
				
				if (!rollTorus)
				{
					rollTorus = new Shape(Shape::Torus, scene, graphicNode, "Template/Chessboard");
				}
				
				rollTorus->setPosition(Ogre::Vector3(0, 0, 0));
				
				Ogre::Matrix3 R;
				R.FromEulerAnglesXYZ(Ogre::Radian(0),
					Ogre::Radian(0), 
					Ogre::Radian(azimuth - M_PI/2));
					
				Ogre::Matrix3 R2;
				R2.FromEulerAnglesXYZ(Ogre::Radian(zenith),
					Ogre::Radian(0.0), 
					Ogre::Radian(0.0));
					
				Ogre::Matrix3 R3;
				R3.FromEulerAnglesXYZ(Ogre::Radian(0.0),
					Ogre::Radian(-roll), 
					Ogre::Radian(0.0));
				
				Ogre::Quaternion Q(R * R2 * R3);
	
				rollTorus->setOrientation(Q);
				rollTorus->setScale(Ogre::Vector3(1.2, 1.2, 1.2));
				
				if (selectionDiscMode == 3)
				{
					rollTorus->setColor(Ogre::ColourValue(1, 1, 1, 1));
				}
				else
				{
					rollTorus->setColor(Ogre::ColourValue(1, 0, 0, 1));
				}
			}
		}
	}
	else
	{
		// z-axis line
		// Note: Add vertex first, then change the color
		manual->position(0, 0, -1);
		// Make it green
		manual->colour(0, 1, 0);
		manual->position(0, 0, 1);
		
		// y-axis line
		manual->position(0, -1, 0);
		manual->position(0, 1, 0);

		// x-axis line
		manual->position(-1, 0, 0);
		manual->position(1, 0, 0);
	}
	
	manual->end();
	
	if (selectionDiscMode > 0 && selectionDiscMode < 7)
	{
		specifyAxes();
	}
	
	wasActive = 1;
	
	graphicNode->setScale(frameScale, frameScale, frameScale);
}

void GReticle::update()
{
	Ogre::Vector3 location;
	
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
	
	if (sourceFrame >= 0)
	{
		// Set the reticle's frame relative position
		// Leave the orientation globally fixed so that they're obvious
		
		tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
		tf::Vector3 & pos = tran.getOrigin();
		tf::Quaternion rot = tran.getRotation();
		
		frameR = Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z());
		frameX = Ogre::Vector3(pos[0], pos[1], pos[2]);
		
		location = frameR * (Ogre::Vector3(relativePos[0], relativePos[1], relativePos[2])) + frameX;
	
		graphicNode->setPosition(location);
		graphicNode->setOrientation(frameR);
		
		axesNode->setPosition(frameX);
		axesNode->setOrientation(frameR);
	}
	else
	{
		// Todo : It is not necessary to call this continually!
		
		location = Ogre::Vector3(relativePos[0], relativePos[1], relativePos[2]);
		
		frameX = Ogre::Vector3::ZERO;
		frameR = Ogre::Quaternion(1, 0, 0, 0);
		
		graphicNode->setPosition(location);
		graphicNode->setOrientation(frameR);
		
		axesNode->setPosition(frameX);
		axesNode->setOrientation(frameR);
	}
	
	// If this reticle is expected to broadcast it's data, do so
	// TODO:  Attach frame to reticle with line?
	
	if (broadcastOn && selectionDiscMode > 0)
	{
		tf::Transform transform;
		
		transform.setOrigin(tf::Vector3(location[0], location[1], location[2]));
		
		Ogre::Vector3 zu, zv, xdir, ru, rv, zdir, ydir;
		
		zu[0] = cos(azimuth); zu[1] = sin(azimuth); zu[2] = 0; 
		zv[0] = 0; zv[1] = 0; zv[2] = 1;
		xdir = cos(zenith) * zu + sin(zenith) * zv;
		ru = zu * cos(zenith + M_PI/2) + zv * sin(zenith + M_PI/2);
		rv = xdir.crossProduct(ru);
		zdir = sin(roll) * ru + cos(roll) * rv;
		ydir = zdir.crossProduct(xdir);
		
		Ogre::Matrix3 R1;
		
		R1.SetColumn(0, xdir);
		R1.SetColumn(1, ydir);
		R1.SetColumn(2, zdir);
		
		Ogre::Quaternion R = frameR * R1;
		
		transform.setRotation(tf::Quaternion(R.x, R.y, R.z, R.w));
		
		tfBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixedFrame, TFFrame));
	}
	
	// For 'inflation' highlighting (used in the pick selection GUI) 
	
	if (type != 4 && inflation > 1)
	{
		if (firstInflate)
		{
			lastChange = ros::Time::now();
			firstInflate = 0;
		}
		else
		{
			inflation -= (ros::Time::now() - lastChange).toSec() * 25;
			lastChange = ros::Time::now();
		}
		
		graphicNode->setOrientation(Ogre::Quaternion(cos(cos(inflation/3) * 0.2), 
			0, 0, sin(cos(inflation/3) * 0.2)));
		graphicNode->setScale(frameScale * inflation, frameScale * inflation, frameScale * inflation);
	}
	else
	{
		inflation = 1;
		firstInflate = 1;
		graphicNode->setScale(frameScale, frameScale, frameScale);
	}
}


void GReticle::show(bool value)
{
	// Make the graphic node visible if it can be
	if (visible)
	{
		if (!value)
		{
			parent->removeChild(graphicNode);
			parent->removeChild(axesNode);
			visible = false;
		}
	}
	else
	{
		if (value)
		{
			if (!graphicNode)
			{
				graphicNode = parent->createChildSceneNode();
				axesNode = parent->createChildSceneNode();
				
				// Create child scene nodes of axesNode
				xTextNode = axesNode->createChildSceneNode();	
				yTextNode = axesNode->createChildSceneNode();	
				zTextNode = axesNode->createChildSceneNode();	
			}
			else
			{
				parent->addChild(graphicNode);				
				parent->addChild(axesNode);
			}
			
			visible = true;
		}
	}
}

void GReticle::cleanupGimbal()
{
	if (azimuthSphere)
	{
		delete azimuthSphere;
		azimuthSphere = NULL;
	}
	
	if (zenithSphere)
	{
		delete zenithSphere;
		zenithSphere = NULL;
	}
	
	if (rollSphere)
	{
		delete rollSphere;
		rollSphere = NULL;
	}
	
	if (azimuthTorus)
	{
		delete azimuthTorus;
		azimuthTorus = NULL;
	}
	
	if (zenithTorus)
	{
		delete zenithTorus;
		zenithTorus = NULL;
	}
	
	if (rollTorus)
	{
		delete rollTorus;
		rollTorus = NULL;
	}
	
	if (azimuthCone)
	{
		delete azimuthCone;
		azimuthCone = NULL;
	}
	
	if (zenithCone)
	{
		delete zenithCone;
		zenithCone = NULL;
	}
	
	if (rollCone)
	{
		delete rollCone;
		rollCone = NULL;
	}
	
	if (lateralCone)
	{
		delete lateralCone;
		lateralCone = NULL;
	}

	if (xCone1)
	{
		delete xCone1;
		xCone1 = NULL;
	}
	
	if (yCone1)
	{
		delete yCone1;
		yCone1 = NULL;
	}
	
	if (zCone1)
	{
		delete zCone1;
		zCone1 = NULL;
	}
	
	if (xCone2)
	{
		delete xCone2;
		xCone2 = NULL;
	}
	
	if (yCone2)
	{
		delete yCone2;
		yCone2 = NULL;
	}
	
	if (zCone2)
	{
		delete zCone2;
		zCone2 = NULL;
	}
}

GReticle::~GReticle()
{
	
	if (manual)
	{
		if (graphicNode)
		{
			graphicNode->detachObject(manual);
		}
		scene->destroyManualObject(manual);	
	}
	
	cleanupGimbal();
	
	// Kill the additional scene node and manual object
	
	if (graphicNode)
	{
		parent->removeChild(graphicNode);
		scene->destroySceneNode(graphicNode);
	}
	
	if (axesNode)
	{
		parent->removeChild(axesNode);
		scene->destroySceneNode(axesNode);
	}
	
	if (xTextNode)
	{
		parent->removeChild(xTextNode);
		parent->removeChild(yTextNode);
		parent->removeChild(zTextNode);
		scene->destroySceneNode(xTextNode);
		scene->destroySceneNode(yTextNode);
		scene->destroySceneNode(zTextNode);
	}
}
