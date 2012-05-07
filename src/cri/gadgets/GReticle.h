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

#ifndef __GRETICLE
#define __GRETICLE

#include "CRI.h"
#include "IntersectionMath.h"

#include <Procedural.h>

#include "OgreTools2.h"
#include "GFrame.h"
#include "MovableText.h"

class GReticle : public Gadget
{
	public:
	
	static const int Spheres = 1;
	static const int Tori = 2;
	
	// Frame shift
	int targetType;
	string targetName;
	double relativePos[3];
	
	// Parameters
	// TODO: do we need these:
	int triangle;
	double x[3];
	
	// Frame parameters
	string frameName;
	Ogre::Quaternion frameR;
	Ogre::Vector3 frameX;
	
	// Parameters for selection disc
	int wasActive;
	int selectionDiscMode;
	int drawAxes;
	Ogre::Vector3 au, av, zu, zv, ru, rv;
	double azimuth, zenith, roll;
	double relAzimuth, startingAzimuth;
	double relZenith, startingZenith;
	double relRoll, startingRoll;
	int firstPass;
	int gimbalMode;
	
	double axesXTic, axesYTic, axesZTic, notchHeight;
	int bigNotchCount;
	
	// Parameters for broadcasting a frame
	int broadcastOn;
	string TFFrame;
	
	// Parameters for highlighting
	double inflation;
	ros::Time lastChange;
	int firstInflate;
	
	Ogre::Entity * centerSphere;
	
	Shape * azimuthSphere, * zenithSphere, * rollSphere;
	Shape * azimuthTorus, * zenithTorus, * rollTorus;
	
	Shape * azimuthCone, * zenithCone, * rollCone, * lateralCone;
	double azimuthConeDist, zenithConeDist, rollConeDist;
	double azimuthConeScale, zenithConeScale, rollConeScale;
	
	Shape * xCone1, * yCone1, * zCone1;
	Shape * xCone2, * yCone2, * zCone2;
	
	double axisConeDist[3];
	double axisConeScale[3];
	double axisCylinderDist[3];
	double axisCylinderRScale[3];
	double axisCylinderHScale[3];
	
	void show(bool value);
	void update();
	
	// We will use the mouse
	double distToClosest(Ogre::Vector3 origin, vector<Ogre::Vector3> & potentials);
	
	bool clickedTorus(int number, Ogre::Ray ray, vector<Ogre::Vector3> & potentials);
	bool clickedAxis(int number, Ogre::Ray ray, vector<Ogre::Vector3> & potentials);
	virtual void giveMouse(Ogre::Ray & ray, int leftClicked, int rightClicked, int active);
	virtual bool clicked(Ogre::Ray & ray, Ogre::Vector3 cameraPos, double & distance);
	virtual bool clickedInRange(Ogre::Ray & ray);
	
	// Our contribution to this gadget's Python interface
	void specifyAxes();
	void applyProperties();
	
	GReticle(const char * _name);
	void cleanupGimbal();
	~GReticle();
	
	private:
	
	Ogre::ManualObject * manual;
	Ogre::ManualObject * axesManual;
	// TODO: Neaten things up in the show-hide-gimbal department with scene nodes
	//Ogre::SceneNode * gimbalNode, * frameNode;
	Ogre::SceneNode * axesNode, * xTextNode, * yTextNode, * zTextNode;
	// TODO: Azimuth, Zenith and Roll texts
	Ogre::SceneNode * aziTextNode, * zenTextNode, * rollTextNode;
	Ogre::MovableText * xText, * yText, * zText, * aziText, * zenText, * rollText;
};

#endif
