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

#ifndef __GQUIVER
#define __GQUIVER

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
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <boost/lexical_cast.hpp>

#include <Procedural.h>
#include "OgreTools2.h"

#include "validate_floats.h"

class GQuiver : public Gadget
{
	public:
	
	string topic;
	ros::Subscriber sub;
	int msgsRecieved;
	
	GQuiver(const char * _name);
	~GQuiver();
	
	void update();
	void show(bool value);	
	void applyProperties();
	
	void removeFirstIn();
	void cleanup();
	void addFrameToNode(Ogre::SceneNode * newNode);
	
	// todo : void getIntersections(vector<Ogre::Vector3> & potentials, Ogre::Ray & ray);
	
	void odometryCallback(const nav_msgs::Odometry msg);
	void poseStampedCallback(const geometry_msgs::PoseStamped msg);
	void poseArrayCallback(const geometry_msgs::PoseArray msg);
	
	void subscribeOdometry(string _topic);
	void subscribePoseStamped(string _topic);
	void subscribePoseArray(string _topic);
	
	vector<Shape *> xCones, yCones, zCones;
	vector<Shape *> xCylinders, yCylinders, zCylinders;
	vector<Ogre::SceneNode *> sceneNodes;
	Ogre::SceneNode * singleNode;
	
	//vector<ros::Time> creationTimes;
	
	int xColour[3];
	int yColour[3];
	int zColour[3];
	
	double positionTolerance;
	double angleTolerance;
	double scale;
	int countLimit;
	bool drawXAxis, drawYAxis, drawZAxis;
};

#endif
