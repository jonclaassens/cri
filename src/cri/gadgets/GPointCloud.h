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

#ifndef __GPOINTCLOUD
#define __GPOINTCLOUD

#include <OgreSubMesh.h>
#include <OgreBillboard.h>
#include <OgreBillboardSet.h>
#include <resource_retriever/retriever.h>
#include <boost/filesystem.hpp>

#include "CRI.h"

// To resolve some compatibility issue between ROS and CEGUI/OGRE headers
#ifdef Success
#undef Success
#endif

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include "GFrame.h"

#include <nav_msgs/Path.h>

#include <boost/lexical_cast.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "validate_floats.h"

// For reason we have to undef None for laser_geometry to include ?!
#undef None
#include <laser_geometry/laser_geometry.h>

class GPointCloud : public Gadget
{
	public:
	
	string topic;
	ros::Subscriber pcSub;
	//message_filters::Subscriber<sensor_msgs::LaserScan> lsSub;
	ros::Subscriber lsSub;
	int msgsRecieved;
	
	int elementNo;
	string materialName;
	
	int billboardType;
	double alpha;
	double billboardWidth;
	// Message persistence in seconds
	double persistence;
	
	// colourTransform can be
	// 0 - none
	// 1 - flat colour using scR, scG, scB
	// 2 - channel dependent and interpolating between sc and ec colours using minInt and maxInt as
	//     as bounds
	
	static const int CT_NONE = 0;
	static const int CT_FLAT = 1;
	static const int CT_CHANNEL = 2;
	static const int CT_RGB1 = 3;
	static const int CT_RGB3 = 4;
	
	static const int BT_CIRCLE = 1;
	static const int BT_DIAMOND = 2;
	static const int BT_CROSS = 3;
	
	int colourTransform;
	string xAxis;
	string yAxis;
	string zAxis;
	string rChannel;
	string gChannel;
	string bChannel;
	string colourChannel;
	string rgbChannel;
	int autoBoundMode;
	int unknownBounds;
	double minInt;
	double maxInt;
	int scR, scG, scB;
	int ecR, ecG, ecB;
	
	int firstMessage;
	
	laser_geometry::LaserProjection * laserProjector;
	ros::Duration filterTolerance;
	tf::MessageFilter<sensor_msgs::LaserScan> * tfFilter;
	
	//sensor_msgs::PointCloud2 data;
	
	GPointCloud(const char * _name);
	//GPointCloud(string _name);
	
	void update();
	void show(bool value);	
	void applyProperties();
	void cleanupData();
	
	// todo : void getIntersections(vector<Ogre::Vector3> & potentials, Ogre::Ray & ray);
	
	void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void subscribePointCloud2(string _topic);
	void subscribeLaserScan(string _topic);
	
	~GPointCloud();
	
	vector<ros::Time> creationTimes;
	
	// For points
	vector<Ogre::ManualObject *> manuals;
	vector<Ogre::ManualObject *> freeManuals;
	
	// For billboards
	vector<Ogre::BillboardSet *> billboards;
	vector<Ogre::BillboardSet *> freeBillboards;
	
	vector<Ogre::SceneNode *> nodes;
	vector<Ogre::SceneNode *> freeNodes;
	
	Ogre::MaterialPtr material;
};

#endif

