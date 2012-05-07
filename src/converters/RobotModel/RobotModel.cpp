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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <tinyxml.h>

#include <Eigen/Core>
#include <Eigen/Dense> 
#include <Eigen/Geometry> 

#include <string>
#include <boost/shared_ptr.hpp>
#include <vector>

#include "RobotModel.hpp"

using namespace Eigen;
using namespace std;

int RobotModel::go(ros::NodeHandle & n)
{
	// Submit that this program will publish markers
	visPub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 30, true);
	
	// Get the URDF data
	std::string key;
	if (n.searchParam("robot_description", key))
	{
		n.getParam(key, modelText);
	}
	else
	{
		return -1;
	}
	
	ROS_INFO("Found URDF data.");
	
	return parseAndCreate();
}

int RobotModel::go(ros::NodeHandle & n, string data)
{
	// Submit that this program will publish markers
	visPub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 30, true);
	
	modelText = data;
	
	return parseAndCreate();
}

int RobotModel::parseAndCreate()
{
	TiXmlDocument modelDoc;
	
	// Parse XML file
	modelDoc.Parse(modelText.c_str());
	if (!modelDoc.RootElement())
	{
		ROS_ERROR("URDF failed XML parse");
		return -1;
	}

	if (!description.initXml(modelDoc.RootElement()))
	{
		ROS_ERROR("URDF failed Model parse");
		return -1;
	}

	ROS_INFO("URDF parsed.");

	// Get linkages
	description.getLinks(linkages);
	
	// Blank marker
	visualization_msgs::Marker marker;
	
	// Run through linkages and publish their geometry as markers
	linkVector::iterator lIt;
	
	ros::Rate rate(100.0);
	
	for (lIt = linkages.begin(); lIt != linkages.end(); lIt++)
	{
		// We only consider linkages with geometry
		if (!(*lIt)->visual || !(*lIt)->visual->geometry)
		{
			continue;
		}
		urdf::Geometry& geometry = *((*lIt)->visual->geometry);
		
		// We need the geometry and origin frame of the linkage 
		urdf::Pose& pose = (*lIt)->visual->origin;
		
		// Fill in the new marker's data
		std::string linkName = (*lIt)->name;
		std::string prefix;
	
		linkName = tf::resolve(prefix, linkName);

		marker.header.frame_id = linkName;
		
		marker.header.stamp = ros::Time::now();
		
		// Need names here!!!
		marker.ns = "Robot";
		marker.id = nameNo++;
		
		// Add == Modify in CRI
		marker.action = visualization_msgs::Marker::ADD;
		
		// Marker position
		marker.pose.position.x = pose.position.x;
		marker.pose.position.y = pose.position.y;
		marker.pose.position.z = pose.position.z;
		
		// Orientation
		Eigen::Quaternionf startOrientQ(pose.rotation.w, 
			pose.rotation.x, 
			pose.rotation.y, 
			pose.rotation.z);
		Eigen::Matrix3f orient;
		orient = startOrientQ.toRotationMatrix();
		
		// Marker color
		const urdf::Color& color = (*lIt)->visual->material->color;
		marker.color.a = color.a;
		marker.color.r = color.r;
		marker.color.g = color.g;
		marker.color.b = color.b;

		switch (geometry.type)
		{
			case urdf::Geometry::SPHERE:
				{
					const urdf::Sphere& sphere = static_cast<const urdf::Sphere&>(geometry);
					marker.type = visualization_msgs::Marker::SPHERE;

					marker.scale.x = sphere.radius*2;
					marker.scale.y = sphere.radius*2;
					marker.scale.z = sphere.radius*2;
					break;
				}

			case urdf::Geometry::BOX:
				{
					const urdf::Box& box = static_cast<const urdf::Box&>(geometry);
					marker.type = visualization_msgs::Marker::CUBE;

					marker.scale.x = box.dim.x;
					marker.scale.y = box.dim.y;
					marker.scale.z = box.dim.z;
					break;
				}

			case urdf::Geometry::CYLINDER:
				{
					const urdf::Cylinder& cylinder = static_cast<const urdf::Cylinder&>(geometry);
					marker.type = visualization_msgs::Marker::CYLINDER;


					Eigen::Matrix3f rotX;
					rotX = Eigen::AngleAxisf(M_PI / 2,  Vector3f::UnitX());
					orient = orient * rotX;

					marker.scale.x = cylinder.radius*2;
					marker.scale.y = cylinder.length;
					marker.scale.z = cylinder.radius*2;
					break;
				}

			case urdf::Geometry::MESH:
				{
					const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(geometry);

					marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					marker.scale.x = mesh.scale.x;
					marker.scale.y = mesh.scale.y;
					marker.scale.z = mesh.scale.z;

					if (mesh.filename.empty())
					{
						break;
					}

					marker.mesh_resource = mesh.filename;
					printf("Model mesh %s added.\n", mesh.filename.c_str());
					break;
				}
		}
		
		Eigen::Quaternionf orientQ(orient);
		
		 
		marker.pose.orientation.x = orientQ.x();
		marker.pose.orientation.y = orientQ.y();
		marker.pose.orientation.z = orientQ.z();
		marker.pose.orientation.w = orientQ.w();
		
		visPub.publish(marker);
		ros::spinOnce();
		rate.sleep();
	}
	
	// Need to sleep (for some reason)
	
	ROS_INFO("Done building visual model.");
	
	return 0;
}

