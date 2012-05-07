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

#define ROBOT

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>

#ifdef ROBOT
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include "RobotModel.hpp"
#include "ROSTools.h"
#endif

// Markers
#include <visualization_msgs/Marker.h> // Implemented   // In test
#include <visualization_msgs/MarkerArray.h>

// Navigation messages
#include <nav_msgs/GridCells.h>  // Implemented   // In test
#include <nav_msgs/OccupancyGrid.h> // Implemented   // In test
#include <nav_msgs/Path.h> // Implemented   // In test
#include <nav_msgs/Odometry.h> // Implemented   // In test

#include <sensor_msgs/LaserScan.h> // Implemented   // In test
#include <sensor_msgs/PointCloud.h> 
#include <sensor_msgs/PointCloud2.h> // Implemented   // In test
#include <sensor_msgs/Range.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/String.h>

#include <cri/MarkerScript.h>

#include "pcl_ros/point_cloud.h"

#include <string>
#include <map>

#include <boost/lexical_cast.hpp>

using namespace std;
 
void publishMarkerArrow(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0.5;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	
	//marker.points.resize(2);
	//marker.points[0].x = 0; marker.points[0].y = 0; marker.points[0].z = 0;
	//marker.points[1].x = 0; marker.points[1].y = 1; marker.points[1].z = 0;
	
	pub.publish(marker);
}

void publishMarkerCube(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 3;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 0.5;
	marker.color.g = 1.0;
	marker.color.b = 0.5;
		
	pub.publish(marker);
}

void publishMarkerSphere(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 6;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 0.5;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 1.0;
		
	pub.publish(marker);
}

void publishMarkerCylinder(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 9;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.5;
		
	pub.publish(marker);
}

void publishMarkerLineStrip(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 12;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	
	marker.points.resize(5);
	marker.points[0].x = 0; marker.points[0].y = 0; marker.points[0].z = 0;
	marker.points[1].x = 0; marker.points[1].y = 1; marker.points[1].z = 0;
	marker.points[2].x = 1; marker.points[2].y = 1; marker.points[2].z = 0;
	marker.points[3].x = 1; marker.points[3].y = 0; marker.points[3].z = 0;
	marker.points[4].x = 0; marker.points[4].y = 0; marker.points[4].z = 0;
	
	pub.publish(marker);
}

void publishMarkerLineList(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 15;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 0.7;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	
	marker.points.resize(4);
	marker.points[0].x = 0; marker.points[0].y = 0; marker.points[0].z = 0;
	marker.points[1].x = 0; marker.points[1].y = 1; marker.points[1].z = 0;
	marker.points[2].x = 1; marker.points[2].y = 1; marker.points[2].z = 0;
	marker.points[3].x = 1; marker.points[3].y = 0; marker.points[3].z = 0;
	
	pub.publish(marker);
}

void publishMarkerCubeList(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 18;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	
	marker.points.resize(5);
	marker.points[0].x = -2; marker.points[0].y = 0; marker.points[0].z = 0;
	marker.points[1].x = -1; marker.points[1].y = 0; marker.points[1].z = 0;
	marker.points[2].x = 0; marker.points[2].y = 0; marker.points[2].z = 0;
	marker.points[3].x = 1; marker.points[3].y = 0; marker.points[3].z = 0;
	marker.points[4].x = 2; marker.points[4].y = 0; marker.points[4].z = 0;
	
	marker.colors.resize(5);
	marker.colors[0].r = 1; marker.colors[0].g = 0; marker.colors[0].b = 0; marker.colors[0].a = 1.0;
	marker.colors[1].r = 0; marker.colors[1].g = 1; marker.colors[1].b = 0; marker.colors[1].a = 1.0;
	marker.colors[2].r = 0; marker.colors[2].g = 0; marker.colors[2].b = 1; marker.colors[2].a = 1.0;
	marker.colors[3].r = 1; marker.colors[3].g = 1; marker.colors[3].b = 0; marker.colors[3].a = 1.0;
	marker.colors[4].r = 0; marker.colors[4].g = 1; marker.colors[4].b = 1; marker.colors[4].a = 1.0;
	
	pub.publish(marker);
}

void publishMarkerSphereList(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 21;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	
	marker.points.resize(5);
	marker.points[0].x = -2; marker.points[0].y = 0; marker.points[0].z = 0;
	marker.points[1].x = -1; marker.points[1].y = 0; marker.points[1].z = 0;
	marker.points[2].x = 0; marker.points[2].y = 0; marker.points[2].z = 0;
	marker.points[3].x = 1; marker.points[3].y = 0; marker.points[3].z = 0;
	marker.points[4].x = 2; marker.points[4].y = 0; marker.points[4].z = 0;
	
	marker.colors.resize(5);
	marker.colors[0].r = 1; marker.colors[0].g = 0; marker.colors[0].b = 0; marker.colors[0].a = 1.0;
	marker.colors[1].r = 0; marker.colors[1].g = 1; marker.colors[1].b = 0; marker.colors[1].a = 1.0;
	marker.colors[2].r = 0; marker.colors[2].g = 0; marker.colors[2].b = 1; marker.colors[2].a = 1.0;
	marker.colors[3].r = 1; marker.colors[3].g = 1; marker.colors[3].b = 0; marker.colors[3].a = 1.0;
	marker.colors[4].r = 0; marker.colors[4].g = 1; marker.colors[4].b = 1; marker.colors[4].a = 1.0;
	
	pub.publish(marker);
}

void publishMarkerPoints(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 24;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 15;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	
	marker.points.resize(5);
	marker.points[0].x = -2; marker.points[0].y = 0; marker.points[0].z = 0;
	marker.points[1].x = -1; marker.points[1].y = 0; marker.points[1].z = 0;
	marker.points[2].x = 0; marker.points[2].y = 0; marker.points[2].z = 0;
	marker.points[3].x = 1; marker.points[3].y = 0; marker.points[3].z = 0;
	marker.points[4].x = 2; marker.points[4].y = 0; marker.points[4].z = 0;
	
	marker.colors.resize(5);
	marker.colors[0].r = 1; marker.colors[0].g = 0; marker.colors[0].b = 0; marker.colors[0].a = 1.0;
	marker.colors[1].r = 0; marker.colors[1].g = 1; marker.colors[1].b = 0; marker.colors[1].a = 1.0;
	marker.colors[2].r = 0; marker.colors[2].g = 0; marker.colors[2].b = 1; marker.colors[2].a = 1.0;
	marker.colors[3].r = 1; marker.colors[3].g = 1; marker.colors[3].b = 0; marker.colors[3].a = 1.0;
	marker.colors[4].r = 0; marker.colors[4].g = 1; marker.colors[4].b = 1; marker.colors[4].a = 1.0;
	
	pub.publish(marker);
}

void publishMarkerText(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 27;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.text = "Technology test!";
	
	pub.publish(marker);
}


void publishMarkerTriangle(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 30;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	
	marker.points.resize(6);
	marker.points[0].x = 0; marker.points[0].y = 0; marker.points[0].z = 0;
	marker.points[1].x = 0; marker.points[1].y = 1; marker.points[1].z = 0;
	marker.points[2].x = 1; marker.points[2].y = 1; marker.points[2].z = 0;
	marker.points[3].x = 0; marker.points[3].y = 0; marker.points[3].z = 1;
	marker.points[4].x = 0; marker.points[4].y = 1; marker.points[4].z = 1;
	marker.points[5].x = 1; marker.points[5].y = 1; marker.points[5].z = 1;
	
	marker.colors.resize(6);
	marker.colors[0].r = 1; marker.colors[0].g = 0; marker.colors[0].b = 0; marker.colors[0].a = 1.0; 
	marker.colors[1].r = 0; marker.colors[1].g = 1; marker.colors[1].b = 0; marker.colors[1].a = 1.0;
	marker.colors[2].r = 0; marker.colors[2].g = 0; marker.colors[2].b = 1; marker.colors[2].a = 1.0;
	marker.colors[3].r = 1; marker.colors[3].g = 1; marker.colors[3].b = 0; marker.colors[3].a = 1.0;
	marker.colors[4].r = 0; marker.colors[4].g = 1; marker.colors[4].b = 1; marker.colors[4].a = 1.0;
	marker.colors[5].r = 1; marker.colors[5].g = 1; marker.colors[5].b = 1; marker.colors[5].a = 1.0;
	
	pub.publish(marker);
}

void publishMarkerMesh(ros::Publisher & pub, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 33;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.04;
	marker.scale.y = 0.04;
	marker.scale.z = 0.04;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.mesh_resource = "package://cri/graphics/meshes/Dalek.stl";
	
	pub.publish(marker);
}

double lastPoint;

void publishPointCloud2(ros::Publisher & pub)
{
	double waveform;
	int i;
	int points = 300;
	
	sensor_msgs::PointCloud2 pc2;

	pc2.data.resize(points * 12);

	pc2.header.frame_id = string("/pc2");
	pc2.header.stamp = ros::Time::now();

	pc2.height = 1;
	pc2.width = points;
	pc2.point_step = 12;
	pc2.row_step = points;
	pc2.is_dense = false;

	pc2.fields.resize(3);
	pc2.fields[0].name = string("x");
	pc2.fields[0].offset = 0;
	pc2.fields[0].datatype = 7;
	pc2.fields[0].count = 1;

	pc2.fields[1].name = string("y");
	pc2.fields[1].offset = 4;
	pc2.fields[1].datatype = 7;
	pc2.fields[1].count = 1;

	pc2.fields[2].name = string("z");
	pc2.fields[2].offset = 8;
	pc2.fields[2].datatype = 7;
	pc2.fields[2].count = 1;

	for (i = 0; i < points * 12; i +=12) 
	{
		waveform = i / 12.0 / 100 + lastPoint;
		
		*((float *)&(pc2.data[i])) = 5 * sin(waveform);
		*((float *)&(pc2.data[i + 4])) = 5 * cos(3 * waveform);
		*((float *)&(pc2.data[i + 8])) = 5 * sin(5 * waveform);
	}

	lastPoint += 0.2;

	pub.publish(pc2);
}

void publishRGBPointCloud2(ros::Publisher & pub)
{
	double waveform;
	int i;
	int points = 300;
	
	sensor_msgs::PointCloud2 pc2;

	pc2.data.resize(points * 24);

	pc2.header.frame_id = string("/pc2");
	pc2.header.stamp = ros::Time::now();

	pc2.height = 1;
	pc2.width = points;
	pc2.point_step = 24;
	pc2.row_step = points;
	pc2.is_dense = false;

	pc2.fields.resize(6);
	pc2.fields[0].name = string("x");
	pc2.fields[0].offset = 0;
	pc2.fields[0].datatype = 7;
	pc2.fields[0].count = 1;

	pc2.fields[1].name = string("y");
	pc2.fields[1].offset = 4;
	pc2.fields[1].datatype = 7;
	pc2.fields[1].count = 1;

	pc2.fields[2].name = string("z");
	pc2.fields[2].offset = 8;
	pc2.fields[2].datatype = 7;
	pc2.fields[2].count = 1;
	
	pc2.fields[3].name = string("r");
	pc2.fields[3].offset = 12;
	pc2.fields[3].datatype = 7;
	pc2.fields[3].count = 1;

	pc2.fields[4].name = string("g");
	pc2.fields[4].offset = 16;
	pc2.fields[4].datatype = 7;
	pc2.fields[4].count = 1;

	pc2.fields[5].name = string("b");
	pc2.fields[5].offset = 20;
	pc2.fields[5].datatype = 7;
	pc2.fields[5].count = 1;

	for (i = 0; i < points * 24; i += 24) 
	{
		waveform = i / 24.0 / 100 + lastPoint;
		
		*((float *)&(pc2.data[i])) = 5 * sin(waveform);
		*((float *)&(pc2.data[i + 4])) = 5 * cos(3 * waveform);
		*((float *)&(pc2.data[i + 8])) = 5 * sin(5 * waveform);
		
		*((float *)&(pc2.data[i + 12])) = fabs(sin(waveform));
		*((float *)&(pc2.data[i + 16])) = fabs(cos(waveform));
		*((float *)&(pc2.data[i + 20])) = 0;
	}

	lastPoint += 0.2;

	pub.publish(pc2);
}

void publishLaserScan(ros::Publisher & pub)
{
	double waveform;
	int i;
	
	sensor_msgs::LaserScan laserScan;

	laserScan.header.frame_id = string("/scan");
	laserScan.header.stamp = ros::Time::now() - ros::Duration(1);

	laserScan.angle_min = -M_PI/2;
	laserScan.angle_max = M_PI/2;
	laserScan.angle_increment = M_PI / 360;
	
	laserScan.time_increment = 0.002;
	laserScan.scan_time = 0.002 * 360;

	laserScan.range_min = 0.5;
	laserScan.range_max = 15;
	
	laserScan.ranges.resize(360);
	laserScan.intensities.resize(360);
	for (i = 0; i < 360; i++)
	{
		waveform = (i - 180) / 180.0 * M_PI / 2;
		laserScan.ranges[i] = sin(waveform * 10 + lastPoint) + cos(waveform) * 10;
		laserScan.intensities[i] = sin(waveform);
	}
	
	pub.publish(laserScan);
}


void publishGridCells(ros::Publisher & pub)
{
	double inc;
	
	nav_msgs::GridCells gridCells;
	
	gridCells.header.frame_id = "/gridCells";
	gridCells.header.stamp = ros::Time::now();
	gridCells.cell_width = 0.5;
	gridCells.cell_height = 0.5;

	geometry_msgs::Point a;

	inc = 0;

	gridCells.cells.clear();
	for (double xx = -5; xx <= 5; xx +=0.5)
	{
		for (double yy = -5; yy <= 5; yy +=0.5)
		{
			a.x = xx; 
			a.y = yy; 
			a.z = -5 + sin(inc + lastPoint) * 0.1; 
			gridCells.cells.push_back(a);
			inc += 0.1;
		}
	}
	
	pub.publish(gridCells);
}

int firstOccMap = 1;
nav_msgs::OccupancyGrid occGrid;

void publishOccupancyGrid(ros::Publisher & pub)
{
	int x, y;
	double r;
	
	if (firstOccMap)
	{
		occGrid.header.frame_id = "/occupancyGrid";
		occGrid.header.stamp = ros::Time::now();
		
		occGrid.info.map_load_time = ros::Time();
		occGrid.info.resolution = 0.01;
		occGrid.info.width = 1024;
		occGrid.info.height = 1024;
		occGrid.info.origin.position.x = 0;
		occGrid.info.origin.position.y = 0;
		occGrid.info.origin.position.z = -5;
		
		occGrid.info.origin.orientation.x = 0;
		occGrid.info.origin.orientation.y = 0;
		occGrid.info.origin.orientation.z = 0;
		occGrid.info.origin.orientation.w = 1;
		
		occGrid.data.resize(1024 * 1024);
		
		firstOccMap = 0;
	}

	for (y = 0; y < 1024; y++)
	{
		for (x = 0; x < 1024; x++)
		{
			occGrid.data[x + 1024 * y] = (unsigned char) (floor(0.8 * occGrid.data[x + 1024 * y]));
			
			r = sqrt((x - 512) * (x - 512) + (y - 512) * (y - 512)) / 512.0;
			if (abs(r - sin(lastPoint)) < 0.1)
			{
				occGrid.data[x + 1024 * y] = 1;
			}
		}
	}
	
	pub.publish(occGrid);
}

#define BUBBLE_COUNT 50
int bubbleBurst[BUBBLE_COUNT];
double bubbleX[BUBBLE_COUNT];
double bubbleY[BUBBLE_COUNT];
double bubbleHeight[BUBBLE_COUNT];
double bubbleSpeed[BUBBLE_COUNT];
ros::Publisher * vis_pub_ptr;

void publishBubbles(ros::Publisher & pub, ros::Publisher & scriptPub)
{
	int i;
	
	visualization_msgs::Marker marker;
	cri::MarkerScript markerScript;
	
	for (i = 0; i < BUBBLE_COUNT; i++)
	{
		bubbleHeight[i] += bubbleSpeed[i];
		if (bubbleHeight[i] > 10)
		{
			bubbleHeight[i] = 0;
		}
	
		marker.header.frame_id = "/world";
		marker.header.stamp = ros::Time::now();
		marker.ns = "bubbles";
		marker.id = i;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = bubbleX[i];
		marker.pose.position.y = bubbleY[i];
		marker.pose.position.z = bubbleHeight[i];
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		if (bubbleBurst[i])
		{
			if (bubbleBurst[i] == 3)
			{
				bubbleBurst[i] = 0;
				
				bubbleX[i] = -15 - 15.0 * rand() * 1.0 / RAND_MAX;
				bubbleY[i] = 15 + 15.0 * rand() * 1.0 / RAND_MAX;
				bubbleHeight[i] = 0;
				bubbleSpeed[i] = 0.02 * rand() * 1.0 / RAND_MAX + 0.01;
				
				// To be sure the arrow was removed
				visualization_msgs::Marker marker;
	
				marker.header.frame_id = "/world";
				marker.header.stamp = ros::Time::now();
				marker.ns = "Arrow";
				marker.id = i;
				marker.type = visualization_msgs::Marker::ARROW;
				marker.action = visualization_msgs::Marker::DELETE;
			
				vis_pub_ptr->publish(marker);
				
				continue;
			}
			
			marker.scale.x = bubbleSpeed[i] * 25 + pow(bubbleBurst[i],2) * 0.3;
			marker.scale.y = bubbleSpeed[i] * 25 + pow(bubbleBurst[i],2) * 0.3;
			marker.scale.z = bubbleSpeed[i] * 25 + pow(bubbleBurst[i],2) * 0.3;
			
			marker.color.a = 0.5 + 0.5 * bubbleBurst[i] / 3.0;
			marker.color.r = 0.5 + 0.5 * bubbleBurst[i] / 3.0;
			marker.color.g = 0.5 + 0.5 * bubbleBurst[i] / 3.0;
			marker.color.b = 1.0;
			
			bubbleBurst[i]++;
		}
		else
		{
			marker.scale.x = bubbleSpeed[i] * 25;
			marker.scale.y = bubbleSpeed[i] * 25;
			marker.scale.z = bubbleSpeed[i] * 25;
			
			marker.color.a = 0.5;
			marker.color.r = 0.5;
			marker.color.g = 0.5;
			marker.color.b = 1.0;
		}
		
			
		pub.publish(marker);
		
		markerScript.ns = "bubbles";
		markerScript.id = i;
		markerScript.action = cri::MarkerScript::ADD_OR_MODIFY;
		markerScript.script_name = "Script1";
		markerScript.trigger_event_type = cri::MarkerScript::LEFT_CLICK;
		//markerScript.filename = "package://cri/src/tutorials/technology_test/BubbleClick.py";
		
		markerScript.code = "pub.publish('" + boost::lexical_cast<string>(i) + "')\n";
		
		scriptPub.publish(markerScript);
		
		markerScript.trigger_event_type = cri::MarkerScript::BECOME_ACTIVE;
		markerScript.code = "pub.publish('" + boost::lexical_cast<string>(1000 + i) + "')\n";
		
		scriptPub.publish(markerScript);
		
		markerScript.trigger_event_type = cri::MarkerScript::BECOME_INACTIVE;
		markerScript.code = "pub.publish('" + boost::lexical_cast<string>(2000 + i) + "')\n";
		
		scriptPub.publish(markerScript);
	}
}

void popCallback(const std_msgs::String& msg)
{
	int number = atoi(msg.data.c_str());
	
	if (number < 1000)
	{
		bubbleBurst[number] = 1;
		
		visualization_msgs::Marker marker;
	
		marker.header.frame_id = "/world";
		marker.header.stamp = ros::Time::now();
		marker.ns = "Arrow";
		marker.id = number - 2000;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::DELETE;
		
		vis_pub_ptr->publish(marker);
	}
	else if (number < 2000)
	{
		visualization_msgs::Marker marker;
	
		marker.header.frame_id = "/world";
		marker.header.stamp = ros::Time::now();
		marker.ns = "Arrow";
		marker.id = number - 1000;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = bubbleX[number - 1000] - 1.2;
		marker.pose.position.y = bubbleY[number - 1000];
		marker.pose.position.z = bubbleHeight[number - 1000];
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		
		vis_pub_ptr->publish(marker);
	}
	else if (number < 3000)
	{
		visualization_msgs::Marker marker;
	
		marker.header.frame_id = "/world";
		marker.header.stamp = ros::Time::now();
		marker.ns = "Arrow";
		marker.id = number - 2000;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::DELETE;
		
		vis_pub_ptr->publish(marker);
	}
}

void publishRobotAndTable(ros::Publisher & pub)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "table";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = -15;
	marker.pose.position.y = 5;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = cos(M_PI/4);
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = sin(M_PI/4);
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.mesh_resource = "package://cri/graphics/meshes/table.stl";
	
	pub.publish(marker);
}

void publishPath(ros::Publisher & pub)
{
	double th;
	int i;
	
	nav_msgs::Path path;
	
	path.header.frame_id = "/path";
	path.header.stamp = ros::Time::now();
	
	path.poses.resize(6);
	for (i = 0; i < 6; i++)
	{
		th = i * 0.2;
		
		path.poses[i].pose.position.x = 5 * cos(th + lastPoint);
		path.poses[i].pose.position.y = 5 * sin(th + lastPoint);
		path.poses[i].pose.position.z = 0;
		
		path.poses[i].pose.orientation.x = 0;
		path.poses[i].pose.orientation.y = 0;
		path.poses[i].pose.orientation.z = 0;
		path.poses[i].pose.orientation.w = 1;
	} 
	
	pub.publish(path);
}

void publishOdometry(ros::Publisher & pub)
{
	double th;
	int i;
	
	nav_msgs::Odometry odo;
	
	odo.header.frame_id = "/odo";
	odo.header.stamp = ros::Time::now();

	odo.pose.pose.position.x = 5 * cos(lastPoint / 3);
	odo.pose.pose.position.y = 5 * sin(lastPoint / 3);
	odo.pose.pose.position.z = 0;
		
	odo.pose.pose.orientation.x = 0;
	odo.pose.pose.orientation.y = 0;
	odo.pose.pose.orientation.z = sin(-lastPoint / 6);
	odo.pose.pose.orientation.w = cos(-lastPoint / 6);
	
	
	pub.publish(odo);
}

void publishPoseStamped(ros::Publisher & pub)
{
	double th;
	int i;
	
	geometry_msgs::PoseStamped pose;

	pose.header.frame_id = "/odo";
	pose.header.stamp = ros::Time::now();

	pose.pose.position.x = 5 * cos(lastPoint / 3);
	pose.pose.position.y = 0;
	pose.pose.position.z = 5 * sin(lastPoint / 3);
		
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = sin(lastPoint / 6);
	pose.pose.orientation.w = cos(lastPoint / 6);
	
	pub.publish(pose);
}

void publishPoseArray(ros::Publisher & pub)
{
	double th;
	int i;
	
	geometry_msgs::PoseArray pose;

	pose.header.frame_id = "/odo";
	pose.header.stamp = ros::Time::now();

	pose.poses.resize(7);
	
	for (i = 0; i < 7; i++)
	{
		th = 0.1 * i;
		
		pose.poses[i].position.x = 0;
		pose.poses[i].position.y = 5 * cos((th + lastPoint) / 3);
		pose.poses[i].position.z = 5 * sin((th + lastPoint) / 3);
			
		pose.poses[i].orientation.x = 0;
		pose.poses[i].orientation.y = 0;
		pose.poses[i].orientation.z = sin((th + lastPoint) / 6);
		pose.poses[i].orientation.w = cos((th + lastPoint) / 6);
	}
	
	pub.publish(pose);
}

int main(int argc, char** argv)
{
	int i;
	double th;
	
	ros::init(argc, argv, "my_tf_broadcaster");
	ros::NodeHandle node;

	tf::TransformBroadcaster br;
	tf::Transform transform;

	ros::Rate rate(5.0);

	// Visualization marker publisher
	ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 100);
	vis_pub_ptr = &vis_pub; 
	
	// Script tag publisher
	ros::Publisher script_pub = node.advertise<cri::MarkerScript>( "visualization_marker_script", 100);
	
	ros::Subscriber popSub = node.subscribe("pop", 10, popCallback);
	
	cri::MarkerScript markerScript;
	
	markerScript.ns = "";
	markerScript.id = -1;
	markerScript.action = cri::MarkerScript::ADD_OR_MODIFY;
	markerScript.script_name = "Script1";
	markerScript.trigger_event_type = cri::MarkerScript::LEFT_CLICK;
	markerScript.filename = "package://cri/src/tutorials/technology_test/InitBubble.py";
	
	// Why 3???
	script_pub.publish(markerScript);
	
	ros::spinOnce();
	rate.sleep();
	
	script_pub.publish(markerScript);
	
	ros::spinOnce();
	rate.sleep();
	
	script_pub.publish(markerScript);
	
	ros::spinOnce();
	rate.sleep();
					
			
	// Point cloud 2 publishers
    ros::Publisher plainpc_pub = node.advertise<sensor_msgs::PointCloud2>( "plain_random_cloud", 20);
    ros::Publisher rgbpc_pub = node.advertise<sensor_msgs::PointCloud2>( "rgb_random_cloud", 20);
    ros::Publisher ls_pub = node.advertise<sensor_msgs::LaserScan>( "laser_scan_cloud", 20);
    lastPoint = 0;
    
    // Grid cells publisher
    ros::Publisher gc_pub = node.advertise<nav_msgs::GridCells>( "grid_cells", 20);
    
    // Occupancy grid publisher
    ros::Publisher og_pub = node.advertise<nav_msgs::OccupancyGrid>( "occupancy_grid", 20);
    
    // Path publisher
    ros::Publisher path_pub = node.advertise<nav_msgs::Path>( "path_data", 20);
    
    // Odometry publisher
    ros::Publisher odo_pub = node.advertise<nav_msgs::Odometry>( "odo_data", 20);
    
    // Pose publisher
    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>( "pose_data", 20);
    
    // PoseArray publisher
    ros::Publisher pose_array_pub = node.advertise<geometry_msgs::PoseArray>( "pose_array_data", 20);
    
	th = 0;
   
	// Initialise bubble active marker demo
	for (i = 0; i < BUBBLE_COUNT; i++)
	{
		bubbleX[i] = -15 - 15.0 * rand() * 1.0 / RAND_MAX;
		bubbleY[i] = 15 + 15.0 * rand() * 1.0 / RAND_MAX;
		bubbleHeight[i] = 0;
		bubbleSpeed[i] = 0.02 * rand() * 1.0 / RAND_MAX + 0.01;
		bubbleBurst[i] = 0;
	}
	
	#ifdef ROBOT
	KDL::Tree tree;
	
	// Find the CRI package directory
	string packagePath = ros::package::getPath("cri");
	string URDFFilename = packagePath + "/graphics/example_robot/urdf/wam.urdf";
	
    if (!kdl_parser::treeFromFile(URDFFilename, tree))
    {
		ROS_ERROR("Failed to construct kdl tree");
		return -1;
	}
	
	robot_state_publisher::RobotStatePublisher robotStatePub(tree);
	
	std::map<std::string, double> robotConfig;
	
	RobotModel * rm = new RobotModel();
	string model = getTextFromFile(URDFFilename);
	robotConfig["j1_joint"] = 0;
	robotConfig["j2_joint"] = 0;
	robotConfig["j3_joint"] = 0;
	robotConfig["j4_joint"] = 0;
	robotConfig["j5_joint"] = 0;
	robotConfig["j6_joint"] = 0;
	robotConfig["j7_joint"] = 0;
	
	robotStatePub.publishFixedTransforms();
	robotStatePub.publishTransforms(robotConfig, ros::Time::now());
	
	#endif
   
    int ensure = 5;
   
	while (node.ok())
	{
		// Need a few frames
		th += 0.1;
		transform.setOrigin(tf::Vector3(25.0 + cos(th) * 5, 25.0, sin(th) * 5));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/pc2"));
		
		transform.setOrigin(tf::Vector3(15.0, 15.0, 0));
		transform.setRotation(tf::Quaternion(sin(th/2.0), 0, 0, cos(th/2.0)));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/scan"));
		
		transform.setOrigin(tf::Vector3(25.0, -25.0,0));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/gridCells"));
		
		transform.setOrigin(tf::Vector3(15.0, -15.0,0));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/occupancyGrid"));
		
		transform.setOrigin(tf::Vector3(-15.0, 0, 1.5));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/robotHome"));
		
		transform.setOrigin(tf::Vector3(-15.0, -15, 1.5));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/path"));
		
		transform.setOrigin(tf::Vector3(-25.0, -25, 1.5));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/odo"));
		
		#ifdef ROBOT
		robotStatePub.publishFixedTransforms();
		robotStatePub.publishTransforms(robotConfig, ros::Time::now());
		
		if (ensure > 0)
		{
			rm->go(node, model);
			ensure--;
		}
		#endif 
	
		// Publish example markers
		publishMarkerArrow(vis_pub, 0);
		publishMarkerCube(vis_pub, 1);
		publishMarkerSphere(vis_pub, 2);
		publishMarkerCylinder(vis_pub, 3);
		publishMarkerLineStrip(vis_pub, 4);
		publishMarkerLineList(vis_pub, 5);
		publishMarkerCubeList(vis_pub, 6);
		publishMarkerSphereList(vis_pub, 7);
		publishMarkerPoints(vis_pub, 8);
		publishMarkerText(vis_pub, 9);
		publishMarkerTriangle(vis_pub, 10);
		publishMarkerMesh(vis_pub, 11);
		
		// Publish example point clouds
		publishPointCloud2(plainpc_pub);
		publishRGBPointCloud2(rgbpc_pub);
		publishLaserScan(ls_pub);
		
		// Publish example grid cells
		publishGridCells(gc_pub);
		
		// Publish example occupancy grid
		publishOccupancyGrid(og_pub);
		
		// Publish example path
		publishPath(path_pub);
		
		// Publish example odometry
		publishOdometry(odo_pub);
		
		// Publish example pose
		publishPoseStamped(pose_pub);
		
		// Publish example pose array
		publishPoseArray(pose_array_pub);
		
		// Bubbles demo
		publishBubbles(vis_pub, script_pub);
		
		// Robot demo
		publishRobotAndTable(vis_pub);
		
		ros::spinOnce();
		
		rate.sleep();
	}

	return 0;
};
 
