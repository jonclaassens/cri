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

#include "GPointCloud.h"

#include "validate_floats.h"

GPointCloud::GPointCloud(const char * _name) : Gadget(_name)
{
	type = Gadget::GADGET_POINTCLOUD;
	
	elementNo = 0;
	
	msgsRecieved = 0;
	
	// Default parameters
	billboardType = 0;
	alpha = 1;
	billboardWidth = 3;
	persistence = 1;

	colourTransform = 2;
	xAxis = "x";
	yAxis = "y";
	zAxis = "z";
	rChannel = "r";
	gChannel = "g";
	bChannel = "b";
	rgbChannel = "rgb";

	colourChannel = "intensity";
	autoBoundMode = 1;
	unknownBounds = 1;
	minInt = 0;
	maxInt = 0;
	scR = 0; scG = 0; scB = 0;
	ecR = 255; ecG = 255; ecB = 255;
	
	// Publish parameters to Python
	addProperty(string("BillboardType"), PYINT_INT, &billboardType);
	addProperty(string("Alpha"), PYINT_DOUBLE, &alpha);
	addProperty(string("BillboardWidth"), PYINT_DOUBLE, &billboardWidth);
	addProperty(string("Persistence"), PYINT_DOUBLE, &persistence);
	
	addProperty(string("ColourTransform"), PYINT_INT, &colourTransform);
	addProperty(string("XAxisChannel"), PYINT_STRING, &xAxis);
	addProperty(string("YAxisChannel"), PYINT_STRING, &yAxis);
	addProperty(string("ZAxisChannel"), PYINT_STRING, &zAxis);
	addProperty(string("RChannel"), PYINT_STRING, &rChannel);
	addProperty(string("GChannel"), PYINT_STRING, &gChannel);
	addProperty(string("BChannel"), PYINT_STRING, &bChannel);
	addProperty(string("ColourChannel"), PYINT_STRING, &colourChannel);
	addProperty(string("RGBChannel"), PYINT_STRING, &rgbChannel);
	
	addProperty(string("AutoCalculateBounds"), PYINT_INT, &autoBoundMode);
	addProperty(string("MinimumBound"), PYINT_DOUBLE, &minInt);
	addProperty(string("MaximumBound"), PYINT_DOUBLE, &maxInt);
	addProperty(string("StartColourR"), PYINT_INT, &scR);
	addProperty(string("StartColourG"), PYINT_INT, &scG);
	addProperty(string("StartColourB"), PYINT_INT, &scB);
	addProperty(string("EndColourR"), PYINT_INT, &ecR);
	addProperty(string("EndColourG"), PYINT_INT, &ecG);
	addProperty(string("EndColourB"), PYINT_INT, &ecB);
	
	needsApply = true;
	
	// Note that this instance hasn't recieved any messages yet
	firstMessage = 1;
}

// Yanked from rviz
inline int32_t findChannelIndex(const sensor_msgs::PointCloud2ConstPtr& cloud, const std::string& channel)
{
	for (size_t i = 0; i < cloud->fields.size(); ++i)
	{
		if (cloud->fields[i].name == channel)
		{
			return i;
		}
	}

	return -1;
}

void GPointCloud::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
	
	/*
	sourceFrame = Frame::findFrame(cloud->header.frame_id);

	if (sourceFrame < 0)
	{
		return;
	}
	
	if (!availableFrames[sourceFrame].valid)
	{
		if (visible && showable)
		{
			showable = false;
			
			// Detach all the persisting manual objects.
			parent->removeChild(graphicNode);
		}
		
		return;
	}
	*/

	std::string frame_id = scan->header.frame_id;

	// Tolerance??
	// Compute tolerance necessary for this scan
	
	/*
	ros::Duration tolerance(scan->time_increment * scan->ranges.size());
	
	if (tolerance > filterTolerance)
	{
		filterTolerance = tolerance;
		tfFilter->setTolerance(filterTolerance);
	}
	*/

	try
	{
		laserProjector->transformLaserScanToPointCloud(frame_id, *scan, *cloud, *tfListener, -1.0, laser_geometry::channel_option::Intensity);
	}
	catch (tf::TransformException& e)
	{
		ROS_DEBUG("LaserScan [%s]: failed to transform scan: %s.  This message should not repeat (tolerance should now be set on our tf::MessageFilter).", name.c_str(), e.what());
		return;
	}

	// Copy the frame id to the cloud message
	cloud->header.frame_id = scan->header.frame_id;
	
	pcCallback(cloud);
}

void GPointCloud::pcCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	double interpol;
	
	// Filter any nan values out of the cloud.  Any nan values that make it through to PointCloudBase
	// will get their points put off in lala land, but it means they still do get processed/rendered
	// which can be a big performance hit
	//sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);
	
	// We update the manual object directly (speed is everything when these clouds get plentiful)

	sourceFrame = Frame::findFrame(cloud->header.frame_id);

	if (sourceFrame < 0)
	{
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame: " + cloud->header.frame_id);
		
		return;
	}
	
	if (!availableFrames[sourceFrame].valid)
	{
		if (visible && showable)
		{
			showable = false;
			
			// Detach all the persisting manual objects.
			parent->removeChild(graphicNode);
		}
		
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame: " + cloud->header.frame_id);
		
		return;
	}

	// This clause tries to autodetect how to display the point cloud
	if (firstMessage)
	{
		// Look for r, g and b channels
		if (findChannelIndex(cloud, rChannel) >= 0 &&
			findChannelIndex(cloud, gChannel) >= 0 &&
			findChannelIndex(cloud, bChannel) >= 0)
		{
			colourTransform = CT_RGB3;
		} // Is there an rgb channels
		else if (findChannelIndex(cloud, rgbChannel) >= 0)
		{
			colourTransform = CT_RGB1;
		} // What about intensity?
		else if (findChannelIndex(cloud, "intensities") >= 0)
		{
			colourChannel = "intensities";
			colourTransform = CT_CHANNEL;
		}
		else if (findChannelIndex(cloud, "intensity") >= 0)
		{
			colourChannel = "intensity";
			colourTransform = CT_CHANNEL;
		}
		
		firstMessage = 0;
	}

	if ((cloud->width * cloud->height) > 0)
	{
		Ogre::ManualObject * tempManual;
		Ogre::SceneNode * tempNode;
		Ogre::BillboardSet * tempBillboard;
		
		if (billboardType == 0)
		{
			// Create a manual object if there nothing available.
			if (freeManuals.size() == 0)
			{	
				// If this is the first time reaching this point, create a material for the billboards.
				if (manuals.size() == 0)
				{
					materialName = name + "PointMat";
					//Ogre::MaterialManager & materialManager = Ogre::MaterialManager::getSingleton();
					
					material = Ogre::MaterialManager::getSingleton().create(materialName.c_str(), 
						Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
					material->setPointSize(billboardWidth);
					Ogre::Technique* pT = material->getTechnique( 0 );
						if( NULL == pT ) pT = material->createTechnique();
					Ogre::Pass* pP = pT->getPass( 0 );
					if( NULL == pP ) pP = pT->createPass();

					// set parameters
					pP->setVertexColourTracking(Ogre::TVC_AMBIENT);
					pP->setLightingEnabled(false);
					//pP->setDepthCheckEnabled(true);
					pP->setDepthWriteEnabled(false);
					pP->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
				}
				
				tempNode = graphicNode->createChildSceneNode();
				tempManual = scene->createManualObject(name + boost::lexical_cast<string>(elementNo++));
				tempNode->attachObject(tempManual);
					
				tempManual->setDynamic(true);
				tempManual->estimateVertexCount(1000);
				tempManual->begin(materialName.c_str(), Ogre::RenderOperation::OT_POINT_LIST);
				
				manuals.push_back(tempManual);
				nodes.push_back(tempNode);
				creationTimes.push_back(ros::Time::now());
			}
			else
			{
				// Take a free manual object from the freeManuals list
				tempManual = freeManuals[0];
				freeManuals.erase(freeManuals.begin());
				
				tempNode = freeNodes[0];
				freeNodes.erase(freeNodes.begin());
				
				nodes.push_back(tempNode);
				manuals.push_back(tempManual);
				creationTimes.push_back(ros::Time::now());
				
				graphicNode->addChild(tempNode);
				
				tempManual->beginUpdate(0);
			}
			
			// Overall pointcloud colour
			if (colourTransform == CT_FLAT)
			{
				tempManual->colour(scR / 255.0, scG / 255.0, scB / 255.0, alpha);
			}
			else
			{
				tempManual->colour(1.0, 1.0, 1.0, alpha);
			}
		}
		else // billboardType is non zero
		{
			
			// Create a manual object if there nothing available.
			if (freeBillboards.size() == 0)
			{	
				tempNode = graphicNode->createChildSceneNode();
				tempBillboard = scene->createBillboardSet(name + boost::lexical_cast<string>(elementNo++));
				tempNode->attachObject(tempBillboard);
				
				billboards.push_back(tempBillboard);
				nodes.push_back(tempNode);
				creationTimes.push_back(ros::Time::now());
			}
			else
			{
				// Take a free manual object from the freeManuals list
				tempBillboard = freeBillboards[0];
				freeBillboards.erase(freeBillboards.begin());
				
				tempNode = freeNodes[0];
				freeNodes.erase(freeNodes.begin());
				
				nodes.push_back(tempNode);
				billboards.push_back(tempBillboard);
				creationTimes.push_back(ros::Time::now());
				
				graphicNode->addChild(tempNode);
			}
			
			// Configure the billboard texture based on the user's choice
			switch (billboardType)
			{
				case BT_DIAMOND:
					tempBillboard->setMaterialName("PointCloud/Diamond");
					break;
					
				case BT_CIRCLE:
					tempBillboard->setMaterialName("PointCloud/Circle");
					break;
					
				case BT_CROSS:
					tempBillboard->setMaterialName("PointCloud/Cross");
					break;
			}
			tempBillboard->setDefaultDimensions(0.1 * billboardWidth, 0.1 * billboardWidth);
		
			tempBillboard->clear();
		}

		// Find relevant channels

		int32_t xi = findChannelIndex(cloud, xAxis);
		int32_t yi = findChannelIndex(cloud, yAxis);
		int32_t zi = findChannelIndex(cloud, zAxis);
		
		int32_t rgbi;
		int32_t ri, gi, bi;
		uint32_t roff = 0;
		uint32_t goff = 0;
		uint32_t boff = 0;
		uint32_t coff = 0;
		
		if (colourTransform == CT_RGB1)
		{
			 rgbi = findChannelIndex(cloud, colourChannel);
			 
			 if (rgbi == -1)
			 {
				 colourTransform = CT_NONE;
				 publishStatus(Gadget::WARNING, "Single RGB channel not found, setting the colour transform to flat");
			 }
			 else
			 {
				 coff = cloud->fields[rgbi].offset;
			 }
		}
		else if (colourTransform == CT_RGB3)
		{
			ri = findChannelIndex(cloud, rChannel);
			gi = findChannelIndex(cloud, gChannel);
			bi = findChannelIndex(cloud, bChannel);
			
			if (ri == -1 || gi == -1 || bi == -1)
			{
				colourTransform = CT_NONE;
				publishStatus(Gadget::WARNING, "R, G and B channels not found, setting the colour transform to flat");
			}
			else
			{
				roff = cloud->fields[ri].offset;
				goff = cloud->fields[gi].offset;
				boff = cloud->fields[bi].offset;
			}
		}
		
		// If we're depending on channel information for point colour, find our 'intensity' channel
		int32_t ci = -1;
		if (colourTransform == CT_CHANNEL)
		{
			ci = findChannelIndex(cloud, colourChannel);
			if (ci < 0)
			{
				colourTransform = CT_NONE;
				publishStatus(Gadget::WARNING, "R, G and B channels not found, setting the colour transform to flat");
			}
			else
			{
				coff = cloud->fields[ci].offset;
			}
		}

		if (xi == -1 || yi == -1 || zi == -1)
		{
			// TODO : No points may crash the system?
			if (billboardType == 0)
			{
				tempManual->end();
			}
			
			// TODO : Need to specify which
			publishStatus(Gadget::ERROR, "Some or all of the channels used to locate the points are not available");
			
			return;
		}

		// Get the point data offsets
		const uint32_t xoff = cloud->fields[xi].offset;
		const uint32_t yoff = cloud->fields[yi].offset;
		const uint32_t zoff = cloud->fields[zi].offset;
		const uint32_t point_step = cloud->point_step;
		const size_t point_count = cloud->width * cloud->height;

		// If we're discovering boundaries for the colour transform automatically, find them
		uint8_t* ptr = (uint8_t *)&cloud->data.front();
		
		// If we haven't seen data yet and we are expected to determined the data bounds automatically
		// , reset the bounds
		if (unknownBounds && autoBoundMode && point_count > 0)
		{
			float c = *reinterpret_cast<const float*>(ptr + coff);
			
			minInt = c;
			maxInt = c;
			unknownBounds = 0;
		}
		
		if (autoBoundMode)
		{
			for (size_t i = 0; i < point_count; ++i)
			{
				float c = *reinterpret_cast<const float*>(ptr + coff);

				if (c < minInt)
				{
					minInt = c;
				}
				
				if (c > maxInt)
				{
					maxInt = c;
				}
				
				ptr += point_step;
			}	
		}

		double interval = maxInt - minInt;
		
		Ogre::Billboard * billboardElement;

		uint32_t output_count = 0;
		ptr = (uint8_t *)&cloud->data.front();
		for (size_t i = 0; i < point_count; ++i)
		{
			float x = *reinterpret_cast<const float*>(ptr + xoff);
			float y = *reinterpret_cast<const float*>(ptr + yoff);
			float z = *reinterpret_cast<const float*>(ptr + zoff);
			float c;
			
			if (validateFloats(Ogre::Vector3(x, y, z)))
			{	
				if (billboardType == 0)
				{
					// Specify point location first, and then its colour property
					tempManual->position(x, y, z);	
				}
				else
				{
					billboardElement = tempBillboard->createBillboard(Ogre::Vector3(x, y, z));	
				}
				
				if (billboardType > 0)
				{
					switch (colourTransform)
					{
						case CT_FLAT:
							{
								billboardElement->setColour(
									Ogre::ColourValue(scR / 255.0, scG / 255.0, scB / 255.0, alpha));
								break;
							}
	
						case CT_NONE:
							{
								billboardElement->setColour(Ogre::ColourValue(1.0, 1.0, 1.0, alpha));
								break;
							}
							
						case CT_CHANNEL:
							{
								c = *reinterpret_cast<const float*>(ptr + coff);
								
								interpol = (c - minInt) / interval;
								
								billboardElement->setColour(
									Ogre::ColourValue((interpol * ecR + (1 - interpol) * scR) / 255.0, 
									(interpol * ecG + (1 - interpol) * scG) / 255.0, 
									(interpol * ecB + (1 - interpol) * scB) / 255.0, alpha));
								break;
							} 
							
						case CT_RGB1:
							{
								int rgb = *reinterpret_cast<int*>(ptr + coff);
								double r, g, b;
								r = (rgb & 0xff0000 >> 32)  / 255.0;
								g = (rgb & 0x00ff00 >> 16)  / 255.0;
								b = (rgb & 0x0000ff)  / 255.0;
								
								billboardElement->setColour(
									Ogre::ColourValue(r, 
										g, 
										b, alpha));
										
								break;
							}
						case CT_RGB3:
							{
								float r = *reinterpret_cast<const float*>(ptr + roff);
								float g = *reinterpret_cast<const float*>(ptr + goff);
								float b = *reinterpret_cast<const float*>(ptr + boff);
								
								billboardElement->setColour(
									Ogre::ColourValue(r, 
										g, 
										b, alpha));
								
								break;
							}
					}
				} // billboardType == 0
				else
				{
					switch (colourTransform)
					{
						case CT_FLAT:
							{
								tempManual->colour(Ogre::ColourValue(scR / 255.0, scG / 255.0, scB / 255.0, alpha));
								break;
							}
	
						case CT_NONE:
							{
								tempManual->colour(Ogre::ColourValue(1.0, 1.0, 1.0, alpha));
								break;
							}
							
						case CT_CHANNEL:
							{
								c = *reinterpret_cast<const float*>(ptr + coff);
								
								interpol = (c - minInt) / interval;
								
								tempManual->colour(
									Ogre::ColourValue((interpol * ecR + (1 - interpol) * scR) / 255.0, 
									(interpol * ecG + (1 - interpol) * scG) / 255.0, 
									(interpol * ecB + (1 - interpol) * scB) / 255.0, alpha));
								break;
							} 
							
						case CT_RGB1:
							{
								int rgb = *reinterpret_cast<int*>(ptr + coff);
								double r, g, b;
								r = (rgb & 0xff0000 >> 32)  / 255.0;
								g = (rgb & 0x00ff00 >> 16)  / 255.0;
								b = (rgb & 0x0000ff)  / 255.0;
								
								tempManual->colour(
									Ogre::ColourValue(r, 
										g, 
										b, alpha));
										
								break;
							}
						case CT_RGB3:
							{
								float r = *reinterpret_cast<const float*>(ptr + roff);
								float g = *reinterpret_cast<const float*>(ptr + goff);
								float b = *reinterpret_cast<const float*>(ptr + boff);
								
								tempManual->colour(
									Ogre::ColourValue(r, 
										g, 
										b, alpha));
								
								break;
							}
					}
				} 	
				
				++output_count;
				
			}

			ptr += point_step;
		}
		
		if (billboardType == 0)
		{
			tempManual->end();
		}
		
		// Set the scene object frame according to TF lastest information
		tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
		tf::Vector3 & pos = tran.getOrigin();
		tf::Quaternion rot = tran.getRotation();
		
		tempNode->setPosition(pos.x(), pos.y(), pos.z());
		tempNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
		
		if (visible && !showable)
		{
			// Attach all the persisting manual objects.
			parent->addChild(graphicNode);

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

void GPointCloud::subscribePointCloud2(string _topic)
{
	topic = _topic;
	type = Gadget::GADGET_POINTCLOUD;
	pcSub = master->n->subscribe<sensor_msgs::PointCloud2> (_topic.c_str(), 1, &GPointCloud::pcCallback, this);
}

void GPointCloud::subscribeLaserScan(string _topic)
{
	topic = _topic;
	type = Gadget::GADGET_LASERSCAN;
	//lsSub.subscribe(*master->n, _topic.c_str(), 2);
	lsSub = master->n->subscribe<sensor_msgs::LaserScan> (_topic.c_str(), 1, &GPointCloud::laserCallback, this);

	//tfFilter = new tf::MessageFilter<sensor_msgs::LaserScan>(*tfListener, "", 10, *master->n);
	
	laserProjector = new laser_geometry::LaserProjection();

	//tfFilter->connectInput(lsSub);
	//tfFilter->registerCallback(boost::bind(&GPointCloud::laserCallback, this, _1));
}


void GPointCloud::update()
{
	int changes;
	
	changes = 1;
	
	while (changes)
	{
		changes = 0;
		if (creationTimes.size() == 0)
		{
			break;
		}
		
		if ((creationTimes[0] - (ros::Time::now() - ros::Duration(persistence))).toSec() < 0)
		{
			changes = 1;
			creationTimes.erase(creationTimes.begin());
			
			freeNodes.push_back(nodes[0]);
			graphicNode->removeChild(nodes[0]);
			nodes.erase(nodes.begin());
			
			if (billboardType > 0)
			{
				freeBillboards.push_back(billboards[0]);
				billboards.erase(billboards.begin());
			}
			else
			{
				freeManuals.push_back(manuals[0]);
				manuals.erase(manuals.begin());
			}
		}
	}
	
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
			
			// Detach all the persisting clouds.
			parent->removeChild(graphicNode);
		}
		
		publishStatus(Gadget::ERROR, "No transform from fixed frame to message frame");
		
		return;
	}
}

void GPointCloud::show(bool value)
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

void GPointCloud::cleanupData()
{
	vector<Ogre::ManualObject *>::iterator manualIt;
	vector<Ogre::SceneNode *>::iterator nodeIt;
	vector<Ogre::BillboardSet *>::iterator billIt;
	
	// Cleanup all the point cloud objects
	for (billIt = billboards.begin(); billIt != billboards.end(); billIt++)
	{
		scene->destroyBillboardSet(*billIt);
	}
	billboards.clear();
	
	for (billIt = freeBillboards.begin(); billIt != freeBillboards.end(); billIt++)
	{
		scene->destroyBillboardSet(*billIt);
	}
	freeBillboards.clear();
	
	for (manualIt = manuals.begin(); manualIt != manuals.end(); manualIt++)
	{
		scene->destroyManualObject(*manualIt);
	}
	manuals.clear();
	
	for (manualIt = freeManuals.begin(); manualIt != freeManuals.end(); manualIt++)
	{
		scene->destroyManualObject(*manualIt);
	}
	freeManuals.clear();
	
	// ... all their nodes
	for (nodeIt = nodes.begin(); nodeIt != nodes.end(); nodeIt++)
	{
		scene->destroySceneNode(*nodeIt);
	}
	nodes.clear();
	
	for (nodeIt = freeNodes.begin(); nodeIt != freeNodes.end(); nodeIt++)
	{
		scene->destroySceneNode(*nodeIt);
	}
	freeNodes.clear();
	
	creationTimes.clear();
}

void GPointCloud::applyProperties()
{
	// These changes also need to be in the callback in case data comes along much later
	// If there is a manual object change the group material properties
	
	// If we have apparently changed billboard type, then cleanup for the next render passes
	if (billboardType == 0 && (billboards.size() > 0 || freeBillboards.size() > 0))
	{
		cleanupData();
	}
	if (billboardType > 0 && (manuals.size() > 0 || freeManuals.size() > 0))
	{
		cleanupData();
	}
	
	if (manuals.size() > 0)
	{	
		if (billboardType == 0)
		{
			material->setPointSize(billboardWidth);
		}
	}
}

GPointCloud::~GPointCloud()
{
	cleanupData();
	
	// and the main node
	if (graphicNode)
	{
		scene->destroySceneNode(graphicNode);
	}
	
	/*
	if (tfFilter)
	{
		tfFilter->clear();
		delete laserProjector;
		delete tfFilter;
	}
	*/
}
