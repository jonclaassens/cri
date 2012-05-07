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

#include <sstream>
#include <iomanip>

void GReticle::specifyAxes()
{
	double xx;
	
	// If there is no axesManual object, create one 
	if (axesManual == NULL)
	{
		axesManual = scene->createManualObject(name + "_axesManual");
		
		axesManual->setDynamic(true);
		axesManual->estimateVertexCount(1000);
		axesManual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
		
		axesNode->attachObject(axesManual);
	}
	else //Otherwise, modify the current one
	{
		axesManual->beginUpdate(0);
	}
	
	axesManual->position(0, 0, relativePos[2]);
	axesManual->colour(0, 1, 0);
	axesManual->position(0, relativePos[1], relativePos[2]);
	
	axesManual->position(0, 0, relativePos[2]);
	axesManual->position(relativePos[0], 0, relativePos[2]);
	
	axesManual->position(0, relativePos[1], 0);
	axesManual->position(0, relativePos[1], relativePos[2]);
	
	axesManual->position(0, relativePos[1], 0);
	axesManual->position(relativePos[0], relativePos[1], 0);
	
	axesManual->position(relativePos[0], 0, 0);
	axesManual->position(relativePos[0], relativePos[1], 0);
	
	axesManual->position(relativePos[0], 0, 0);
	axesManual->position(relativePos[0], 0, relativePos[2]);
	
	axesManual->position(relativePos[0], relativePos[1], 0);
	axesManual->position(relativePos[0], relativePos[1], relativePos[2]);
	
	axesManual->position(relativePos[0], 0, relativePos[2]);
	axesManual->position(relativePos[0], relativePos[1], relativePos[2]);
	
	axesManual->position(0, relativePos[1], relativePos[2]);
	axesManual->position(relativePos[0], relativePos[1], relativePos[2]);
	
	//tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
	//tf::Vector3 & pos = tran.getOrigin();
	//tf::Quaternion rot = tran.getRotation();
	
	double sgnVal = (relativePos[0] < 0) * -1 + (relativePos[0] > 0);
	
	// X axis
	int count = 0;
	for (xx = 0; xx < (fabs(relativePos[0]) - axesXTic); xx += axesXTic)
	{
		axesManual->position(sgnVal * (xx + axesXTic), 0, 0);
		if (count == bigNotchCount)
		{
			axesManual->position(sgnVal * (xx + axesXTic), 0, sgnVal * notchHeight * 3);
		}
		else
		{
			axesManual->position(sgnVal * (xx + axesXTic), 0, sgnVal * notchHeight);
		}
		
		axesManual->position(sgnVal * (xx + axesXTic), 0, 0);
		if (count++ == bigNotchCount)
		{
			axesManual->position(sgnVal * (xx + axesXTic), sgnVal * notchHeight * 3, 0);
			count = 0;
		}
		else
		{
			axesManual->position(sgnVal * (xx + axesXTic), sgnVal * notchHeight, 0);
		}
	}
	
	axesManual->position(0, 0, 0);
	axesManual->position(relativePos[0], 0, 0);
	
	
	sgnVal = (relativePos[1] < 0) * -1 + (relativePos[1] > 0);
	
	// Y axis
	count = 0;
	for (xx = 0; xx < (fabs(relativePos[1]) - axesYTic); xx += axesYTic)
	{
		axesManual->position(0, sgnVal * (xx), 0);
		axesManual->position(0, sgnVal * (xx + axesYTic), 0);
		
		axesManual->position(0, sgnVal * (xx + axesYTic), 0);
		if (count == bigNotchCount)
		{
			axesManual->position(0, sgnVal * (xx + axesYTic), sgnVal * notchHeight * 3);
		}
		else
		{
			axesManual->position(0, sgnVal * (xx + axesYTic), sgnVal * notchHeight);
		}
		
		axesManual->position(0, sgnVal * (xx + axesYTic), 0);
		if (count++ == bigNotchCount)
		{
			axesManual->position(sgnVal * notchHeight * 3, sgnVal * (xx + axesYTic), 0);
			count = 0;
		}
		else
		{
			axesManual->position(sgnVal * notchHeight, sgnVal * (xx + axesYTic), 0);
		}
	}
	
	sgnVal = (relativePos[2] < 0) * -1 + (relativePos[2] > 0);
	
	axesManual->position(0, 0, 0);
	axesManual->position(0, relativePos[1], 0);
	
	
	// Z axis
	count = 0;
	for (xx = 0; xx < (fabs(relativePos[2]) - axesZTic); xx += axesZTic)
	{	
		axesManual->position(0, 0, sgnVal * (xx));
		axesManual->position(0, 0, sgnVal * (xx + axesZTic));
		
		axesManual->position(0, 0, sgnVal * (xx + axesZTic));
		if (count == bigNotchCount)
		{
			axesManual->position(sgnVal * notchHeight * 3, 0, sgnVal * (xx + axesZTic));
		}
		else
		{
			axesManual->position(sgnVal * notchHeight, 0, sgnVal * (xx + axesZTic));
		}
		
		axesManual->position(0, 0, sgnVal * (xx + axesZTic));
		if (count++ == bigNotchCount)
		{
			axesManual->position(0, sgnVal * notchHeight * 3, sgnVal * (xx + axesZTic));
			count = 0;
		}
		else
		{
			axesManual->position(0, sgnVal * notchHeight, sgnVal * (xx + axesZTic));
		}
	}
	
	axesManual->position(0, 0, 0);
	axesManual->position(0, 0, relativePos[2]);
	
	axesManual->end();
	
	// If we must abandon all of our preallocated data
	if (!xText)
	{
		xText = new Ogre::MovableText(name + "xLabel", ".");
		yText = new Ogre::MovableText(name + "yLabel", ".");
		zText = new Ogre::MovableText(name + "zLabel", ".");
		
		// Attach text to scene node	
		xTextNode->attachObject(xText);	
		yTextNode->attachObject(yText);	
		zTextNode->attachObject(zText);
	}
	
	// Update movable text properties
	std::ostringstream ssx;
    ssx << "x = " << std::fixed << std::setprecision(2);
    ssx << relativePos[0];
	
	xText->setCaption(ssx.str());
	xText->setColor(Ogre::ColourValue(0, 1, 0, 1));
	xTextNode->setPosition(Ogre::Vector3(relativePos[0] / 2.0, 
		0, 0));
	xText->setCharacterHeight(0.2);
	
	std::ostringstream ssy;
    ssy << "y = " << std::fixed << std::setprecision(2);
    ssy << relativePos[1];
	
	yText->setCaption(ssy.str());
	yText->setColor(Ogre::ColourValue(0, 1, 0, 1));
	yTextNode->setPosition(Ogre::Vector3(0, relativePos[1] / 2.0, 0));		
	yText->setCharacterHeight(0.2);

	std::ostringstream ssz;
    ssz << "z = " << std::fixed << std::setprecision(2);
    ssz << relativePos[2];
	
	zText->setCaption(ssz.str());
	zText->setColor(Ogre::ColourValue(0, 1, 0, 1));
	zTextNode->setPosition(Ogre::Vector3(0, 0, relativePos[2] / 2.0));
	zText->setCharacterHeight(0.2);
	
}
