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
#include <ros/package.h>
#include <string>

#include "ROSTools.h"

using namespace std;

string parseROSFilename(const string & name)
{
	std::string mod_url = name;
	
	if (name.find("package://") == 0)
	{
		mod_url.erase(0, strlen("package://"));
		size_t pos = mod_url.find("/");
		if (pos == std::string::npos)
		{
			return "";
		}

		std::string package = mod_url.substr(0, pos);
		mod_url.erase(0, pos);
		std::string package_path = ros::package::getPath(package);

		if (package_path.empty())
		{
			return "";
		}

		return package_path + mod_url;
	}
	else return name;
}

string getTextFromFile(string filename)
{
	string str;
	unsigned int fSize;
	FILE * fileHandle;

	// TODO : Some inefficiency here with how the string is returned

	// Check that the file exists
	fileHandle = fopen(filename.c_str(), "r");
	
	// If file is not found throw an exception
	if (!fileHandle)
	{
		return "";
	}
	
	// Allocate a chunk of memory the size of the code text
	fseek(fileHandle, 0L, SEEK_END);
	fSize = ftell(fileHandle);
	
	str.resize(fSize);
	
	// Get the code
	fseek(fileHandle, 0L, SEEK_SET);
	fread((char *)str.data(), 1, fSize, fileHandle);
	
	// We're done with the file
	fclose(fileHandle);
	
	return str;
}
				
