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

#ifndef __PYBLOCK
#define __PYBLOCK

#include <ros/ros.h>
#include <vector>
#include <string.h>
#include <string>
#include <exception>

using namespace std;

#include <boost/thread.hpp>

#include <boost/python.hpp>
#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/list.hpp>
#include <boost/python/iterator.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <boost/lexical_cast.hpp>

#include "CRIExceptions.h"
#include <boost/python/exception_translator.hpp>

typedef vector<string> stringVector;

#define PYINT_INT 0
#define PYINT_DOUBLE 1
#define PYINT_STRING 2

class UnknownPropertyException : public std::exception
{
public:
	string propertyName;
	string childName;
	
	UnknownPropertyException(string name)
	{
		propertyName = name;
	}
	
	UnknownPropertyException(string name, string child)
	{
		propertyName = name;
		childName = child;
	}
	
	~UnknownPropertyException() throw()
	{
	};
	
	const char * what() const throw() 
	{ 
		if (childName == "")
		{
			return ("Unknown CRI property '" + propertyName + "'.").c_str();
		}
		else
		{
			return ("Unknown child '" + childName + "' property '" + propertyName + "'.").c_str();
		}
	}
};

class UnknownChildException : public std::exception
{
public:
	string childName;
	
	UnknownChildException(string name)
	{
		childName = name;
	}
	
	~UnknownChildException () throw()
	{
	};
	
	const char * what() const throw() 
	{ 
		return ("Unknown child '" + childName + "'.").c_str();
	}
};

class UnknownTypeException : public std::exception
{
public:
	string typeString;
	
	UnknownTypeException(int number)
	{
		typeString = boost::lexical_cast<string>(number);
	}
	
	~UnknownTypeException () throw()
	{
	};
	
	const char * what() const throw() 
	{ 
		return ("Unknown type '" + typeString + "'.").c_str();
	}
};


class UnknownCallbackException : public std::exception
{
public:
	string propertyName;
	string childName;
	
	UnknownCallbackException(string name)
	{
		propertyName = name;
	}
	
	UnknownCallbackException(string name, string child)
	{
		propertyName = name;
		childName = child;
	}
	
	~UnknownCallbackException() throw()
	{
	};
	
	const char * what() const throw() 
	{ 
		if (childName == "")
		{
			return ("Unknown CRI callback '" + propertyName + "'.").c_str();
		}
		else
		{
			return ("Unknown child '" + childName + "' callback '" + propertyName + "'.").c_str();
		}
	}
};

class PyInterface
{
	public:
	 
	string intName;
	
	boost::mutex * generalMut;
	bool needsApply;

	vector<string> properties;
	vector<int> types;
	vector<void *> dataPtrs;

	// For the Python interface
	virtual stringVector listChildren();
	virtual stringVector listProperties();
	virtual stringVector listChildProperties(string childName);
	virtual stringVector listCallbacks();
	virtual stringVector listChildCallbacks(string childName);
	
	virtual void set(string name, string value);
	virtual void setChild(string childName, string name, string value);
	virtual string get(string name);
	virtual string getChild(string childName, string name);
	
	
	// Memory safe (in case someone kills it) and thread safe execution functions
	virtual void execCB(string name);
	virtual void execChild(string childName, string name);
	//virtual void execShortcut(int shortcut);
	
	//virtual int getShortcut(string name);
	//virtual int getChildShortcut(string childName, string name);
	virtual void applyProperties();

	// For the C++ interface
	virtual void addProperty(string name, int type, void * data);
	
	// Never a need to do this
	//virtual void removeProperty(string name);
	//virtual void clearProperties();

	// For callbacks (TODO)
	
	virtual void addCallback(string name, void (* callback)(void *), void * data);
	
	// Never a need to do this
	// virtual void removeCallback(string name);
	
	// Child interfaces
	virtual void addInterface(PyInterface * item);
	virtual void removeInterface(PyInterface * item);
	int findChild(string & childname);

	PyInterface() {intName = string("dummy"); needsApply = false; generalMut = new boost::mutex();};
	PyInterface(string _name) {intName = _name; needsApply = false; generalMut = new boost::mutex();};
	~PyInterface() {delete generalMut;};
	
	
	private:
	
	vector<PyInterface *> interfaces;
	vector<void (*)(void *)> callbacks;
	vector<string> callbackNames;
	vector<void *> callbackData;
	
	
	// Todo : Property shortcuts
	vector<int> propertyShortcuts;
	vector<int> propertyShortcutChildren;
	vector<void (*)(void *)> callbackShortcuts;
	vector<void *> callbackSCData;
	vector<int> callbackShortcutChild;
};	

typedef boost::shared_ptr<PyInterface> SharedPyInterface;

class PyBlock
{
	public:
	
	boost::thread * execBlock;
	boost::condition_variable codeReadyCond;
	boost::mutex codeReadyMut;
	
	char * code;
	PyInterface * pyInterface;
	
	bool codeReady, stillAlive;
	
	PyBlock();
	//PyBlock(const boost::python::object & _main_module, const boost::python::object & _main_namespace);

	~PyBlock();
	
	int execute(const char * _filename);
	int execute(const char * _filename, PyInterface * _pyInterface);
	void executeString(const char * codeBlock);
	void executeString(const char * codeBlock, PyInterface * _pyInterface);
	void kill();
	void waitForExecution();
	
	private:
	
	void executionThread(int value);
};

std::string parse_python_exception();
void registerPythonModule();
extern PyThreadState * mainThreadState;

extern boost::python::object main_module; // Python instance
extern boost::python::object main_namespace; // Instance dictionary

#endif
