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

#include "PyBlock.h"

// TODO :  There are potential thread issues with the code pointer in PyBlocks

PyThreadState * mainThreadState;
boost::python::object main_module; // Python instance
boost::python::object main_namespace; // Instance dictionary

//// Global for the Python Portal interface
//boost::shared_ptr<PyPortalInterface> sharedPyInterface;


void translatePropertyException(UnknownPropertyException const& e)
{
	char buffer[256];
	const char * res = e.what();
	
	strncpy(buffer, res, 255);
		
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, buffer);
    //PyErr_SetObject(myCPPExceptionType, boost::python::object(e).ptr());
};

void translateCallbackException(UnknownCallbackException const& e)
{
	char buffer[256];
	const char * res = e.what();
	
	strncpy(buffer, res, 255);
		
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, buffer);
    //PyErr_SetObject(myCPPExceptionType, boost::python::object(e).ptr());
};

void translateChildException(UnknownChildException const& e)
{
	char buffer[256];
	const char * res = e.what();
	
	strncpy(buffer, res, 255);
	
    // Use the Python 'C' API to set up an exception object
	PyErr_SetString(PyExc_RuntimeError, buffer);  //std::string(e.what()));
	//PyErr_SetObject(myCPPExceptionType, boost::python::object(e).ptr());
};

void translateTypeException(UnknownTypeException const& e)
{
	char buffer[256];
	const char * res = e.what();
	
	strncpy(buffer, res, 255);
	
    // Use the Python 'C' API to set up an exception object
	PyErr_SetString(PyExc_RuntimeError, buffer);  //std::string(e.what()));
	//PyErr_SetObject(myCPPExceptionType, boost::python::object(e).ptr());
};

stringVector PyInterface::listChildren()
{
	stringVector result;
	vector<PyInterface *>::iterator it;
	
	for (it = interfaces.begin(); it != interfaces.end(); it++)
	{
		result.push_back((*it)->intName);
	}
	
	return result;
}

stringVector PyInterface::listProperties()
{
	return properties;
}

stringVector PyInterface::listChildProperties(string childName)
{
	int childNum;
	
	childNum = findChild(childName);
	
	if (childNum < 0)
	{
		throw UnknownChildException(childName);
	}
	
	return interfaces[childNum]->listProperties();
}

stringVector PyInterface::listCallbacks()
{
	return callbackNames; 
}

stringVector PyInterface::listChildCallbacks(string childName)
{
	int childNum;
	
	childNum = findChild(childName);
	
	if (childNum < 0)
	{
		throw UnknownChildException(childName);
	}
	
	return interfaces[childNum]->listCallbacks();
}

void PyInterface::set(string name, string value)
{
	int iter;
	
	for (iter = 0; iter < properties.size(); iter++)
	{
		if (properties[iter] == name)
		{
			generalMut->lock();
			
			switch (types[iter])
			{
				case PYINT_INT:
					*((int *)dataPtrs[iter]) = atoi(value.c_str());
					break;
					
				case PYINT_DOUBLE:
					*((double *)dataPtrs[iter]) = atof(value.c_str());
					break;
					
				case PYINT_STRING:
					*((string *)dataPtrs[iter]) = value;
					break;
			}
			
			needsApply = true;
			
			generalMut->unlock();
			return;
		}
	}
	
	throw UnknownPropertyException(name);
}

void PyInterface::setChild(string childName, string name, string value)
{
	int childNum;
	
	childNum = findChild(childName);
	
	if (childNum < 0)
	{
		throw UnknownChildException(childName);
	}
	
	interfaces[childNum]->set(name, value);
}

string PyInterface::get(string name)
{
	int iter;
	
	for (iter = 0; iter < properties.size(); iter++)
	{
		if (properties[iter] == name)
		{
			switch (types[iter])
			{
				case PYINT_INT:
					return boost::lexical_cast<std::string>(*((int *)dataPtrs[iter]));
					
				case PYINT_DOUBLE:
					return boost::lexical_cast<std::string>(*((double *)dataPtrs[iter]));
					
				case PYINT_STRING:
					return *((string *)dataPtrs[iter]);
					
			}
		}
	}
	
	throw UnknownPropertyException(name);
}

string PyInterface::getChild(string childName, string name)
{
	int childNum;
	
	childNum = findChild(childName);
	
	if (childNum < 0)
	{
		throw UnknownChildException(childName);
	}
	
	return interfaces[childNum]->get(name);
}


void PyInterface::execCB(string name)
{
	// Todo : Thread safe???
	
	generalMut->lock();
	
	int idx, found = 0;
	
	for (idx = 0; idx < callbackNames.size(); idx++)
	{
		if (callbackNames[idx] == name)
		{
			callbacks[idx](callbackData[idx]);
			
			found = 1;
			
			break;
		}
	}
	
	generalMut->unlock();
	
	if (!found)
	{
		throw UnknownCallbackException(name);
	}
}

void PyInterface::execChild(string childName, string name)
{
	int childNum;
	
	childNum = findChild(childName);
	
	if (childNum < 0)
	{
		throw UnknownChildException(childName);
	}
	
	return interfaces[childNum]->execCB(name);
}

/*
 * Shortcut functionality is a work in progress
 
void PyInterface::execShortcut(int shortcut)
{
	// Todo : threadsafe ?
	
	generalMut->lock();
	
	if (shortcut >= 0 && shortcut < callbackShortcuts.size())
	{
		if (callbackShortcuts[shortcut] != NULL)
		{
			callbackShortcuts[shortcut](callbackSCData[shortcut]);
		}
	}
	
	generalMut->unlock();
}
	
int PyInterface::getShortcut(string name)
{
	int idx;
	
	for (idx = 0; idx < callbackNames.size(); idx++)
	{
		if (callbackNames[idx] == name)
		{
			callbackShortcuts.push_back(callbacks[idx]);
			callbackShortcutChild.push_back(-1);
			callbackSCData.push_back(callbackData[idx]);
			
			return callbackShortcuts.size() - 1;
		}
	}
}

int PyInterface::getChildShortcut(string childName, string name)
{
	int idx;
	int childNum;
	
	childNum = findChild(childName);
	
	for (idx = 0; idx < interfaces[childNum]->callbackNames.size(); idx++)
	{
		if (interfaces[childNum]->callbackNames[idx] == name)
		{
			callbackShortcuts.push_back(interfaces[childNum]->callbacks[idx]);
			callbackShortcutChild.push_back(childNum);
			
			return callbackShortcuts.size() - 1;
		}
	}
}
*/

void PyInterface::addProperty(string name, int type, void * data)
{
	if (type < 0 || type > 2)
	{
		throw UnknownTypeException(type);
	}	
	
	properties.push_back(name);
	types.push_back(type);
	dataPtrs.push_back(data);
}

//void PyInterface::removeProperty(string name)
//{
//	int iter;
	
//	for (iter = 0; iter < properties.size(); iter++)
//	{
//		if (properties[iter] == name)
//		{
//			properties.erase(properties.begin() + iter);
//			types.erase(types.begin() + iter);
//			dataPtrs.erase(dataPtrs.begin() + iter);
			
//			return;
//		}
//	}
//}

// void PyInterface::clearProperties()
// {
//	properties.clear();
//	types.clear();
//	dataPtrs.clear();
// }

void PyInterface::applyProperties()
{
	// Dummy
}


void PyInterface::addCallback(string name, void (* callback)(void *), void * data)
{
	if (!callback)
	{
		return;
	}
	
	callbacks.push_back(callback);
	callbackNames.push_back(name);
	callbackData.push_back(data);
}

//void PyInterface::removeCallback(string name)
//{
//}


void PyInterface::addInterface(PyInterface * item)
{
	interfaces.push_back(item);
}

void PyInterface::removeInterface(PyInterface * item)
{
	int idx, idx2;
	
	for (idx = 0; idx < interfaces.size(); idx++)
	{
		if (interfaces[idx] == item)
		{
			interfaces.erase(interfaces.begin() + idx);
			
			// Clear all the callback shortcuts
			for (idx2 = 0; idx2 < callbackShortcuts.size(); idx2++)
			{
				if (callbackShortcutChild[idx2] == idx)
				{
					callbackShortcuts[idx2] = NULL;
					callbackShortcutChild[idx2] = -1;
				}
			}
			
			// Clear all the property shortcuts
			
			return;
		}
	}
}

int PyInterface::findChild(string & childname)
{
	int idx;
	
	for (idx = 0; idx < interfaces.size(); idx++)
	{
		if (childname == interfaces[idx]->intName)
		{
			return idx;
		}
	}
	
	return -1;
}


BOOST_PYTHON_MODULE(PyModPyPortalInt)
{
	boost::python::register_exception_translator<UnknownCallbackException>(&translateCallbackException);
	boost::python::register_exception_translator<UnknownPropertyException>(&translatePropertyException);
	boost::python::register_exception_translator<UnknownChildException>(&translateChildException);
	boost::python::register_exception_translator<UnknownTypeException>(&translateTypeException);
	
	boost::python::class_<stringVector>("stringVector")
        .def(boost::python::vector_indexing_suite<stringVector>() );
	
    boost::python::class_<PyInterface, SharedPyInterface>("PyInterface")
		.def("listChildren", &PyInterface::listChildren)
        .def("listProperties", &PyInterface::listProperties)
        .def("listChildProperties", &PyInterface::listChildProperties)
        .def("listCallbacks", &PyInterface::listCallbacks)
        .def("listChildCallbacks", &PyInterface::listChildCallbacks)
        
		.def("set", &PyInterface::set)
		.def("setChild", &PyInterface::setChild)
		.def("get", &PyInterface::get)
		.def("getChild", &PyInterface::getChild)
		
		.def("execCB", &PyInterface::execCB)
		.def("execChild", &PyInterface::execChild)
		//.def("execShortcut", &PyInterface::execShortcut)
		
		//.def("getShortcut", &PyInterface::getShortcut)
		//.def("getChildShortcut", &PyInterface::getChildShortcut)
		.def("applyProperties", &PyInterface::applyProperties)
        ;
};


void registerPythonModule()
{
	initPyModPyPortalInt();
}


PyBlock::PyBlock()
{
	code = NULL;
	
	codeReady = false;
	stillAlive = true;
	
	// Start the execution thread
	execBlock = new boost::thread(&PyBlock::executionThread, this, 0);
}

/*
PyBlock::PyBlock(const boost::python::object & _main_module, const boost::python::object & _main_namespace)
{
	code = NULL;
	fileHandle = NULL;
	
	mustRunOutside = false;
	
	main_module = _main_module;
	main_namespace = _main_namespace;
}
*/

PyBlock::~PyBlock()
{
	kill();
}

void PyBlock::kill()
{
	if (stillAlive)
	{
		// Inform the thread to commit suicide
		stillAlive = false;
		if (code)
		{
			delete [] code;
		}
		code = NULL;
		
		boost::lock_guard<boost::mutex> lock(codeReadyMut);
		codeReady = true;
		codeReadyCond.notify_one();
		
		// Wait for the thread to finish
		execBlock->join();
	}
}

int PyBlock::execute(const char * filename)
{
	return execute(filename, NULL);
}

int PyBlock::execute(const char * filename, PyInterface * _pyInterface)
{
	FILE * fileHandle;
	long fSize;
	
	if (code)
	{
		delete [] code;
		code = NULL;
	}
	
	// Check that the file exists
	fileHandle = fopen(filename, "r");
	
	// If file is not found throw an exception
	if (!fileHandle)
	{
		ROS_ERROR("Cannot find python script file %s.", filename);
		return -1;
	}
	
	// May need this later
	//flockfile(fileHandle);
	
	// Allocate a chunk of memory the size of the code text
	fseek(fileHandle, 0L, SEEK_END);
	fSize = ftell(fileHandle);
	
	code = new char[fSize + 1];
	
	// Get the code
	fseek(fileHandle, 0L, SEEK_SET);
	fread(code, 1, fSize, fileHandle);
	
	// Null terminate the code text
	code[fSize] = 0;
	
	// We're done with the file
	fclose(fileHandle);
	
	// And the interface...
	pyInterface = _pyInterface;
	
	// Tell the execution that we're ready
	{
        boost::lock_guard<boost::mutex> lock(codeReadyMut);
        codeReady = true;
    }
    
    // Tell execution thread to go ahead
    codeReadyCond.notify_one();
    
    return 0;
}

void PyBlock::executeString(const char * codeBlock)
{
	executeString(codeBlock, NULL);
}

void PyBlock::executeString(const char * codeBlock, PyInterface * _pyInterface)
{	
	if (code)
	{
		delete [] code;
		code = NULL;
	}
	
	// Copy the code block
	code = strdup(codeBlock);
	
	// And the interface...
	pyInterface = _pyInterface;
	
	// Tell the execution that we're ready
	{
        boost::lock_guard<boost::mutex> lock(codeReadyMut);
        codeReady = true;
    }
    
    // Tell execution thread to go ahead
    codeReadyCond.notify_one();
}

void PyBlock::executionThread(int value)
{
	// Get the global lock
	PyEval_AcquireLock();
	// Get a reference to the PyInterpreterState
	PyInterpreterState * mainInterpreterState = mainThreadState->interp;
	// Create a thread state object for this thread
	PyThreadState * myThreadState = PyThreadState_New(mainInterpreterState);
	// Free the lock
	PyEval_ReleaseLock();


	while (stillAlive)
	{
		boost::unique_lock<boost::mutex> lock(codeReadyMut);

		while(!codeReady)
		{
			codeReadyCond.wait(lock);
		}
		
		if (code != NULL)
		{
			
			// Grab the global interpreter lock
			PyEval_AcquireLock();
			// Swap in my thread state
			PyThreadState_Swap(myThreadState);			
			
			try
			{	
				// If there is a PyInterface value provided, send it to Python
				if (pyInterface != NULL)
				{
					SharedPyInterface * shared;
					shared = new SharedPyInterface(pyInterface);
					
					char * setupCode = 
						"def setup(PortalInterface):\n" \
						"    print 'setup called with', PortalInterface\n" \
						"    global Portal\n" \
						"    Portal = PortalInterface\n" \
					"\n";
	
					// Pass the reference to 'shared' into python
					boost::python::exec(setupCode, main_namespace);
					boost::python::object setupFunc = main_module.attr("setup");
					
					setupFunc(*shared);
				}
				
				boost::python::exec(code, main_namespace);		
			}
			catch(boost::python::error_already_set const &)
			{
				std::string perror_str = parse_python_exception();
				std::cout << "Error in Python: " << perror_str << std::endl;
			}
			
			// Clear the thread state
			PyThreadState_Swap(NULL);
			// Release our hold on the global interpreter
			PyEval_ReleaseLock();
			
			// Let home know we're done
			codeReady = false;
			codeReadyCond.notify_one();
		}
	}

	// Grab the lock
	PyEval_AcquireLock();
	// Swap my thread state out of the interpreter
	PyThreadState_Swap(NULL);
	// Clear out any cruft from thread state object
	PyThreadState_Clear(myThreadState);
	// Delete my thread state object
	PyThreadState_Delete(myThreadState);
	// Release the lock
	PyEval_ReleaseLock();
	
}

void PyBlock::waitForExecution()
{
	boost::unique_lock<boost::mutex> lock(codeReadyMut);

	while(codeReady)
	{
		codeReadyCond.wait(lock);
	}
}

namespace py = boost::python;

std::string parse_python_exception() 
{
    PyObject *type_ptr = NULL, *value_ptr = NULL, *traceback_ptr = NULL;
    PyErr_Fetch(&type_ptr, &value_ptr, &traceback_ptr);
    std::string ret("Unfetchable Python error");
    if (type_ptr != NULL) {
        py::handle<> h_type(type_ptr);
        py::str type_pstr(h_type);
        py::extract<std::string> e_type_pstr(type_pstr);
        if(e_type_pstr.check())
            ret = e_type_pstr();
        else
            ret = "Unknown exception type";
    }
 
    if (value_ptr != NULL) {
        py::handle<> h_val(value_ptr);
        py::str a(h_val);
        py::extract<std::string> returned(a);
        if(returned.check())
            ret +=  ": " + returned();
        else
            ret += std::string(": Unparseable Python error: ");
    }
 
    if (traceback_ptr != NULL) {
        py::handle<> h_tb(traceback_ptr);
        py::object tb(py::import("traceback"));
        py::object fmt_tb(tb.attr("format_tb"));
        py::object tb_list(fmt_tb(h_tb));
        py::object tb_str(py::str("\n").join(tb_list));
        py::extract<std::string> returned(tb_str);
        if(returned.check())
            ret += ": " + returned();
        else
            ret += std::string(": Unparseable Python traceback");
    }
    return ret;
}
