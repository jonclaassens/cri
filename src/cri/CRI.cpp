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

#include "CRI.h"

#include <ros/package.h>
#include "OgreUtilities.h"


/**
 * The CRI class constructor initializes Ogre, OIS and the Python interface to the program.
 * It also registers the properties and callbacks that are used to interface with CRI through
 * Python.
 *
 * @param ros::NodeHandle * $_n
 *   An initialized node handle
 *
 */
CRI::CRI(ros::NodeHandle * _n) : PyInterface(string("CRI"))
{
	window = NULL;
	
	// Find the CRI package directory
	packagePath = ros::package::getPath("cri");
	Ogre::String lConfigFileName = "";
	Ogre::String lPluginsFileName = "";
	Ogre::String lLogFileName = (packagePath + "/log/Ogre.log").c_str();
	
	
	// Setup CRI's main Python instance
	Py_Initialize();
	PyEval_InitThreads();
	
	main_module = boost::python::import("__main__");
	main_namespace = main_module.attr("__dict__");
	
	mainThreadState = NULL;
	
	// save a pointer to the main PyThreadState object
	mainThreadState = PyThreadState_Get();
	
	// release the lock
	PyEval_ReleaseLock();
	
	registerPythonModule();
	
	// Parameter properties first
	addProperty(string("parm1"), PYINT_STRING, &parm1);
	addProperty(string("parm2"), PYINT_STRING, &parm2);
	
	// Need to register the ID property (TODO:  Better explanation)
	addProperty(string("MainWindowId"), PYINT_STRING, &mainWindowId);
	
	// Start main pyqt initialisation and QApplication 'spin' scripts
	pyQtInterfaceMain = new PyBlock();
	ROS_INFO("Starting initialization script and waiting for completion...");
	if (pyQtInterfaceMain->execute((packagePath + "/src/python/Initialize.py").c_str(), this) < 0)
	{
		ROS_ERROR("Cannot find pyqt initialization script.  Bailing...");
		return;
	}
	pyQtInterfaceMain->waitForExecution();
	
	ROS_INFO("Giving a thread to PyQT QApplication...");
	if (pyQtInterfaceMain->execute((packagePath + "/src/python/QApplicationLoop.py").c_str()) < 0)
	{
		ROS_ERROR("Cannot find QApplication loop script.  Bailing...");
		return;
	}
	
	ROS_INFO("Waiting on QApplication...");
	// Create python script execution thread for executing interface related scripts
	pyQtInterfaceExec = new PyBlock();
	pyQtInterfaceExec->execute((packagePath + "/src/python/WaitForQApplication.py").c_str());
	
	// Wait for main python thread to have the pyqt QApplication up (If the above script fails
	// to complete, so what, the user will get some annoying messages, but eventually 
	// QApplication will accept messages...)
	pyQtInterfaceExec->waitForExecution();
	
	// Python thread for executing active marker scripts
	pyQtMarkerExec = new PyBlock();
	
	// Register main properties
	// Setup general python interface properties
	// This property is the global size of all GFrame gadgets (the axes)
	frameScale = 0.5;
	addProperty(string("FrameScale"), PYINT_DOUBLE, &frameScale);
	
	// Property for drawing frame trees
	drawFrameTree = 0;
	addProperty(string("DrawFrameTree"), PYINT_INT, &drawFrameTree);
	
	// The origin frame tf name
	addProperty(string("FixedFrame"), PYINT_STRING, &fixedFrame);
	
	// Mouse properties
	addProperty(string("MenuStartX"), PYINT_INT, &menuStartX);
	addProperty(string("MenuStartY"), PYINT_INT, &menuStartY);
	
	// Selection properties
	addProperty(string("ActiveGadgetName"), PYINT_STRING, &activeGadgetName);
	
	// Add general Python interface callbacks for CRI
	
	addCallback(string("Die"), &CRI::die, this);
	addCallback(string("WindowChanged"), &CRI::windowChanged, this);
	addCallback(string("SetPickOn"), &CRI::setPickOn, this);
	addCallback(string("GetFrameParent"), &CRI::getFrameParent, this);
	addCallback(string("AddTopic"), &CRI::addTopic, this);
	addCallback(string("AddReticle"), &CRI::addReticle, this);
	addCallback(string("AddGrid"), &CRI::addGrid, this);
	addCallback(string("DeleteGadget"), &CRI::deleteGadget, this);
	addCallback(string("GetKeyPressedAssignment"), &CRI::getKeyPressedAssignment, this);
	addCallback(string("SetKeyPressedAssignment"), &CRI::setKeyPressedAssignment, this);
	addCallback(string("GetKeyReleasedAssignment"), &CRI::getKeyReleasedAssignment, this);
	addCallback(string("SetKeyReleasedAssignment"), &CRI::setKeyReleasedAssignment, this);
	addCallback(string("getMousePosition"), &CRI::getMousePosition, this);
	
	
	// Initialize OGRE
	ROS_INFO("Initializing OGRE 3D...");
		
	// Setup the Ogre3d root
	root = new Ogre::Root(lConfigFileName, lPluginsFileName, lLogFileName);

	// Select the plugins used by this application
	typedef std::vector<Ogre::String> Strings;
	
	Strings lPluginNames;
	lPluginNames.push_back("RenderSystem_GL");
	//lPluginNames.push_back("Plugin_ParticleFX");
	//lPluginNames.push_back("Plugin_CgProgramManager");
	lPluginNames.push_back("Plugin_OctreeSceneManager");

	// Load the Ogre3d plugins
	{
		Strings::iterator lIter = lPluginNames.begin();
		Strings::iterator lIterEnd = lPluginNames.end();
		for(;lIter != lIterEnd; lIter++)
		{
			Ogre::String& lPluginName = (*lIter);
			
			bool lIsInDebugMode = OGRE_DEBUG_MODE;
			if(lIsInDebugMode)
			{
				lPluginName.append("_d");
			}
			root->loadPlugin(lPluginName);
		}
	}

	// Select the renderer
	// For now we're just going with the first renderer
    Ogre::RenderSystemList::iterator r_it;
    Ogre::RenderSystemList renderSystems = root->getAvailableRenderers();
    r_it = renderSystems.begin();
    root->setRenderSystem(*r_it);
    root->initialise(false);
	
	// This is for in case we need to use a config dialog later
	// if (!root->showConfigDialog())
	// {
	//	return;
	//}
	
	bool lCreateAWindowAutomatically = false;
	
	Ogre::String windowTitle = "Visualizer";
	Ogre::String lCustomCapacities = "";
	root->initialise(lCreateAWindowAutomatically, windowTitle, lCustomCapacities);
	
	unsigned int lSizeX = 800;
	unsigned int lSizeY = 600;
	
	bool lFullscreen = false; 
	Ogre::NameValuePairList lParams;
	
	// Fullscreen antialiasing
	
	lParams["FSAA"] = "0"; 
	
	// Vertical synchronisation will prevent some image-tearing, but also
	// will provide smooth framerate in windowed mode
	
	lParams["vsync"] = "true";
	// IMPORTANT params['parentWindowHandle'] = )
	// lParams["currentGLContext"] = "true";
	lParams["parentWindowHandle"] = mainWindowId;
	
	window = root->createRenderWindow(windowTitle, 
		atoi(parm1.c_str()), // Pass back from the initialization Python script
		atoi(parm2.c_str()), 
		lFullscreen, &lParams);
	
	windowMaxWidth = lSizeX;
	windowMaxHeight = lSizeY;
	
	OIS::ParamList paramList;    
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;

	// Here we initialize OIS to handle the mouse and keyboard.

	rotateSpeed = 0.1;
	mouseDownL = false;
    mouseDownR = false;
    leftClicked = 0;
    rightClicked = 0;

	// Get window handle for OIS
	window->getCustomAttribute( "WINDOW", &windowHnd );

	// Fill a parameter list for OIS for OIS
	windowHndStr << (unsigned int) windowHnd;
	paramList.insert(std::make_pair( std::string("WINDOW"), windowHndStr.str()));
	
	// We want OIS to play nice with its neighbors
	paramList.insert(std::make_pair(std::string("x11_mouse_grab"), std::string("false")));
	paramList.insert(std::make_pair(std::string("x11_keyboard_grab"), std::string("false")));
	paramList.insert(std::make_pair(std::string("x11_mouse_hide"), std::string("false")));

	// Create inputsystem
	inputSystem = OIS::InputManager::createInputSystem( paramList );
 
	// Setup the mouse listener
	if (inputSystem->getNumberOfDevices(OIS::OISMouse) > 0) 
	{
		mouse = static_cast<OIS::Mouse *>( inputSystem->createInputObject(OIS::OISMouse, true));
		mouse->setEventCallback( this );

		// Get window size
		unsigned int width, height, depth;
		int left, top;
		window->getMetrics(width, height, depth, left, top);

		// Set mouse region
		this->setWindowExtents( width, height );
		
	}
	else
	{
		mouse = NULL;
	}
	
	// Setup the keyboard listener
	if (inputSystem->getNumberOfDevices(OIS::OISKeyboard) > 0) 
	{
		keyboard = static_cast<OIS::Keyboard*>(inputSystem->createInputObject(OIS::OISKeyboard, true));
		keyboard->setEventCallback(this);
	}
	else
	{
		keyboard = NULL;
	}
	
	// Set the key code text names
	for (int i; i < 145; i++)
	{
		keyStrings[keyCodes[i]] = keyNames[i];
	}
	
	// Register as a window listener
    Ogre::WindowEventUtilities::addWindowEventListener(window, this);
    root->addFrameListener(this);
	
	// Load resources
	Ogre::ConfigFile config;
	config.load((packagePath + "/resources.cfg").c_str());
	
	// Go through all sections & settings in the file
	Ogre::ConfigFile::SectionIterator seci = config.getSectionIterator();
	 
	Ogre::String secName, typeName, archName;
	while (seci.hasMoreElements())
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			typeName = i->first;
			archName = i->second;
			
			archName = parseROSFilename(archName);
			
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
				archName, typeName, secName);
		}
    }
    
	// Set default mipmap level (note: some APIs ignore this)
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(2);
		
	// Initialise all resource groups
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
	
	// Create scene
	scene = root->createSceneManager(Ogre::ST_GENERIC, "SceneManager");
	
	// Need procedural geometry, initialise it..
	Shape::initialisePrimitives();
	
	rootSceneNode = scene->getRootSceneNode();
	
	// Setup the camera
	camera = scene->createCamera("Camera");
	cameraNode = rootSceneNode->createChildSceneNode("CameraNode");
	cameraNode->attachObject(camera);
	
	float lViewportWidth = 1.0f; //0.88f;
	float lViewportHeight = 1.0f; //0.88f;
	float lViewportLeft	= (1.0f - lViewportWidth) * 0.5f;
	float lViewportTop = (1.0f - lViewportHeight) * 0.5f;
	unsigned short lMainViewportZOrder = 100;
	Ogre::Viewport * vp = window->addViewport(camera, lMainViewportZOrder, lViewportLeft, lViewportTop, lViewportWidth, lViewportHeight);
	vp->setAutoUpdated(true);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0.2));
	float ratio = float(vp->getActualWidth()) / float(vp->getActualHeight());
	camera->setAspectRatio(ratio);
	
	camera->setNearClipDistance(0.1f);
	camera->setFarClipDistance(10000.0f);
	
	// Start with a stationary camera looking down at the world
	camPos[0] = 0; camPos[1] = -8; camPos[2] = 3;
	
	// Initial free camera velocity set at sqrt(3) m/s (if moving up, forward and right at the same time)
	camVel[0] = 15; camVel[1] = 15; camVel[2] = 15;
	
	// Comment here
	firstCamCalc = true;
	
	// Initial free camera roll angular velocity set at 0.1 rad/s
	rollVel = 2;
	dRoll = 0;
	dCam[0] = 0; dCam[1] = 0; dCam[2] = 0;
	camFocus[0] = 0; camFocus[1] = 0; camFocus[2] = 0;
	camPose[0] = 0; camPose[1] = 0;
	camDistance = 10;
	
	cameraMode = CAM_FREE;
	
	// Register camera properties with Python
	addProperty(string("CamMode"), PYINT_INT, &(cameraMode));
	
	// for tracking mode
	addProperty(string("SourceFrame"), PYINT_INT, &(sourceFrame));
	addProperty(string("TrackFrameName"), PYINT_STRING, &(trackFrameName));
	
	// for focus mode
	addProperty(string("CamXFocus"), PYINT_DOUBLE, &(camFocus[0]));
	addProperty(string("CamYFocus"), PYINT_DOUBLE, &(camFocus[1]));
	addProperty(string("CamZFocus"), PYINT_DOUBLE, &(camFocus[2]));	
	addProperty(string("CamDistance"), PYINT_DOUBLE, &(camDistance));
	addProperty(string("CamAzimuth"), PYINT_DOUBLE, &(camPose[0]));
	addProperty(string("CamZenith"), PYINT_DOUBLE, &(camPose[1]));
	
	// for flying mode
	addProperty(string("CamXPos"), PYINT_DOUBLE, &(camPos[0]));
	addProperty(string("CamYPos"), PYINT_DOUBLE, &(camPos[1]));
	addProperty(string("CamZPos"), PYINT_DOUBLE, &(camPos[2]));
	
	// for flying mode
	addProperty(string("CamXVel"), PYINT_DOUBLE, &(camVel[0]));
	addProperty(string("CamYVel"), PYINT_DOUBLE, &(camVel[1]));
	addProperty(string("CamZVel"), PYINT_DOUBLE, &(camVel[2]));
	addProperty(string("CamRollVel"), PYINT_DOUBLE, &(rollVel));
	
	addProperty(string("AverageFPS"), PYINT_DOUBLE, &(averageFPS));
	addProperty(string("AverageTriangleCount"), PYINT_DOUBLE, &(averageTriangleCount));
	averageTriangleCount = 0;
	
	// Setup the camera
	switch (cameraMode)
	{
		case CAM_FREE:
			camera->setFixedYawAxis(false);
			camera->setPosition(Ogre::Vector3(camPos[0], camPos[1], camPos[2]));
			camera->lookAt(0, 0, 0);
			break;
			
		case CAM_FOCUS:
			recalcFocusCamera();
			break;
			
		case CAM_TRACKING:
			break;
	}
	
	// Setup rendering options
	skybox = 1;
	addProperty(string("SkyboxOn"), PYINT_INT, &(skybox));
	
	if (skybox)
	{
		// Setup the skybox so that up is the Z axis (ROS is weird with this)
		Ogre::Quaternion quat;
		quat.FromAngleAxis(Ogre::Radian(M_PI/2), Ogre::Vector3(1, 0, 0));
		scene->setSkyBox(true, "Skyboxes/Standard", 5000, true, quat);
	}
	
	// Lighting
	scene->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));
	mainLight = scene->createLight( "MainLight");
	mainLight->setType(Ogre::Light::LT_POINT);
	//mainLight->setDirection(Ogre::Vector3(0, -1, 1 ));
	mainLight->setDiffuseColour(Ogre::ColourValue(0.8f, 0.8f, 0.8f));
	
	// Setup the window
	window->setActive(true);
	window->setAutoUpdated(false);
	root->clearEventTimes();
	windowResized(window);

	// Add the subscriber for markers
	n = _n;
	boss = this;

	// Start the frame listener (default fixed frame is /world)
	fixedFrame = "/world";
	Frame::initFrames();

	// Add a grid to the gadget list
	GGrid * g;
	g = new GGrid("MainGrid");
	
	addGadget(g);
	g->applyProperties();
	g->show(true);
	
	currentMarkerTopic = "visualization_marker";
	markerTopic = currentMarkerTopic;
	currentMarkerArrayTopic = "visualization_marker_array";
	markerArrayTopic = currentMarkerArrayTopic;
	currentMarkerScriptTopic = "visualization_marker_script";
	markerScriptTopic = currentMarkerScriptTopic;
	
	markerSub = n->subscribe(currentMarkerTopic, 100, GActiveMarker::markerCallback);
	markerArraySub = n->subscribe(currentMarkerArrayTopic, 10, GActiveMarker::markerArrayCallback);
	markerScriptSub = n->subscribe(currentMarkerScriptTopic, 100, GActiveMarker::scriptCallback);

	addProperty(string("MarkerTopic"), PYINT_STRING, &(markerTopic));
	addProperty(string("MarkerArrayTopic"), PYINT_STRING, &(markerArrayTopic));
	addProperty(string("MarkerScriptTopic"), PYINT_STRING, &(markerScriptTopic));

	// Initialize interface state variables
	reticleNo = 0;
	pickOn = false;
	menuOn = false;
	activeGadget = NULL;

	// Create default solid color material
	Ogre::ColourValue val2 = Ogre::ColourValue(1.0,0.0,0.0,1);
	Ogre::MaterialPtr matptr = Ogre::MaterialManager::getSingleton().create("SolidColour", "General"); 
	matptr->setReceiveShadows(false); 
	matptr->getTechnique(0)->setLightingEnabled(true);
	matptr->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
	//matptr->getTechnique(0)->getPass(0)->setDiffuse(val2); 
	//matptr->getTechnique(0)->getPass(0)->setAmbient(val2);
	matptr->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	matptr->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);



// There is a bug in Ogre, it doesn't load fonts unless you kick its ass with this:
Ogre::ResourceManager::ResourceMapIterator iter = Ogre::FontManager::getSingleton().getResourceIterator();
	while (iter.hasMoreElements()) { iter.getNext()->load(); }

Ogre::OverlayManager& overlayManager = Ogre::OverlayManager::getSingleton();
 
// Create a panel
Ogre::OverlayContainer* panel = static_cast<Ogre::OverlayContainer*>(
    overlayManager.createOverlayElement("Panel", "PanelName"));
panel->setMetricsMode(Ogre::GMM_RELATIVE);
panel->setPosition(0, 0);
panel->setDimensions(1, 1);
//panel->setMaterialName("MaterialName"); // Optional background material
 
// Create a text area
Ogre::TextAreaOverlayElement* textArea = static_cast<Ogre::TextAreaOverlayElement*>(
    overlayManager.createOverlayElement("TextArea", "TextAreaName"));
textArea->setMetricsMode(Ogre::GMM_RELATIVE);
textArea->setPosition(0.01, 0.96);
textArea->setDimensions(1, 1);
textArea->setCaption("Powered by CRI 0.5 Alpha.");
textArea->setCharHeight(0.03);
textArea->setFontName("Arial");
textArea->setColourBottom(Ogre::ColourValue(0, 0.8, 0));
textArea->setColourTop(Ogre::ColourValue(0, 0.8, 0));
 
// Create an overlay, and add the panel
Ogre::Overlay* overlay = overlayManager.create("OverlayName");
overlay->add2D(panel);
 
// Add the text area to the panel
panel->addChild(textArea);
 
// Show the overlay
overlay->show();

	publishStatus(Gadget::OKAY, "CRI started...");

	return;
}

CRI::~CRI()
{
	printf("Killing gadgets...\n");
	// Clear all the gadgets
	vector<Gadget *>::iterator gadIt;
	
	for (gadIt = gadgets.begin(); gadIt != gadgets.end(); gadIt++)
	{
		delete (*gadIt);
	}

	// Kill Ogre
	
	printf("Killing Ogre...\n");
	// Clean up the input systems
	if (mouse) 
	{
		inputSystem->destroyInputObject(mouse);
	}
	
	if( keyboard) 
	{
		inputSystem->destroyInputObject(keyboard);
	}
	
	
	inputSystem->destroyInputSystem(inputSystem);
	
	// Clean up after Ogre
	window->removeAllViewports();
	scene->destroyAllCameras();
	scene->destroyAllManualObjects();
	scene->destroyAllEntities();
	rootSceneNode->removeAndDestroyAllChildren();

	Ogre::ResourceGroupManager& lRgMgr = Ogre::ResourceGroupManager::getSingleton();
	lRgMgr.shutdownAll(); //destroyResourceGroup(lNameOfResourceGroup);
	
	// Execute Python kill script
	//pyQtInterfaceExec->execute((packagePath + "/src/python/ExecQuit.py").c_str(), NULL);
	//delete pyQtInterfaceExec;
	//delete pyQtInterfaceMain;
	
	exit(0);
	
	// TODO: Why is this a disaster?!
	
	printf("Killing the python interface...\n");
	// Clean up the Python interface
	
	pyQtInterfaceExec->execute((packagePath + "/src/python/ExecQuit.py").c_str(), NULL);
	
	PyEval_AcquireLock();
	PyThreadState_Swap(mainThreadState);
	Py_Finalize();
}

int getMousePos(int & x, int & y)
{
	Display *dsp = XOpenDisplay( NULL );
	if( !dsp ){ return 1; }

	int screenNumber = DefaultScreen(dsp);

	XEvent event;

	/* get info about current pointer position */
	XQueryPointer(dsp, RootWindow(dsp, DefaultScreen(dsp)),
		&event.xbutton.root, &event.xbutton.window,
		&event.xbutton.x_root, &event.xbutton.y_root,
		&event.xbutton.x, &event.xbutton.y,
		&event.xbutton.state);

	x = event.xbutton.x;
	y = event.xbutton.y;

	XCloseDisplay( dsp );
	
	return 0;
}

/**
 * -
 *
 * @param Ogre::FrameEvent& $evt
 *   An initialized node handle
 *
 */
bool CRI::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(window->isClosed())
        return false;
 
	// Todo: Need to add shutdown code in quit
    //if (shutDown)
    //    return false;
 
	// No CEGUI
    //Need to inject timestamps to CEGUI System.
    //CEGUI::System::getSingleton().injectTimePulse(evt.timeSinceLastFrame);
 
    return true;
}

/**
 * Method spin() contains the main loop for the program.  Every iteration captures a general mutex for access
 * to gadget data then updates graphics frames based on new data from TF, applies property 
 * changes to gadgets ordered by the Python interface, invokes the renderer and captures 
 * keyboard and mouse events.
 *
 */
void CRI::spin()
{
	vector<Gadget *>::iterator gadgetIt;
	
	// Don't let random stuff change unless we give the go ahead
	generalMut->lock();
	
	shutdown = false;
	
	// While the OpenGL window is open loop the renderer...
	while(!window->isClosed() && !shutdown)
	{
		// WTF does this do?
		window->update(false);
		bool lVerticalSynchro = true;
		
		// Let any Python callbacks awaiting execution have a go
		generalMut->unlock();
		generalMut->lock();
		
		/*
		// Legacy approach, we don't use the individual python interfaces anymore, please ignore.
		// Lock all the gadgets for rendering (incase Python is busy with them)
		for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
		{
			(*gadgetIt)->generalMut->lock();
		}
		*/
		
		// Update ROS TF and TF2 frames
		Frame::updateFrames(this);
		
		// Have any of CRI's main class properties changed?
		if (needsApply)
		{
			// Check if any marker topics have changed, and if so update
			
			if (currentMarkerTopic != markerTopic)
			{
				markerSub.shutdown();
				
				currentMarkerTopic = markerTopic;
				markerSub = n->subscribe(currentMarkerTopic, 100, GActiveMarker::markerCallback);
			}
				
			if (currentMarkerArrayTopic != markerArrayTopic)
			{
				markerArraySub.shutdown();
				
				currentMarkerArrayTopic = markerArrayTopic;
				markerArraySub = n->subscribe(currentMarkerArrayTopic, 10, GActiveMarker::markerArrayCallback);
			}
			
			if (currentMarkerScriptTopic != markerScriptTopic)
			{
				markerScriptSub.shutdown();
				
				currentMarkerScriptTopic = markerScriptTopic;
				markerScriptSub = n->subscribe(currentMarkerScriptTopic, 100, GActiveMarker::scriptCallback);
			}	
			
			// Check if skybox state has changed
			if (!skybox && scene->isSkyBoxEnabled())
			{
				scene->setSkyBox(false, "");
			}
			else if (skybox && !scene->isSkyBoxEnabled())
			{
				// Setup the skybox so that up is the Z axis (ROS is weird with this)
				Ogre::Quaternion quat;
				quat.FromAngleAxis(Ogre::Radian(M_PI/2), Ogre::Vector3(1, 0, 0));
				scene->setSkyBox(true, "Skyboxes/Standard", 5000, true, quat);
			}
			
			// Adjust the camera if necessary
			switch (cameraMode)
			{
				case CAM_FREE:
					break;
			
				case CAM_FOCUS:
					recalcFocusCamera();
					break;
			
				case CAM_TRACKING:
					{
						sourceFrame = Frame::findFrame(trackFrameName);
						if (sourceFrame < 0)
						{
							cameraMode = CAM_FREE;
						}
					}
					
					break;
			}
			
			// Update frame properties for all frames if necessary
			for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
			{
				if ((*gadgetIt)->type == Gadget::GADGET_FRAME)
				{
					(*gadgetIt)->needsApply = true;
				}
			}
			
			needsApply = false;
		}
		
		// Update any gadgets that have had their properties changed by the Python interface
		for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
		{
			(*gadgetIt)->update();
			if ((*gadgetIt)->needsApply)
			{
				(*gadgetIt)->applyProperties();
				(*gadgetIt)->needsApply = false;
			}
		}
		
		// Delete any gadgets that want to die
		int done = 0;
		while (!done)
		{
			done = 1;
			
			for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
			{
				// If a gadget wants to die, kill it
				if ((*gadgetIt)->killMe)
				{
					if (activeGadget == (*gadgetIt))
					{
						activeGadget = NULL;
					}
					
					delete *gadgetIt;
					
					removeInterface(*gadgetIt);
					gadgets.erase(gadgetIt);
					done = 0;
					break;
				}
			}
		}
		
		// Capture mouse
		mouse->capture();
		
		// Note that a 5x5 pixel correction is add to the mouse position to make things look right
		Ogre::Ray ray = camera->getCameraToViewportRay(mx + 0.007, my + 0.007);
		
		// If a gadget is active but the user clicked off it, make it inactive
		if (activeGadget && (leftClicked > 0))
		{
			if (!activeGadget->clickedInRange(ray))
			{
				activeGadget = NULL;
			}
		}
		
		// If none of the gadgets are active, check if the user is clicking on one, and if so
		// make it active, if it allows.
		if (!activeGadget && (leftClicked > 0))
		{
			double distance, minDistance;
			
			for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
			{
				if ((*gadgetIt)->clicked(ray, 
					Ogre::Vector3(camPos[0], camPos[1], camPos[2]), 
					distance))
				{
					if (!activeGadget)
					{
						activeGadget = *gadgetIt;
						minDistance = distance;
					}
					else if (distance < minDistance)
					{
						activeGadget = *gadgetIt;
						minDistance = distance;
					}
				}
			}
			
			// Don't want the click event moving on to activating an element of the 
			// gadget in the next loop.
			leftClicked = 0;
		}
		
		for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
		{
			(*gadgetIt)->giveMouse(ray, 
				leftClicked > 0,
				rightClicked > 0,
				(*gadgetIt) == activeGadget);
		}
		
		if (leftClicked > 0)
		{
			leftClicked = 0;
		}
		if (rightClicked > 0)
		{
			rightClicked = 0;
		}
		
		// Allow the python interface to enquire which gadget is active
		if (activeGadget)
		{
			activeGadgetName = activeGadget->name;
		}
		else
		{
			activeGadgetName = "";
		}
		
		// Adjust the camera
		switch (cameraMode)
		{
			case CAM_FREE:
				{
					if (firstCamCalc)
					{
						camTimeMark = ros::Time::now();
						firstCamCalc = false;
					}
					else
					{
						// Move the camera as per its velocity and pose
						Ogre::Quaternion quat = camera->getOrientation();
						Ogre::Vector3 x = quat.xAxis();
						Ogre::Vector3 y = quat.yAxis();
						Ogre::Vector3 z = quat.zAxis();
						
						Ogre::Vector3 tPos(camPos[0], camPos[1], camPos[2]);
						
						double dt = (ros::Time::now() - camTimeMark).toSec();
					
						// Inelegant for the sake of the Python interface :P
						tPos = tPos + (x * dCam[0] + y * dCam[1] + z * dCam[2]) * dt;
						camera->setPosition(tPos);
						
						// A bit ugly this:
						// Moving light with the camera
						mainLight->setPosition(tPos);
						camPos[0] = tPos[0]; camPos[1] = tPos[1]; camPos[2] = tPos[2];
						
						camera->roll(Ogre::Radian(dRoll * dt));
						
						
						camTimeMark = ros::Time::now();
					}
					
					break;
				}
				
			case CAM_FOCUS:
				break;
				
			case CAM_TRACKING:
				if (availableFrames[sourceFrame].valid)
				{
					tf::StampedTransform & tran = availableFrames[sourceFrame].extrapTransform;
					tf::Vector3 & pos = tran.getOrigin();
					tf::Quaternion rot = tran.getRotation();
	
					// Keep the free camera position following the camera frame so that
					// if the user detaches his camera doesn't move suddenly.
					
					camPos[0] = pos.x(); camPos[1] = pos.y(); camPos[2] = pos.z();
					camera->setPosition(pos.x(), pos.y(), pos.z());
					camera->setOrientation(Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z()));
				}
				
				break;
		}
		
		
		// Render the frame
		root->renderOneFrame();
		
		/*
		// Legacy approach, we don't use the individual python interfaces anymore, please ignore.

		// Unlock all the gadgets
		for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
		{
			(*gadgetIt)->generalMut->unlock();
		}
		*/
		
		// Swap the buffers
		window->swapBuffers(lVerticalSynchro);
		
		//Need to capture/update each device
		keyboard->capture();
		
		// And handle the messages
		Ogre::WindowEventUtilities::messagePump();		
		
		// Give ROS some time
		ros::spinOnce();
		
		// Update data
		averageFPS = window->getAverageFPS();
		averageTriangleCount = averageTriangleCount * 0.9 + window->getTriangleCount() * 0.1;
	}
	
	// And we're done
	//root->shutdown();
}

void CRI::windowResized(Ogre::RenderWindow * renderWindow)
{
	setWindowExtents(renderWindow->getWidth(), renderWindow->getHeight());
}

// TODO : This function may be redundant
void CRI::setWindowExtents(int width, int height)
{
	// Set mouse region (if window resizes, we should alter this to reflect as well)
	const OIS::MouseState &mouseState = mouse->getMouseState();
	
	mouseState.width  = width;
	mouseState.height = height;
	
	windowMaxWidth = width;
	windowMaxHeight = height;
}

void CRI::setPickOn(void * classData)
{
	CRI * cri = (CRI *)classData;
	// We are in pick mode
	cri->pickOn = true;
}

bool CRI::mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
	mx = ((double)arg.state.X.abs) / windowMaxWidth;
	my = ((double)arg.state.Y.abs) / windowMaxHeight;
	
	// Left mouse button down
	if (id == OIS::MB_Left)
	{
		if (pickOn)
		{
			GReticle * tGad;
			vector<Gadget *>::iterator gadgetIt;
			vector<Ogre::Vector3> potentials;
			int potIdx;
			vector<int> frameNos;
			vector<Gadget *> newReticles;
			
			// Todo:  Why is this critical?
			// Go "critical section" just in case and lock all the gadgets
			/*
			for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
			{
				(*gadgetIt)->generalMut->lock();
			}
			*/
			
			Ogre::Ray ray = camera->getCameraToViewportRay(mx, my);
			
			// Clear any existing existing reticles
			int done = 0;
			while (!done)
			{
				done = 1;
				
				for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
				{
					if ((*gadgetIt)->type == Gadget::GADGET_RETICLE_TRAN)
					{	
						delete *gadgetIt;
						
						removeInterface(*gadgetIt);
						gadgets.erase(gadgetIt);
						done = 0;
						break;
					}
				}
			}
			
			// Update derived frames and markers
			for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
			{
				potentials.clear();
				frameNos.clear();
				
				(*gadgetIt)->getIntersections(potentials, frameNos, ray);
				
				// Add reticles along the intersections
				
				for (potIdx = 0; potIdx < potentials.size(); potIdx++)
				{
					
					// Check that the point isn't behind the camera
					if (ray.getDirection().dotProduct(potentials[potIdx] - ray.getOrigin()) < 0)
					{
						continue;
					}
					
					tGad = new GReticle(("Reticle" + boost::lexical_cast<std::string>(reticleNo++)).c_str());
					tGad->sourceFrame = frameNos[potIdx];
					if (tGad->sourceFrame >= 0)
					{
						tGad->frameName = availableFrames[tGad->sourceFrame].name;
					}
					tGad->relativePos[0] = potentials[potIdx][0];
					tGad->relativePos[1] = potentials[potIdx][1];
					tGad->relativePos[2] = potentials[potIdx][2];

					tGad->targetType = (*gadgetIt)->type;
					tGad->targetName = (*gadgetIt)->name;					

					newReticles.push_back(tGad);
				}
			}
			
			// Unlock all the gadgets
			/*
			for (gadgetIt = gadgets.begin(); gadgetIt != gadgets.end(); gadgetIt++)
			{
				(*gadgetIt)->generalMut->unlock();
			}
			*/		
				
			// Add new reticles to main gadget vector
			for (gadgetIt = newReticles.begin(); gadgetIt != newReticles.end(); gadgetIt++)
			{
				addGadget(*gadgetIt);
				(*gadgetIt)->show(true);
				(*gadgetIt)->needsApply = true;
			}
					
			// Execute selections list python code
			pyQtInterfaceExec->execute((packagePath + "/src/python/ExecListSelectionsDialog.py").c_str());
		
			// Done picking
			pickOn = false;
		}
		else // !pickOn
		{
			mouseDownL = true;
			if (leftClicked == 0)
			{
				leftClickStart = ros::Time::now();
				leftClicked = -1;
			}
		}
	}

	// Right mouse button down
	else if (id == OIS::MB_Right)
	{	
		// Mouse position stored for Python interface to use
		getMousePos(menuStartX, menuStartY);
		
		// Show the commmand window
		pyQtInterfaceExec->execute((packagePath + "/src/python/ExecMainMenuDialog.py").c_str());
		
		mouseDownR = true;
		
		if (rightClicked == 0)
		{
			rightClicked = -1;
		}
	}
	
	return true;
}
  
bool CRI::mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
	// Left mouse button up
	if (id == OIS::MB_Left)
	{
		mouseDownL = false;
		
		if (leftClicked == -1)
		{
			// If the click was short enough to actually be a click, take it as one
			if ((ros::Time::now() - leftClickStart).toSec() < 0.2)
			{
				leftClicked = 1;
			}
			else leftClicked = 0;
			
		}
	}

	// Right mouse button up
	else if (id == OIS::MB_Right)
	{
		mouseDownR = false;
		
		if (rightClicked == -1)
		{
			rightClicked = 1;
		}
	}
	
	// No CEGUI
	// Tell CEGUI about the release
	//CEGUI::System::getSingleton().injectMouseButtonUp(convertButton(id));
	
	
	return true;
} 

void CRI::recalcFocusCamera()
{
	// Calculate the position and orientation of a camera focusing at a point
						
	camPos[0] = camFocus[0] + cos(camPose[1]) * cos(camPose[0]) * camDistance;
	camPos[1] = camFocus[1] + cos(camPose[1]) * sin(camPose[0]) * camDistance;
	camPos[2] = camFocus[2] + sin(camPose[1]) * camDistance;
	
	Ogre::Vector3 zAx(cos(camPose[1]) * cos(camPose[0]),
		cos(camPose[1]) * sin(camPose[0]),
		sin(camPose[1]));
		
	Ogre::Vector3 up(0, 0, 1);
	
	Ogre::Vector3 xAx = up.crossProduct(zAx);
	xAx.normalise();
	Ogre::Vector3 yAx = xAx.crossProduct(-zAx);

	camera->setPosition(camPos[0], camPos[1], camPos[2]);
	
	Ogre::Quaternion quat;
	quat.FromAxes(xAx, yAx, zAx);
	
	camera->setOrientation(quat);
}

bool CRI::mouseMoved(const OIS::MouseEvent &arg)
{
	mx = ((double)arg.state.X.abs) / windowMaxWidth;
	my = ((double)arg.state.Y.abs) / windowMaxHeight;	
	
	// Handle the scroll button
	switch (cameraMode)
	{
		case CAM_FREE:  // Move forward/backward on the scrollers command in CAM_FREE mode
			{
				Ogre::Vector3 motion = ((double)arg.state.Z.rel)/250.0 * camera->getDirection();
					
				camPos[0] += motion[0];
				camPos[1] += motion[1];
				camPos[2] += motion[2];
				
				break;
			}
			
		case CAM_FOCUS: // Move toward/from the camera orbit point
			camDistance -= ((double)arg.state.Z.rel)/250.0;
			
			if (camDistance < 0)
			{
				camDistance = 0;
			}
		
			recalcFocusCamera();
			break;
	}
	
	if (mouseDownR)
	{
		
	}
	// If we are dragging the left mouse button.
	else if (mouseDownL)
	{
		if (!menuOn)
		{
			switch (cameraMode)
			{
				case CAM_FREE:
					camera->yaw(Ogre::Degree(-arg.state.X.rel * rotateSpeed));
					camera->pitch(Ogre::Degree(-arg.state.Y.rel * rotateSpeed));
					
					break;
				
				case CAM_FOCUS:
				{
					// Yaw
					camPose[0] += arg.state.X.rel * rotateSpeed * 0.1;
					// Pitch
					camPose[1] -= arg.state.Y.rel * rotateSpeed * 0.1;
					
					recalcFocusCamera();
					
					// A bit ugly this:
					// Moving light with the camera
					mainLight->setPosition(Ogre::Vector3(camPos[0], camPos[1], camPos[2]));
					
					break;
				}
					
				case CAM_TRACKING:
					break;
			}
		}
	
		return true;
	}
		
	return true;
}

bool CRI::keyPressed(const OIS::KeyEvent& evt)
{
	char * code;
	
	code = keyPressedAssignments[evt.key].getScript();
	
	if (code != NULL)
	{
		pyQtInterfaceExec->executeString(code);
	}
	
	switch (cameraMode)
	{
		case CAM_FREE:
			switch (evt.key)
			{
				
				
				// Facilitating camera roll with the keyboard
				case OIS::KC_E:
					dRoll = rollVel;
					break;
				case OIS::KC_C:
					dRoll = -rollVel;
					break;
				
				// Facilitating keyboard 'strafing' with the keyboard
				case OIS::KC_Q:
					dCam[2] = -camVel[2];
					break;
					
				case OIS::KC_Z:
					dCam[2] = camVel[2];
					break;
					
				case OIS::KC_LEFT:
				case OIS::KC_A:
					dCam[0] = -camVel[0];
					break;
					
				case OIS::KC_RIGHT:
				case OIS::KC_D:
					dCam[0] = camVel[0];
					break;
					
				//case OIS::KC_LEFT:
				case OIS::KC_UP:
				case OIS::KC_W:
					dCam[1] = camVel[1];
					break;
					
				//case OIS::KC_RIGHT:
				case OIS::KC_DOWN:
				case OIS::KC_S:
					dCam[1] = -camVel[1];
					break;
					
				case OIS::KC_ESCAPE: 
					activeGadget = NULL;
				
					// Make this work
					//mShutDown = true;
					break;
				default:
					break;
			}
		default:
			break;
	}
	
	
	
    return true;
}

bool CRI::keyReleased(const OIS::KeyEvent& evt)
{
	char * code;
	
	code = keyReleasedAssignments[evt.key].getScript();
	
	if (code != NULL)
	{
		pyQtInterfaceExec->executeString(code);
	}	
	
	switch (cameraMode)
	{
		case CAM_FREE:
			switch (evt.key)
			{
				// Facilitating camera roll with the keyboard
				case OIS::KC_E:
					dRoll = 0;
					break;
				case OIS::KC_C:
					dRoll = 0;
					break;
				
				// Facilitating keyboard 'strafing' with the keyboard
				case OIS::KC_Q:
					dCam[2] = 0;
					break;
				
				case OIS::KC_Z:
					dCam[2] = 0;
					break;
					
				case OIS::KC_LEFT:
				case OIS::KC_A:
					dCam[0] = 0;
					break;
					
				case OIS::KC_RIGHT:
				case OIS::KC_D:
					dCam[0] = 0;
					break;
					
				//case OIS::KC_LEFT:
				case OIS::KC_UP:
				case OIS::KC_W:
					dCam[1] = 0;
					break;
					
				//case OIS::KC_RIGHT:
				case OIS::KC_DOWN:
				case OIS::KC_S:
					dCam[1] = 0;
					break;
				default:
					break;
			}
			break;
	}
    return true;
}

void CRI::addGadget(Gadget * element)
{
	element->assignGlobals(this, rootSceneNode, scene);
	
	gadgets.push_back(element);
	
	addInterface(element);
}

void CRI::publishStatus(int _statusNo, string _status)
{
	status = _status;
	statusNo = _statusNo;
	
	// Check if this status is just a repetition, if so ignore it
	if (_status == status && _statusNo == statusNo)
	{
		return;
	}
	
	publishStatus(_statusNo, "CRI", _status);
}

void CRI::publishStatus(int _statusNo, string source, string _status)
{
	pyQtInterfaceExec->executeString(("r.publishStatus('" +
		source + "', '" + _status + "', " + 
		boost::lexical_cast<std::string>(_statusNo) + ")\n").c_str());
}


void CRI::getFrameParent(void * classData)
{
	string result;
	CRI * cri = (CRI *)classData;
	
	if (tfListener->getParent(cri->parm1, ros::Time(), result) == false)
	{
		cri->parm1 = "";
	}
	else
	{
		cri->parm1 = result;
	}
}

void CRI::addTopic(void * classData)
{
	vector<Gadget *>:: iterator gadgetIter;
	
	CRI * cri = (CRI *)classData;
	// Parm1 is the topic name
	// Parm2 is the topic type
	
	// Check for duplicate topics, and return if one is found.
	for (gadgetIter = cri->gadgets.begin(); gadgetIter != cri->gadgets.end(); gadgetIter++)
	{
		if ((*gadgetIter)->name == cri->parm1)
		{
			return;
		}
	}
	
	if (cri->parm2 == "sensor_msgs/PointCloud2")
	{
		GPointCloud * pc;
		
		pc = new GPointCloud(cri->parm1.c_str());

		cri->addGadget(pc);
		
		pc->show(true);
		pc->subscribePointCloud2(cri->parm1);
	}
	else if (cri->parm2 == "sensor_msgs/LaserScan")
	{
		GPointCloud * pc;
		
		pc = new GPointCloud(cri->parm1.c_str());

		cri->addGadget(pc);
		
		pc->show(true);
		pc->subscribeLaserScan(cri->parm1);
	}
	else if (cri->parm2 == "nav_msgs/Path")
	{
		GPath * path;
		
		path = new GPath(cri->parm1.c_str());

		cri->addGadget(path);
		
		path->show(true);
		path->subscribe(cri->parm1);
	}
	else if (cri->parm2 == "nav_msgs/GridCells")
	{
		GGridCells * gridCells;
		
		gridCells = new GGridCells(cri->parm1.c_str());

		cri->addGadget(gridCells);
		
		gridCells->show(true);
		gridCells->subscribe(cri->parm1);
	}
	else if (cri->parm2 == "nav_msgs/Odometry")
	{
		GQuiver * quiver;
		
		quiver = new GQuiver(cri->parm1.c_str());

		cri->addGadget(quiver);
		
		quiver->show(true);
		quiver->subscribeOdometry(cri->parm1);
	}
	else if (cri->parm2 == "geometry_msgs/PoseStamped")
	{
		GQuiver * quiver;
		
		quiver = new GQuiver(cri->parm1.c_str());

		cri->addGadget(quiver);
		
		quiver->show(true);
		quiver->subscribePoseStamped(cri->parm1);
	}
	else if (cri->parm2 == "geometry_msgs/PoseArray")
	{
		GQuiver * quiver;
		
		quiver = new GQuiver(cri->parm1.c_str());

		cri->addGadget(quiver);
		
		quiver->show(true);
		quiver->subscribePoseArray(cri->parm1);
	}
	else if (cri->parm2 == "nav_msgs/OccupancyGrid")
	{
		GTextureQuad * textureQuad;
		
		textureQuad = new GTextureQuad(cri->parm1.c_str());

		cri->addGadget(textureQuad);
		
		textureQuad->show(true);
		textureQuad->subscribe(cri->parm1);
	}
	else
	{
		printf("Unknown message type %s\n", cri->parm1.c_str());
	}
}

void CRI::addReticle(void * classData)
{
	// Parm1 is is the gadget name, "" will have use generate a random one
	// Parm2 is the gadget frame name, "" will have no frame assigned
	
	CRI * cri = (CRI *)classData;
	
	GReticle * reticle;
	
	reticle = new GReticle(cri->parm1.c_str());
	
	cri->addGadget(reticle);
	
	reticle->show(true);
	
	if (cri->parm2 == "")
	{
		reticle->sourceFrame = -1;
	}
	else
	{
		reticle->frameName = cri->parm2;
	}
	
	reticle->needsApply = true;
}

int gridNo = 0;

void CRI::addGrid(void * classData)
{
	CRI * cri = (CRI *)classData;
	
	// Parm1 is is the gadget name, "" will have use generate a random one
	// Parm2 is the gadget frame name, "" will have no frame assigned
	
	// Create a grid
	GGrid * newGrid;
	
	if (cri->parm1 == "")
	{
		string name = ("Grid" + boost::lexical_cast<std::string>(gridNo++));
		newGrid = new GGrid(name.c_str());
	}
	else
	{
		newGrid = new GGrid(cri->parm1.c_str());
	}
	
	cri->addGadget(newGrid);
	
	newGrid->show(true);
	
	if (cri->parm2 == "")
	{
		newGrid->sourceFrame = -1;
	}
	else
	{
		newGrid->frameName = cri->parm2;
	}
	
	cri->needsApply = true;
}

void CRI::deleteGadget(void * classData)
{
	int idx, notdone;
	
	CRI * cri = (CRI *)classData;
	
	// In case there are duplicate names, keep going until all matches are deleted
	notdone = 1;
	while (notdone)
	{
		notdone = 0;
		for (idx = 0; idx < cri->gadgets.size(); idx++)
		{
			if (cri->gadgets[idx]->name == cri->parm1)
			{
				delete cri->gadgets[idx];
				
				cri->removeInterface(cri->gadgets[idx]);
				cri->gadgets.erase(cri->gadgets.begin() + idx);
				
				idx--;
			}
		}
	}
}

void CRI::getKeyPressedAssignment(void * classData)
{
	CRI * cri = (CRI *)classData;
	int num;
	
	num = atoi(cri->parm1.c_str());
	
	if (num >= 0 && num <= 0xED)
	{
		cri->parm1 = cri->keyStrings[num];
		cri->parm2 = cri->keyPressedAssignments[num].scriptFileName;
	} 
}

void CRI::setKeyPressedAssignment(void * classData)
{
	CRI * cri = (CRI *)classData;
	int num;
	
	num = atoi(cri->parm1.c_str());
	
	if (num >= 0 && num <= 0xED)
	{
		cri->keyPressedAssignments[num].assign(cri->parm2);
	} 
}

void CRI::getKeyReleasedAssignment(void * classData)
{
	CRI * cri = (CRI *)classData;
	int num;
	
	num = atoi(cri->parm1.c_str());
	
	if (num >= 0 && num <= 0xED)
	{
		cri->parm1 = cri->keyStrings[num];
		cri->parm2 = cri->keyReleasedAssignments[num].scriptFileName;
	} 
}

void CRI::setKeyReleasedAssignment(void * classData)
{
	CRI * cri = (CRI *)classData;
	int num;
	
	num = atoi(cri->parm1.c_str());
	
	if (num >= 0 && num <= 0xED)
	{
		cri->keyReleasedAssignments[num].assign(cri->parm2);
	} 
}

void CRI::getMousePosition(void * classData)
{
	CRI * cri = (CRI *)classData;
	
	getMousePos(cri->mouseX, cri->mouseY);
	
	cri->parm1 = boost::lexical_cast<std::string>(cri->mouseX);
	cri->parm2 = boost::lexical_cast<std::string>(cri->mouseY);
}

void CRI::windowChanged(void * classData)
{
	int width, height;
	CRI * cri = (CRI *)classData;
	
	//cri->window->windowMovedOrResized();
	
	// Correct window size
	width = atoi(cri->parm1.c_str());
	height = atoi(cri->parm2.c_str());
	cri->window->resize(width, height);
	
	// Correct aspect ratio
	float ratio = float(width) / float(height);
	cri->camera->setAspectRatio(ratio);
}

void CRI::die(void * classData)
{
	CRI * cri = (CRI *)classData;
	
	exit(0);
	
	cri->shutdown = true;
}



Gadget::Gadget(const char * _name) : PyInterface(string(_name))
{
	// Gadget is invisible and not wanting to die, by default
	showable = false;
	visible = false;
	killMe = false;
	
	// Need to have the gadget's applyProperties() method called asap
	needsApply = true;
	
	name = string(_name);
	
	// No frame assignment
	sourceFrame = -1;
	type = GADGET_OTHER;
	
	// Must register to python the gadget type
	addProperty(string("type"), PYINT_INT, &type);
	addProperty(string("status"), PYINT_STRING, &status);
	addProperty(string("statusNo"), PYINT_INT, &statusNo);
	
	
	// No scene node
	graphicNode = NULL;
	
	// Clear status
	status = "";
	statusNo = 0;
}

//void Gadget::publishStatus(int _statusNo, char * _status)
//{
	//status = _status;
	//statusNo = _statusNo;
//}

void Gadget::publishStatus(int _statusNo, string _status)
{
	// Check if this status is just a repetition, if so ignore it
	if (_status == status && _statusNo == statusNo)
	{
		return;
	}
	
	status = _status;
	statusNo = _statusNo;
	
	master->publishStatus(statusNo, name, status);
}

void Gadget::update()
{
}

void Gadget::getIntersections(vector<Ogre::Vector3> & potentials, vector<int> & frameNos, Ogre::Ray & ray)
{
	// The virtual function will report nothing.
	
	return;
}

void Gadget::giveMouse(Ogre::Ray & ray, int leftClicked, int rightClicked, int active)
{
	// The virtual function will do nothing
	
	return;
}

bool Gadget::clicked(Ogre::Ray & ray, Ogre::Vector3 cameraPos, double & distance)
{
	// The virtual function will do nothing
	
	return false;
}

bool Gadget::clickedInRange(Ogre::Ray & ray)
{
	// The virtual function will do nothing
	
	return false;
}

void Gadget::assignGlobals(CRI * _master, Ogre::SceneNode * _parent, Ogre::SceneManager * _scene)
{	
	master = _master;
	parent = _parent;
	scene = _scene;
}

Gadget::~Gadget()
{
}

void Gadget::show(bool value)
{
};


keyAssignment::keyAssignment()
{
	code = NULL;
}

keyAssignment::~keyAssignment()
{
	if (code)
	{
		delete code;
	}
}
	
char * keyAssignment::getScript()
{
	return code;
}

int keyAssignment::assign(string fileName)
{
	FILE * fileHandle;
	long fSize;
	
	if (code)
	{
		delete [] code;
		code = NULL;
	}
	
	// Check that the file exists
	fileHandle = fopen(fileName.c_str(), "r");
	
	// If file is not found throw an exception
	if (!fileHandle)
	{
		scriptFileName = "";
		
		return -1;
	}
	
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
	
	scriptFileName = fileName;
	
	return 0;
}
