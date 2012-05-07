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

#ifndef __CRI
#define __CRI

#include <ros/ros.h>

#include <stdio.h>

#include "CRIMain.h"
#include "PyBlock.h"
#include <vector>

#include <OgreException.h>
#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreManualObject.h>
#include <OgreEntity.h>
#include <OgreBillboardSet.h>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreWindowEventUtilities.h>
#include <OgreFrameListener.h>
#include <OgreMath.h>
#include <OgreMeshManager.h>
#include <OgreMeshSerializer.h>
#include <OgreLight.h>
#include <OgreFontManager.h>
#include <OgreTextAreaOverlayElement.h>

#include <Procedural.h>
#include "OgreTools2.h"
#include "ROSTools.h"

#include <OISInputManager.h>
#include <OISMouse.h>
#include <OISKeyboard.h>

#include <vector>

#define OGRE_DEBUG_MODE 0

#include <boost/python.hpp>
#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
//#include <boost/enable_shared_from_this.hpp>

// No CEGUI
//#include <CEGUI.h>
//#include <OgreCEGUIResourceProvider.h>
//#include <RendererModules/Ogre/CEGUIOgreRenderer.h>

#include <X11/X.h>
#include <X11/Xlib.h> 
#include <X11/Xutil.h>

using namespace boost::python;
using namespace std;

class CRI;

class Gadget : public PyInterface
{	
	public:
	// The TF/TF2 frame this gadget is attached to
	int sourceFrame;
	
	int type;
	static const int GADGET_GRID = 1;
	static const int GADGET_FRAME = 2;
	static const int GADGET_RETICLE_TRAN = 3;
	static const int GADGET_RETICLE = 4;
	static const int GADGET_MARKER = 5;
	static const int GADGET_POINTCLOUD = 10;
	static const int GADGET_PATH = 11;
	static const int GADGET_GRIDCELLS = 12;
	static const int GADGET_TEXTUREQUAD = 13;
	static const int GADGET_LASERSCAN = 14;
	static const int GADGET_QUIVER = 15;
	static const int GADGET_ODOMETRY = 16;
	static const int GADGET_POSESTAMPED = 17;
	static const int GADGET_POSEARRAY = 18;
	
	static const int GADGET_OTHER = 0;
	
	static const int OKAY = 0;
	static const int WARNING = -1;
	static const int ERROR = -2;
	
	bool showable;
	bool visible;
	bool killMe;
	
	Gadget(const char * name);
	virtual ~Gadget();

	//void changeParent(Ogre::SceneNode * _parent);
	virtual void show(bool value);
	virtual void update();
	//virtual void publishStatus(int _statusNo, char * _status);
	virtual void publishStatus(int _statusNo, string _status);
	
	// Todo: Optimize this process
	virtual void getIntersections(vector<Ogre::Vector3> & potentials, vector<int> & frameNos, Ogre::Ray & ray);
	
	virtual void giveMouse(Ogre::Ray & ray, int leftClicked, int rightClicked, int active);
	virtual bool clicked(Ogre::Ray & ray, Ogre::Vector3 cameraPos, double & distance);
	virtual bool clickedInRange(Ogre::Ray & ray);
	
	void assignGlobals(CRI * _master, Ogre::SceneNode * _parent, Ogre::SceneManager * _scene);
	
	string name;
	string status;
	int statusNo;
	
	CRI * master;
	Ogre::SceneManager * scene;
	Ogre::SceneNode * graphicNode;
	Ogre::SceneNode * parent;
};

class keyAssignment
{
	public:
	keyAssignment();
	~keyAssignment();
	string scriptFileName;
	
	char * getScript();
	int assign(string fileName);
	
	private:
	char * code;
};

class CRI : public OIS::KeyListener, OIS::MouseListener, public Ogre::WindowEventListener, public Ogre::FrameListener, public PyInterface
{
	public:
	ros::NodeHandle * n;

	CRI(ros::NodeHandle * _n);
	~CRI();

	vector<Gadget *> gadgets;

	void spin();
	void addGadget(Gadget * element);
	
	// Mouse related
	
	bool mouseDownL, mouseDownR;
	double rotateSpeed;
	int mouseX, mouseY;
	
	// Position of right click to open the menu window, used by subsequence windows 
	// to keep a consistent start location
	
	int menuStartX, menuStartY;
	
	// Regularly updated mouse position in the window from 0.0 to 1.0
	double mx, my;
	// Click state
	int leftClicked, rightClicked;
	ros::Time leftClickStart;
	Gadget * activeGadget;
	string activeGadgetName;
	
	virtual bool mouseMoved(const OIS::MouseEvent &arg);
    virtual bool mousePressed(const OIS::MouseEvent &arg,OIS::MouseButtonID id);
    virtual bool mouseReleased(const OIS::MouseEvent &arg,OIS::MouseButtonID id);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
    
    virtual void windowResized(Ogre::RenderWindow * renderWindow);

	// Keyboard related

	keyAssignment keyPressedAssignments[0xED + 1];
	keyAssignment keyReleasedAssignments[0xED + 1];
	string keyStrings[0xED + 1];
	virtual bool keyPressed(const OIS::KeyEvent& evt);
    virtual bool keyReleased(const OIS::KeyEvent& evt);

	void setWindowExtents(int width, int height);
	
	// Renderer data
	double averageFPS;	
	double averageTriangleCount;
	
	// Camera variables
	Ogre::Camera * camera;
	Ogre::SceneNode * cameraNode;  
	Ogre::Vector3 dCam;
	ros::Time camTimeMark;
	bool firstCamCalc;
	double dRoll;
	
	// Camera parameters	
	int cameraMode;
	
	static const int CAM_FREE = 0;
	static const int CAM_FOCUS = 1;
	static const int CAM_TRACKING = 2;
	
	// For CAM_FREE mode
	double camPos[3];
	double camVel[3];
	double rollVel;
	
	// For CAM_FOCUS mode
	double camFocus[3];
	double camPose[2]; // Azimuth then Zenith
	double camDistance;
	
	// For CAM_TRACKING mode
	int sourceFrame;
	string trackFrameName;
	
	// Camera functions
	void recalcFocusCamera();
	
	// Rendering options
	int skybox; // 1 - Render skybox, 0 - no skybox
	
	// Ogre variables
	Ogre::Root * root;
	Ogre::RenderWindow * window;
	Ogre::SceneManager * scene;
	Ogre::SceneNode * rootSceneNode;
	Ogre::Light * mainLight;
	
	OIS::InputManager * inputSystem;
	OIS::Mouse * mouse;
	OIS::Keyboard * keyboard;
	
	bool shutdown;
	int windowMaxWidth;
	int windowMaxHeight;
	
	int reticleNo;
	
	bool pickOn;
	bool menuOn;
	
	// No CEGUI in this release
	//CEGUI::Window * winRoot;
	//CEGUI::OgreRenderer* CErenderer;
	
	//void setupCEGUIEventHandlers();
	//bool handleBtnElements(const CEGUI::EventArgs& e);
	//bool handleBtnTopics(const CEGUI::EventArgs& e);
	//bool handleBtnScripts(const CEGUI::EventArgs& e);
	//bool handleBtnPick(const CEGUI::EventArgs& e);
	//bool handleBtnSettings(const CEGUI::EventArgs& e);
	//void toggleLocalMenu();
	
	void publishStatus(int _statusNo, string _status);
	void publishStatus(int _statusNo, string source, string _status);
	
	string status;
	int statusNo;
	
	// Subscription parameters
	string currentMarkerTopic;
	string markerTopic;
	string currentMarkerArrayTopic;
	string markerArrayTopic;
	string currentMarkerScriptTopic;
	string markerScriptTopic;
	
	// ROS subscribers
	ros::Subscriber markerSub;
	ros::Subscriber markerArraySub;
	ros::Subscriber markerScriptSub;
	
	// CRI's main Python instance
	//boost::python::object main_module; // Python instance
	//boost::python::object main_namespace; // Instance dictionary
	
	PyBlock * pyQtInterfaceMain;
	PyBlock * pyQtInterfaceExec;
	PyBlock * pyQtMarkerExec;

	// General parameters for Python interface
	string packagePath;
	string mainWindowId;
	
	string parm1;
	string parm2;

	static void die(void * classData);
	static void windowChanged(void * classData);
	static void setPickOn(void * classData);
	// getFrameParent() is there to add this functionality to the Python UI
	static void getFrameParent(void * classData);
	static void addTopic(void * classData);
	static void addReticle(void * classData);
	static void addGrid(void * classData);
	static void deleteGadget(void * classData);
	static void setKeyPressedAssignment(void * classData);
	static void getKeyPressedAssignment(void * classData);
	static void setKeyReleasedAssignment(void * classData);
	static void getKeyReleasedAssignment(void * classData);
	
	static void getMousePosition(void * classData);
	
};

#include "GActiveMarker.h"
#include "GFrame.h"
#include "GGrid.h"
#include "GGridCells.h"
#include "GPath.h"
#include "GPointCloud.h"
#include "GReticle.h"
#include "GTextureQuad.h"
#include "GQuiver.h"

#endif
