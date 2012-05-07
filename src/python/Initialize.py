import os
import sys
import itertools
import math
import socket
import time
import traceback
import yaml
import xmlrpclib
import thread

sys.argv = [''];

# Totally lazy importing
import roslib

roslib.load_manifest('rospy')
roslib.load_manifest('rosgraph')
roslib.load_manifest('tf')

import rospy
import rosgraph.masterapi
import tf


# More lazy importing
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4 import QtOpenGL


# Import all the necessary dialogs
sys.path.append(roslib.packages.get_pkg_dir('cri') + '/src/python/')
import AddTopicDialog2
import ListGadgetsDialog2
import ListSelectionsDialog
import GeneralSettingsDialog
import PickAssignDialog
import KeyAssignDialog
import MainMenuDialog
import TFFramesDialog
import PCSettingsDialog
import QuiverSettingsDialog
import ReticleSettingsDialog
import SettingsMenuDialog
import StatusDialog

import LoadState

# Create our main window
class MainWindow(QtGui.QMainWindow):
	
	def __init__(self, Portal, parent = None):   
                
		QtGui.QMainWindow.__init__(self, parent)      
		self.Portal = Portal 

	def closeEvent(self, event):
		
		# Tell CRI c++ core to die
		self.Portal.execCB('Die')

class RenderWidget(QtOpenGL.QGLWidget):

	def __init__(self, Portal, parent = None):   
                
		QtOpenGL.QGLWidget.__init__(self, parent)      
		self.Portal = Portal             
                
	def initializeGL(self):                

		self.Portal.set('MainWindowId', str(int(self.winId())))   
		
		# Tell CRI c++ the window size
		self.Portal.set('parm1', str(self.width()))
		self.Portal.set('parm2', str(self.height()))
		
		# We don't want to fight with qt on how to update this widget. 
		# It is the CRI core's responsibility
		self.setUpdatesEnabled(False)        
        
	def resizeGL(self, width, height):
		
		# Tell CRI c++ part that the widget has resized
		self.Portal.set('parm1', str(self.width()))
		self.Portal.set('parm2', str(self.height()))
		self.Portal.execCB('WindowChanged')
		
		# Delay a bit so that we update after we're sure the renderer has accepted a size change
		time.sleep(0.1)
		
		# Repaint over anything left on the edges
		self.update()
                
	def paintGL(self): 
		pass
		
		# Need callback here
		# self.Ogre.renderOneFrame()

class StatusMessage:
	def __init__(self):
		self.source = ""
		self.message = ""
		self.statusNo = 0
		self.no = 0
		self.time = rospy.Time.now()
		
		
	def __init__(self, no, time, statusSource, status, statusNo):
		self.no = no
		self.time = time
		self.source = statusSource
		self.message = status
		self.statusNo = statusNo


class MainScript(QtCore.QObject):
	def __init__(self, application, parentWindow):
		self.app = application
		self.parentWindow = parentWindow
		QtCore.QObject.__init__(self)
		
		# Get the home directory, and set a default save-state file
		home = os.getenv("HOME")
		self.saveFileName = home + '/.CRI.xml'
		
		# Initialize a tf listener
		self.tf = tf.TransformListener()
		
		# Initialize default pick scripts
		self.typeToName = {1: 'Grid', 
			2: 'Frame',
			3: 'Reticle',
			4: 'Reticle',
			5: 'Marker',
			10: 'Point Cloud',
			11: 'Path',
			12: 'Grid Cells',
			13: 'Texture Quad',
			14: 'Laser Scan',
			15: 'Quiver',
			16: 'Odometry',
			17: 'Pose Stamped',
			18: 'Pose Array'}
		
		self.pickTypeToScriptDict = dict()
		rootPath = roslib.packages.get_pkg_dir('cri')
		
		# frameOptions = {'Attach' : 'AttachToFrame.py', 
		#	'Orbit' : 'OrbitFrame.py'}
		
		self.pickTypeToScriptDict[2] = frameOptions
		
		gridOptions = {'Orbit' : rootPath + '/src/python/DefaultPickScripts/OrbitGridPoint.py',
			'Place Gimbal' : rootPath + '/src/python/DefaultPickScripts/SelectDirection.py'}
		
		self.pickTypeToScriptDict[1] = gridOptions
		
		# markerOptions = {'beer' : 'beer.py'}
		# self.pickTypeToScriptDict[5] = markerOptions
		
		self.dialogMainMenu = None
		self.dialogAddTopic = None
		self.dialogTFFrames = None
		self.dialogListGadgets = None
		self.dialogListSelections = None
		self.dialogGeneralSettings = None
		self.dialogPCSettings = None
		self.dialogQuiverSettings = None
		self.dialogReticleSettings = None
		self.dialogKeyAssign = None
		self.dialogPickAssign = None
		self.dialogSettingsMenu = None
		self.dialogStatus = None

		self.maxStatusMessages = 1000

		# Allocate mutex for status messages
		self.statusMutex = thread.allocate_lock()
		self.statusMessages = []
		self.msgNo = 0
		
		# try:
		#	loadState(self.Portal, self.saveFileName)
		# except:
		#	pass

	def publishStatus(self, statusSource, status, statusNo):
		
		self.statusMutex.acquire()
		
		self.msgNo = self.msgNo + 1
		msg = StatusMessage(self.msgNo, rospy.Time.now(), statusSource, status, statusNo)
		self.statusMessages.append(msg)
		
		self.statusMutex.release()
		
		if len(self.statusMessages) > self.maxStatusMessages:
			del self.statusMessages[0]

	def runMainMenu(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogMainMenu != None:
			try:
				self.dialogMainMenu.close()
			except:
				pass
			
		self.dialogMainMenu = MainMenuDialog.MainMenuDialog(self.parentWindow, Portal, self)
		return	
	
	def runAddTopic(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogAddTopic != None:
			try:
				self.dialogAddTopic.close()
			except:
				pass
		
		self.dialogAddTopic = AddTopicDialog2.AddTopicGUI(self.parentWindow, Portal)
		
		#self.dialogAddTopic = QtGui.QDockWidget(self.parentWindow);
		#self.dialogAddTopic.setWidget(AddTopicDialog2.AddTopicGUI(self.dialogAddTopic, Portal))
		#self.parentWindow.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self.dialogAddTopic);
		
		return
		
	def runTFFrames(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogTFFrames != None:
			try:
				self.dialogTFFrames.close()
			except:
				pass
				
		self.dialogTFFrames = TFFramesDialog.TFFramesDialog(self.parentWindow, Portal, self.tf)
		return
		
	def runListGadgets(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogListGadgets != None:
			try:
				self.dialogListGadgets.close()
			except:
				pass
		
		self.dialogListGadgets = ListGadgetsDialog2.ListGadgetsDialog2(self.parentWindow, Portal, self)
		return
		
	def runListSelections(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogListSelections != None:
			try:
				self.dialogListSelections.close()
			except:
				pass
		
		self.dialogListSelections = ListSelectionsDialog.ListSelectionsDialog(self.parentWindow, Portal, self)
		return
		
	def runGeneralSettings(self, childName):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogGeneralSettings != None:
			try:
				self.dialogGeneralSettings.close()
			except:
				pass
		
		self.dialogGeneralSettings = GeneralSettingsDialog.GeneralSettingsDialog(self.parentWindow, Portal, childName)
		return
		
	def runPCSettings(self, childName):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogPCSettings != None:
			try:
				self.dialogPCSettings.close()
			except:
				pass
		
		self.dialogPCSettings = PCSettingsDialog.PCSettingsDialog(self.parentWindow, Portal, childName)
		return
		
	def runQuiverSettings(self, childName):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogQuiverSettings != None:
			try:
				self.dialogQuiverSettings.close()
			except:
				pass
		
		self.dialogQuiverSettings = QuiverSettingsDialog.QuiverSettingsDialog(self.parentWindow, Portal, childName)
		return
		
	def runReticleSettings(self, childName):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogReticleSettings != None:
			try:
				self.dialogReticleSettings.close()
			except:
				pass
		
		self.dialogReticleSettings = ReticleSettingsDialog.ReticleSettingsDialog(self.parentWindow, Portal, childName)
		return
		
	def runKeyAssign(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogKeyAssign != None:
			try:
				self.dialogKeyAssign.close()
			except:
				pass
		
		self.dialogKeyAssign = KeyAssignDialog.KeyAssignDialog(self.parentWindow, Portal)
		return
		
	def runPickAssign(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogPickAssign != None:
			try:
				self.dialogPickAssign.close()
			except:
				pass
		
		self.dialogPickAssign = PickAssignDialog.PickAssignDialog(self.parentWindow, Portal, self)
		return
		
	def runSettingsMenu(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogSettingsMenu != None:
			try:
				self.dialogSettingsMenu.close()
			except:
				pass
		
		self.dialogSettingsMenu = SettingsMenuDialog.SettingsMenuDialog(self.parentWindow, Portal, self)
		return
		
	def runStatus(self):
		
		# If the window has existed or exists, try to close it (and have it delete itself)
		if self.dialogStatus != None:
			try:
				self.dialogStatus.close()
			except:
				pass
		
		self.dialogStatus = StatusDialog.StatusDialog(self.parentWindow, Portal, self)
		return

	def runQuit(self):
		self.app.quit()
		return


rospy.init_node('PyPortal', anonymous=True, log_level=rospy.INFO, disable_signals=True)

app = QtGui.QApplication([''])
app.setQuitOnLastWindowClosed(False)

# Show main window
mainWindow = MainWindow(Portal)
r = MainScript(app, mainWindow)

mainWindow.setWindowTitle("Test")
mainWindow.setGeometry(20,20,800,800)
mainWindow.setCentralWidget(RenderWidget(Portal))

#grid = QtGui.QGridLayout()   
#grid.addWidget(RenderWidget(Portal), 0, 0)
#grid.addWidget(QtGui.QLabel('Something here'), 1, 0)
#mainWindow.setLayout(grid)

mainWindow.show()   
