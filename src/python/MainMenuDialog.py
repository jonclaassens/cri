from PyQt4 import QtGui
from PyQt4 import QtCore
#import xml.etree.ElementTree as xml
import lxml.etree as xml
from LoadState import loadState
from SaveState import saveState

from LaunchPropertiesDialog import launchPropertiesDialog

class MainMenuDialog(QtGui.QDialog):
    
	def __init__(self, parent, Portal, r):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
		self.r = r
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()

	def detachCamClicked(self):
		
		self.Portal.set('CamMode', '0')
		
		self.close()

	def propertiesClicked(self):
		
		activeName = self.Portal.get('ActiveGadgetName')
		
		
		
		if activeName != "":
			childType = int(self.Portal.getChild(activeName, "type"))
			
			launchPropertiesDialog(activeName, childType, self.r)
			self.close()	
			return
		
		self.close()

	def addTopicClicked(self):
		
		# Create a worker thread to initialize the topics dialog
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit(QtCore.SIGNAL("runAddTopic"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runAddTopic"), self.r.runAddTopic)
		workThread.start()
		
		workThread.wait()
		
		self.close()

		
	def sceneElementsClicked(self):
		
		# Create a worker thread to initialize the scene elements dialog
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runListGadgets"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runListGadgets"), self.r.runListGadgets)
		workThread.start()
		
		workThread.wait()
		
		self.close()

		
	def TFFramesClicked(self):
		
		# Create a worker thread to initialize the scene elements dialog
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runTFFrames"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runTFFrames"), self.r.runTFFrames)
		workThread.start()
		
		workThread.wait()
		
		self.close()
		
		
	def pickClicked(self):
		
		self.Portal.execCB('SetPickOn')
		
		self.close()		
		
	def settingsClicked(self):
		
		# Create a worker thread to initialize the key assignment dialog
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runSettingsMenu"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runSettingsMenu"), self.r.runSettingsMenu)
		workThread.start()
		
		workThread.wait()
		
		self.close()	
		
	def statusClicked(self):
		
		# Create a worker thread to initialize the key assignment dialog
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runStatus"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runStatus"), self.r.runStatus)
		workThread.start()
		
		workThread.wait()
		
		self.close()

	def saveClicked(self):
		
		if saveState(self.Portal, self.r.saveFileName) == True:
			
			self.close()
		
		else:
			QtGui.QMessageBox.critical(self,
				"Critical",
				"Could not save to file '" + self.r.saveFileName + "'.")
		
	def saveAsClicked(self):
		
		fileName = str(QtGui.QFileDialog.getSaveFileName(self, 'Save file', '/home', 'XML files (*.xml)'))
		
		if fileName == "":
			return
		
		if saveState(self.Portal, fileName) == True:
			
			self.r.saveFileName = fileName
			self.close()
		
		else:
			QtGui.QMessageBox.critical(self,
				"Critical",
				"Could not save to file '" + fileName + "'.")
		
	def loadClicked(self):	
		
		fileName = str(QtGui.QFileDialog.getOpenFileName(self, 'Open file', '/home', 'XML files (*.xml)'))
		
		if fileName == "":
			return
		
		if loadState(self.Portal, fileName) == True:
			
			self.r.saveFileName = fileName
			
			self.close()

		else:
			QtGui.QMessageBox.critical(self,
				"Critical",
				"Could not open file '" + fname + "'.")

	def initUI(self):
		
		# Layout the widget

		self.parentGrid = QtGui.QGridLayout()
        
		row = 0
		if int(self.Portal.get('CamMode')) != 0:
			button = QtGui.QPushButton('Detach Camera')
			self.parentGrid.addWidget(button, row, 0)
			row = row + 1
			button.clicked.connect(self.detachCamClicked)
			
			line = QtGui.QFrame();
			line.setObjectName("line");
			line.setFrameShape(QtGui.QFrame.HLine);
			line.setFrameShadow(QtGui.QFrame.Sunken);
			self.parentGrid.addWidget(line, row, 0)
			row = row + 1
			
		activeName = self.Portal.get('ActiveGadgetName')
		if activeName != "":
			button = QtGui.QPushButton('Gadget Properties')
			self.parentGrid.addWidget(button, row, 0)
			row = row + 1
			button.clicked.connect(self.propertiesClicked)
			
			line = QtGui.QFrame();
			line.setObjectName("line");
			line.setFrameShape(QtGui.QFrame.HLine);
			line.setFrameShadow(QtGui.QFrame.Sunken);
			self.parentGrid.addWidget(line, row, 0)
			row = row + 1
			
        
		button = QtGui.QPushButton('Topics')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.addTopicClicked)
		
		button = QtGui.QPushButton('Scene Elements')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.sceneElementsClicked)
		
		button = QtGui.QPushButton('TF Frames')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.TFFramesClicked)
		
		button = QtGui.QPushButton('Pick')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.pickClicked)
		
		line = QtGui.QFrame();
		line.setObjectName("line");
		line.setFrameShape(QtGui.QFrame.HLine);
		line.setFrameShadow(QtGui.QFrame.Sunken);
		self.parentGrid.addWidget(line, row, 0)
		row = row + 1
		
		button = QtGui.QPushButton('Settings')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.settingsClicked)
		
		button = QtGui.QPushButton('Status')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.statusClicked)
		
		line = QtGui.QFrame();
		line.setObjectName("line");
		line.setFrameShape(QtGui.QFrame.HLine);
		line.setFrameShadow(QtGui.QFrame.Sunken);
		self.parentGrid.addWidget(line, row, 0)
		row = row + 1
			
		button = QtGui.QPushButton('Save')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.saveClicked)
		
		button = QtGui.QPushButton('Save As...')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.saveAsClicked)
		
		button = QtGui.QPushButton('Load')
		self.parentGrid.addWidget(button, row, 0)
		row = row + 1
		button.clicked.connect(self.loadClicked)
		
		self.setLayout(self.parentGrid)  
		# self.setWindowFlags(QtCore.Qt.FramelessWindowHint)  
		self.setWindowTitle('Menu') 
		
		self.Portal.execCB('getMousePosition')
		mouseX = int(self.Portal.get('parm1'))
		mouseY = int(self.Portal.get('parm2'))
		self.move(mouseX, mouseY)
		
		# Set up focus lost, kill window event (this doesn't work)
		# self.setFocusPolicy(QtCore.Qt.StrongFocus)

		# Show the result
		self.show()
		

