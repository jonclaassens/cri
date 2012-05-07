import os
import sys
import itertools
import math
import socket
import time
import traceback
import yaml
import xmlrpclib
import functools

from LaunchPropertiesDialog import launchPropertiesDialog

# Lazy importing
from PyQt4 import QtGui
from PyQt4 import QtCore

class ListGadgetsDialog2(QtGui.QDialog):
    
	def properties(self):
		
		selected = self.TWGadgets.currentItem()
		if selected != None:
			# Is this a root (type) item with children?  If not, we may proceed to add the topic
			if selected.childCount() == 0:
				childName = str(selected.text(0))
			else:
				return
		else:
			return
		
		# Gadget type?
		childType = int(self.Portal.getChild(childName, "type"))
		
		launchPropertiesDialog(childName, childType, self.r)
		
	def delete(self, childName):

		selected = self.TWGadgets.currentItem()
		if selected != None:
			# Is this a root (type) item with children?  If not, we may proceed to add the topic
			if selected.childCount() == 0:
				name = selected.text(0)

				# Kill selected gadget
				self.Portal.set("parm1", str(name))
				self.Portal.execCB("DeleteGadget")
		
		self.refresh()
    
	def importURDF(self):
    
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runImportURDF"))

				return

		workThread = WorkThread()
		#QtCore.QObject.connect(workThread, QtCore.SIGNAL("runImportURDF"), self.r.runImportURDF)
		
		workThread.start()
		workThread.wait()
    
	def refresh(self):
		
		# Clear the TreeWidget
		self.TWGadgets.clear()
		
		self.typeWidgets = dict()
		
		# List the gadget types
		for k in self.typeStrings.keys():
			root = QtGui.QTreeWidgetItem([self.typeStrings[k]])
			self.TWGadgets.addTopLevelItem(root)
			self.typeWidgets[k] = root
			
		# And a catchall
		self.nodeCatchAll = QtGui.QTreeWidgetItem(['Unknown'])
		self.TWGadgets.addTopLevelItem(self.nodeCatchAll)
		
		# Get all the current gadgets
		children = self.Portal.listChildren()
		
		# Set up some colors for the status messages
		red = QtGui.QColor()
		red.setRgbF(1, 0, 0)
		
		yellow = QtGui.QColor()
		yellow.setRgbF(0.8, 0.5, 0)
		
		green = QtGui.QColor()
		green.setRgbF(0, 0.7, 0)
		
		for c in children:
			
			typeNo = int(self.Portal.getChild(c, 'type'))
			statusValue = self.Portal.getChild(c, 'status')
			
			element = QtGui.QTreeWidgetItem([c, c, statusValue])
			
			statusCode = int(self.Portal.getChild(c, 'statusNo'));
			if statusCode == -2:
				element.setTextColor(2, red)
			elif statusCode == -1:
				element.setTextColor(2, yellow)
			elif statusCode == 0:
				element.setTextColor(2, green)
			
			# Map type number to root, type widget
			try:
				targetType = self.typeWidgets[typeNo]
			except:
				typeNo = None
			
			if typeNo is None:
				self.nodeCatchAll.addChild(element)
				
			else:
				targetType.addChild(element)
		
		# self.frameTop.setFrameShape(QtGui.QFrame.StyledPanel)
		
		self.TWGadgets.setColumnWidth(0,250)
	
	
	def __init__(self, parent, Portal, r):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
		self.r = r
        
		self.typeStrings = dict()
		self.typeStrings[1] = "Grids"
		self.typeStrings[2] = "Frames"
		self.typeStrings[3] = "Camera References"
		self.typeStrings[4] = "Reticles"
		self.typeStrings[5] = "Active Markers"
		self.typeStrings[10] = "Point Clouds"
		self.typeStrings[11] = "Paths"
		self.typeStrings[12] = "Grid Cells"
		self.typeStrings[13] = "Occupancy Grids"
		self.typeStrings[14] = "Texture Quad"
		self.typeStrings[15] = "Quiver"
		self.typeStrings[16] = "Odometry"
		self.typeStrings[17] = "Pose Stamped"
		self.typeStrings[18] = "Pose Array"
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()
	
	def initUI(self):
		
		# Layout the widget
        
		self.gridout = QtGui.QGridLayout()
        
		self.frameTop = QtGui.QFrame()
		self.frameBot = QtGui.QFrame()
		
		self.gridTop = QtGui.QGridLayout()
		self.frameTop.setLayout(self.gridTop)
		
		self.gridBot = QtGui.QGridLayout()
		self.frameBot.setLayout(self.gridBot)
		
		self.gridout.addWidget(self.frameTop, 0, 0)
		self.gridout.addWidget(self.frameBot, 1, 0)

		# Add the refresh button

		button = QtGui.QPushButton('Refresh')
		self.gridBot.addWidget(button, 0, 0)
		button.clicked.connect(self.refresh)
		
		# Add the refresh button

		button = QtGui.QPushButton('Properties')
		self.gridBot.addWidget(button, 0, 1)
		button.clicked.connect(self.properties)
		
		# Add the delete button
		
		button = QtGui.QPushButton('Delete')
		self.gridBot.addWidget(button, 0, 2)
		button.clicked.connect(self.delete)
		
		# Add the 'import URDF' button
		
		button = QtGui.QPushButton('Import URDF')
		self.gridBot.addWidget(button, 0, 3)
		button.clicked.connect(self.importURDF)
		
		# Add the close button
		
		button = QtGui.QPushButton('Close')
		self.gridBot.addWidget(button, 0, 4)
		button.clicked.connect(self.close)

		self.setLayout(self.gridout)   
			
		self.setWindowTitle('Select gadget to appraise:') 

		# Add the topic tree view
		
		self.TWGadgets = QtGui.QTreeWidget()
		self.TWGadgets.setColumnCount(2)
		self.TWGadgets.setHeaderLabels(['Name', 'Topic', 'Status'])
		self.gridTop.addWidget(self.TWGadgets, 0, 0)

		# Move the window to where the menu was
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)

		# Update the data

		self.refresh()
		self.resize(640, 480)
		
		# Show the result
		 
		self.show()
		

