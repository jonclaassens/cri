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

# Lazy importing
from PyQt4 import QtGui
from PyQt4 import QtCore

class GeneralSettingsDialog(QtGui.QDialog):
    
	def __init__(self, parent, Portal, childName):
		QtGui.QDialog.__init__(self, parent)
        
		self.childName = childName
		self.Portal = Portal
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()
        
	def initUI(self):
        
		self.grid = QtGui.QGridLayout()

		if self.childName == '':
			self.parms = self.Portal.listProperties()
		else:
			self.parms = self.Portal.listChildProperties(self.childName)
		
		pos = 0
		self.eds = []
		
		for x in self.parms:
			
			# Is the global general settings, or for a gadget
			if self.childName == '': # global
				
				tempLE = QtGui.QLineEdit(self.Portal.get(x))
			else: # gadget
				
				if x == 'type':
					value = int(self.Portal.getChild(self.childName, x))
					
					result = {
						1: lambda x: "Grid",
						2: lambda x: "Frame",
						3: lambda x: "Reticle",
						4: lambda x: "Reticle",
						5: lambda x: "Marker",
						10: lambda x: "Point Cloud",
						11: lambda x: "Path",
						12: lambda x: "Grid Cells",
						13: lambda x: "Texture Quad",
						14: lambda x: "Laser Scan",
						15: lambda x: "Quiver",
						16: lambda x: "Odometry",
						17: lambda x: "Pose Stamped"
					}[value](x)
					
					tempLE = QtGui.QLabel(result)
					self.grid.addWidget(tempLE, pos, 1)
					self.grid.addWidget(QtGui.QLabel('Gadget Type'), pos, 0)
					
					self.eds.append(None)
					
					pos = pos + 1
					
				elif x == 'statusNo':
					self.eds.append(None)
					pass
				
				elif x == 'status':
					statusCode = int(self.Portal.getChild(self.childName, 'statusNo'))
					value = self.Portal.getChild(self.childName, x)

					tempLE = QtGui.QLabel('')
					self.grid.addWidget(tempLE, pos, 1)
					
					self.eds.append(None)
					
					if statusCode == -2:
						tempLE.setText("<font color='red'>" + value + "</font>");
					elif statusCode == -1:
						tempLE.setText("<font color='yellow'>" + value + "</font>");
					elif statusCode == 0:
						tempLE.setText("<font color='green'>" + value + "</font>");
						
					self.grid.addWidget(QtGui.QLabel('Status'), pos, 0)
				
					pos = pos + 1
					
				else:
					tempLE = QtGui.QLineEdit(self.Portal.getChild(self.childName, x))
					
					self.connect(tempLE, QtCore.SIGNAL("textChanged(const QString&)"),
					self.evListChanged)
			
					self.eds.append(tempLE)
					
					self.grid.addWidget(tempLE, pos, 1)
					
					self.grid.addWidget(QtGui.QLabel(x), pos, 0)
					
					pos = pos + 1
			
			
		self.setLayout(self.grid)

		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)
		
		self.setWindowTitle('Settings')    
		self.show()
		
	def evListChanged(self, value):
		if self.childName == '':
			for x in range(len(self.parms)):
				if self.eds[x] != None:
						self.Portal.set(self.parms[x], str(self.eds[x].text()))
		else:
			for x in range(len(self.parms)):
				if self.eds[x] != None:
					self.Portal.setChild(self.childName, self.parms[x], str(self.eds[x].text()))
			
