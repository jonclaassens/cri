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

class QuiverSettingsDialog(QtGui.QDialog):
    
	def __init__(self, parent, Portal, childName):
		QtGui.QDialog.__init__(self, parent)
        
		self.childName = childName
		self.Portal = Portal
        
		self.haveTimer = 0
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()
        
	def refresh(self):
		def deleteItems(layout): 
			if layout is not None: 
				while layout.count(): 
					item = layout.takeAt(0) 
					widget = item.widget() 
					if widget is not None: 
						widget.deleteLater() 
					else: 
						deleteItems(item.layout()) 	
    
		deleteItems(self.grid.layout())

		if self.haveTimer == 1:
			del self.refreshTimer
			self.haveTimer = 0
    
		self.grid.addWidget(QtGui.QLabel('Type'), 0, 0)

		self.typeNo = int(self.Portal.getChild(self.childName, 'type'))
		
		if self.typeNo == 15:	
			self.grid.addWidget(QtGui.QLabel('Quiver'), 0, 1)
		elif self.typeNo == 16:
			self.grid.addWidget(QtGui.QLabel('Odometry'), 0, 1)
		elif self.typeNo == 17:
			self.grid.addWidget(QtGui.QLabel('Pose Stamped'), 0, 1)
		elif self.typeNo == 18:
			self.grid.addWidget(QtGui.QLabel('Pose Array'), 0, 1)
		
		row = 1
		
		self.grid.addWidget(QtGui.QLabel('Array Scale'), row, 0)
		self.LEScale = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Scale'))
		self.grid.addWidget(self.LEScale, row, 1)
		self.connect(self.LEScale, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)
			
		row = row + 1
		
		if self.typeNo == 16:
			self.grid.addWidget(QtGui.QLabel('History Length (in number)'), row, 0)
			self.LENumberLimit = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'CountLimit'))
			self.grid.addWidget(self.LENumberLimit, row, 1)
			self.connect(self.LENumberLimit, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
				
			row = row + 1
			
			self.grid.addWidget(QtGui.QLabel('Position Tolerance'), row, 0)
			self.LEPositionTol = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'PositionTolerance'))
			self.grid.addWidget(self.LEPositionTol, row, 1)
			self.connect(self.LEPositionTol, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
				
			row = row + 1
			
			self.grid.addWidget(QtGui.QLabel('Angle Tolerance'), row, 0)
			self.LEAngleTol = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'AngleTolerance'))
			self.grid.addWidget(self.LEAngleTol, row, 1)
			self.connect(self.LEAngleTol, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			
			row = row + 1
				
		if self.typeNo == 16 or self.typeNo == 17 or self.typeNo == 18:
			self.grid.addWidget(QtGui.QLabel('Arrow Colour (R, G, B [0..255])'), row, 0)
			
			rgbFrame = QtGui.QFrame()
			self.RGBGrid = QtGui.QGridLayout()
			self.grid.addWidget(rgbFrame, row, 1)
			row = row + 1
			rgbFrame.setLayout(self.RGBGrid)
			
			self.LER = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'ArrowColourR'))
			self.RGBGrid.addWidget(self.LER, 0, 0)
			self.connect(self.LER, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
				
			self.LEG = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'ArrowColourG'))
			self.RGBGrid.addWidget(self.LEG, 0, 1)
			self.connect(self.LEG, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
				
			self.LEB = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'ArrowColourB'))
			self.RGBGrid.addWidget(self.LEB, 0, 2)
			self.connect(self.LEB, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)

	def initUI(self):
        
		self.grid = QtGui.QGridLayout()		
		self.setLayout(self.grid)
		
		self.refresh()

		# Move the window to where the menu was
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)
		
		self.setWindowTitle('Point Cloud Settings')    
		self.show()
		
	def evValueChanged(self, value):

		try:
			scale = float(self.LEScale.text())
			if scale >= 0:
				self.Portal.setChild(self.childName, 'Scale', str(scale))
		except:
			pass


		if self.typeNo == 16:
			try:
				countLimit = int(self.LENumberLimit.text())
				
				if countLimit >= 1:
					# This limit needs to be emphasized and not a keyword
					if countLimit <= 1000:
						self.Portal.setChild(self.childName, 'CountLimit', str(countLimit))
			except:
				pass
				
			try:
				posTol = float(self.LEPositionTol.text())
				
				if posTol >= 0:
					self.Portal.setChild(self.childName, 'PositionTolerance', str(posTol))
			except:
				pass
				
			try:
				angTol = float(self.LEAngleTol.text())
				
				if angTol >= 0:
					self.Portal.setChild(self.childName, 'AngleTolerance', str(angTol))
			except:
				pass
	
		if self.typeNo == 16 or self.typeNo == 17 or self.typeNo == 18:
			try:
				r = int(self.LER.text())
				if r > 255:
					r = 255
				elif r < 0:
					r = 0
				self.Portal.setChild(self.childName, 'ArrowColourR', str(r))
			except:
				pass

			try:
				g = int(self.LEG.text())
				if g > 255:
					g = 255
				elif g < 0:
					g = 0
				self.Portal.setChild(self.childName, 'ArrowColourG', str(g))
			except:
				pass
			
			try:
				b = int(self.LEB.text())
				if b > 255:
					b = 255
				elif b < 0:
					b = 0
				self.Portal.setChild(self.childName, 'ArrowColourB', str(b))
			except:
				pass

	# def evStateChanged(self, value):
	#	self.evValueChanged("dummy")
