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

class ReticleSettingsDialog(QtGui.QDialog):
    
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
    
		# Clear the widget
		deleteItems(self.grid.layout())
		
		
		# X, Y and Z
		self.grid.addWidget(QtGui.QLabel('X'), 0, 0)
		self.LEX = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'X'))
		self.grid.addWidget(self.LEX, 0, 1)
		self.connect(self.LEX, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)
			
		self.grid.addWidget(QtGui.QLabel('Y'), 1, 0)
		self.LEY = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Y'))
		self.grid.addWidget(self.LEY, 1, 1)
		self.connect(self.LEY, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)
		
		self.grid.addWidget(QtGui.QLabel('Z'), 2, 0)
		self.LEZ = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Z'))
		self.grid.addWidget(self.LEZ, 2, 1)
		self.connect(self.LEZ, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)


		# Azimuth, zenith and roll
		self.grid.addWidget(QtGui.QLabel('Azimuth (radians)'), 0, 2)
		self.LEAzimuth = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Azimuth'))
		self.grid.addWidget(self.LEAzimuth, 0, 3)
		self.connect(self.LEAzimuth, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)
			
		self.grid.addWidget(QtGui.QLabel('Zenith'), 1, 2)
		self.LEZenith = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Zenith'))
		self.grid.addWidget(self.LEZenith, 1, 3)
		self.connect(self.LEZenith, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)
		
		self.grid.addWidget(QtGui.QLabel('Roll'), 2, 2)
		self.LERoll = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Roll'))
		self.grid.addWidget(self.LERoll, 2, 3)
		self.connect(self.LERoll, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)

		# TF functionality
		self.grid.addWidget(QtGui.QLabel('Broadcast TF frame'), 3, 0)
		self.CBBroadcast = QtGui.QCheckBox('On')
		self.grid.addWidget(self.CBBroadcast, 3, 1)
		self.connect(self.CBBroadcast, QtCore.SIGNAL("stateChanged(int)"),
			self.evStateChanged)
			
		state = int(self.Portal.getChild(self.childName, 'BroadcastOn'))
		if state == 0:
			self.CBBroadcast.setCheckState(QtCore.Qt.Unchecked)
		else:
			self.CBBroadcast.setCheckState(QtCore.Qt.Checked)
		
		self.grid.addWidget(QtGui.QLabel('Frame name'), 4, 0)
		self.LEFrame = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'TFFrame'))
		self.grid.addWidget(self.LEFrame, 4, 1)
		self.connect(self.LEFrame, QtCore.SIGNAL("textChanged(const QString&)"),
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

		# Position
		try:
			x = float(self.LEX.text())
			self.Portal.setChild(self.childName, 'X', str(x))
		except:
			pass	
			
		try:
			y = float(self.LEY.text())
			self.Portal.setChild(self.childName, 'Y', str(y))
		except:
			pass
			
		try:
			z = float(self.LEZ.text())
			self.Portal.setChild(self.childName, 'Z', str(z))
		except:
			pass

		# Azimuth, zenith and roll
		try:
			a = float(self.LEAzimuth.text())
			self.Portal.setChild(self.childName, 'Azimuth', str(a))
		except:
			pass	
			
		try:
			z = float(self.LEZenith.text())
			self.Portal.setChild(self.childName, 'Zenith', str(z))
		except:
			pass
			
		try:
			r = float(self.LERoll.text())
			self.Portal.setChild(self.childName, 'Roll', str(r))
		except:
			pass


		# Broadcast?
		if self.CBBroadcast.checkState() == QtCore.Qt.Checked:
			self.Portal.setChild(self.childName, 'BroadcastOn', '1')
		else:
			self.Portal.setChild(self.childName, 'BroadcastOn', '0')

		# Broadcast TF Frame name 
		self.Portal.setChild(self.childName, 'TFFrame', str(self.LEFrame.text()))

		#else:
		#	self.refresh()
		#	self.layout().setSizeConstraint(QtGui.QLayout.SetFixedSize)
		#	self.update()

	def evStateChanged(self, value):
		self.evValueChanged("dummy")
		
	def refreshTimeout(self):
		if self.CBAuto.checkState() == QtCore.Qt.Checked:
			self.LEMaxBound.setText(self.Portal.getChild(self.childName, 'MaximumBound'))
			self.LEMinBound.setText(self.Portal.getChild(self.childName, 'MinimumBound'))
			
