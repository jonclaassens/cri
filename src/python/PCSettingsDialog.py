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

class PCSettingsDialog(QtGui.QDialog):
    
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
		# TODO: Type of point cloud (PCL, PointCloud2, etc)
		self.grid.addWidget(QtGui.QLabel('Point Cloud'), 0, 1)
		
		self.grid.addWidget(QtGui.QLabel('Billboard shape'), 1, 0)
		self.CBShape = QtGui.QComboBox()
		self.CBShape.addItem('Point')
		self.CBShape.addItem('Circle')
		self.CBShape.addItem('Diamond')
		self.CBShape.addItem('Cross')
		self.CBShape.setCurrentIndex(int(self.Portal.getChild(self.childName, 'BillboardType')))
		self.grid.addWidget(self.CBShape, 1, 1)	
		self.connect(self.CBShape, QtCore.SIGNAL("currentIndexChanged(const QString&)"),
			self.evValueChanged)
		
		self.grid.addWidget(QtGui.QLabel('Alpha'), 2, 0)
		self.LEAlpha = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Alpha'))
		self.grid.addWidget(self.LEAlpha, 2, 1)
		self.connect(self.LEAlpha, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)
		
		self.grid.addWidget(QtGui.QLabel('Billboard Width'), 3, 0)
		self.LEWidth = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'BillboardWidth'))
		self.grid.addWidget(self.LEWidth, 3, 1)
		self.connect(self.LEWidth, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)
			
		self.grid.addWidget(QtGui.QLabel('Persistence (s)'), 4, 0)
		self.LEPersist = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Persistence'))
		self.grid.addWidget(self.LEPersist, 4, 1)
		self.connect(self.LEPersist, QtCore.SIGNAL("textChanged(const QString&)"),
			self.evValueChanged)
		
		self.grid.addWidget(QtGui.QLabel('Colour Transform'), 5, 0)
		self.CBTransform = QtGui.QComboBox()
		self.CBTransform.addItem('None')
		self.CBTransform.addItem('Flat')
		self.CBTransform.addItem('Channel Dependent')
		self.CBTransform.addItem('RGB 1 Channel')
		self.CBTransform.addItem('RGB 3 Channel')
		self.colourTrans = int(self.Portal.getChild(self.childName, 'ColourTransform'))
		self.CBTransform.setCurrentIndex(self.colourTrans)
		self.grid.addWidget(self.CBTransform, 5, 1)
		self.connect(self.CBTransform, QtCore.SIGNAL("currentIndexChanged(const QString&)"),
			self.evValueChanged)
		
		if self.colourTrans == 1: # Flat
			self.grid.addWidget(QtGui.QLabel('Colour'), 6, 0)
		
			scFrame = QtGui.QFrame()
			scRGBGrid = QtGui.QGridLayout()
			self.grid.addWidget(scFrame, 6, 1)
			scFrame.setLayout(scRGBGrid)
			
			self.SCR = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'StartColourR'))
			scRGBGrid.addWidget(self.SCR, 0, 0)
			self.connect(self.SCR, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			self.SCG = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'StartColourG'))
			scRGBGrid.addWidget(self.SCG, 0, 1)
			self.connect(self.SCG, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			self.SCB = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'StartColourB'))
			scRGBGrid.addWidget(self.SCB, 0, 2)
			self.connect(self.SCB, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			
		elif self.colourTrans == 2: # Channel dependent colouring
			self.grid.addWidget(QtGui.QLabel('Input Channel'), 6, 0)
			self.LEInChannel = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'ColourChannel'))
			self.grid.addWidget(self.LEInChannel, 6, 1)
			self.connect(self.LEInChannel, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			
			self.grid.addWidget(QtGui.QLabel('Auto-Calculate Bounds'), 7, 0)
			self.CBAuto = QtGui.QCheckBox('On')
			self.grid.addWidget(self.CBAuto, 7, 1)
			self.connect(self.CBAuto, QtCore.SIGNAL("stateChanged(int)"),
				self.evStateChanged)
			
			self.grid.addWidget(QtGui.QLabel('Minimum Bound'), 8, 0)
			self.LEMinBound = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'MinimumBound'))
			self.grid.addWidget(self.LEMinBound, 8, 1)
			self.connect(self.LEMinBound, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			
			self.grid.addWidget(QtGui.QLabel('Minimum Colour'), 9, 0)
			scFrame = QtGui.QFrame()
			scRGBGrid = QtGui.QGridLayout()
			self.grid.addWidget(scFrame, 9, 1)
			scFrame.setLayout(scRGBGrid)
			
			self.SCR = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'StartColourR'))
			scRGBGrid.addWidget(self.SCR, 0, 0)
			self.connect(self.SCR, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			self.SCG = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'StartColourG'))
			scRGBGrid.addWidget(self.SCG, 0, 1)
			self.connect(self.SCG, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			self.SCB = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'StartColourB'))
			scRGBGrid.addWidget(self.SCB, 0, 2)
			self.connect(self.SCB, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			
			self.grid.addWidget(QtGui.QLabel('Maximum Bound'), 10, 0)
			self.LEMaxBound = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'Maximum Bound'))
			self.grid.addWidget(self.LEMaxBound, 10, 1)
			self.connect(self.LEMaxBound, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			
			self.grid.addWidget(QtGui.QLabel('Maximum Colour'), 11, 0)
			scFrame = QtGui.QFrame()
			scRGBGrid = QtGui.QGridLayout()
			self.grid.addWidget(scFrame, 11, 1)
			scFrame.setLayout(scRGBGrid)
			
			self.ECR = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'EndColourR'))
			scRGBGrid.addWidget(self.ECR, 0, 0)
			self.connect(self.ECR, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			self.ECG = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'EndColourG'))
			scRGBGrid.addWidget(self.ECG, 0, 1)
			self.connect(self.ECG, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
			self.ECB = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'EndColourB'))
			scRGBGrid.addWidget(self.ECB, 0, 2)
			self.connect(self.ECB, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
    
			# Auto boundary calculation mode affects readability of max and min bounds line edits
			state = int(self.Portal.getChild(self.childName, 'AutoCalculateBounds'))
			if state == 0:
				self.CBAuto.setCheckState(QtCore.Qt.Unchecked)
				self.LEMaxBound.setReadOnly(False)
				self.LEMinBound.setReadOnly(False)
			else:
				self.CBAuto.setCheckState(QtCore.Qt.Checked)
				self.LEMaxBound.setReadOnly(True)
				self.LEMinBound.setReadOnly(True)
    
			# Create a timer to update the bounds line edits
			self.refreshTimer = QtCore.QTimer()
			self.connect(self.refreshTimer, QtCore.SIGNAL("timeout()"), self.refreshTimeout)
			self.refreshTimer.setInterval(1000)
			self.refreshTimer.setSingleShot(False)
			self.refreshTimer.start()
			self.haveTimer = 1
			
		elif self.colourTrans == 3: # RGB from one channel
		
			self.grid.addWidget(QtGui.QLabel('RGB Channel'), 6, 0)
			self.LERGBChannel = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'RGBChannel'))
			self.grid.addWidget(self.LERGBChannel, 6, 1)
			self.connect(self.LERGBChannel, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)

		elif self.colourTrans == 4: # RGB from one channel
		
			self.grid.addWidget(QtGui.QLabel('R Channel'), 6, 0)
			self.LERChannel = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'RChannel'))
			self.grid.addWidget(self.LERChannel, 6, 1)
			self.connect(self.LERChannel, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
				
			self.grid.addWidget(QtGui.QLabel('G Channel'), 7, 0)
			self.LEGChannel = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'GChannel'))
			self.grid.addWidget(self.LEGChannel, 7, 1)
			self.connect(self.LERChannel, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evValueChanged)
				
			self.grid.addWidget(QtGui.QLabel('B Channel'), 8, 0)
			self.LEBChannel = QtGui.QLineEdit(self.Portal.getChild(self.childName, 'BChannel'))
			self.grid.addWidget(self.LEBChannel, 8, 1)
			self.connect(self.LEBChannel, QtCore.SIGNAL("textChanged(const QString&)"),
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

		billboardType = self.CBShape.currentIndex()
		self.Portal.setChild(self.childName, 'BillboardType', str(billboardType))

		try:
			alpha = float(self.LEAlpha.text())
			if alpha > 1:
				alpha = 1
			elif alpha < 0:
				alpha = 0
			self.Portal.setChild(self.childName, 'Alpha', str(alpha))
		except:
			pass
	

		try:
			billboardWidth = float(self.LEWidth.text())
			if billboardWidth > 10:
				billboardWidth = 10
			elif billboardWidth < 0:
				billboardWidth = 0
			self.Portal.setChild(self.childName, 'BillboardWidth', str(billboardWidth))
		except:
			pass

		try:
			persistence = float(self.LEPersist.text())
			if persistence < 0:
				persistence = 0
			self.Portal.setChild(self.childName, 'Persistence', str(persistence))
		except:
			pass

		transform = self.CBTransform.currentIndex()
		self.Portal.setChild(self.childName, 'ColourTransform', str(transform))
		
		if self.colourTrans == transform:
			
			if transform == 1: # Flat

				try:
					scr = int(self.SCR.text())
					if scr > 255:
						scr = 255
					elif scr < 0:
						scr = 0
					self.Portal.setChild(self.childName, 'StartColourR', str(scr))
				except:
					pass
				
				try:
					scg = int(self.SCG.text())
					if scg > 255:
						scg = 255
					elif scg < 0:
						scg = 0
					self.Portal.setChild(self.childName, 'StartColourG', str(scg))
				except:
					pass
				
				try:
					scb = int(self.SCB.text())
					if scb > 255:
						scb = 255
					elif scb < 0:
						scb = 0
					self.Portal.setChild(self.childName, 'StartColourB', str(scb))
				except:
					pass
				
			elif transform == 2: # Channel dependent colouring
		
				self.Portal.setChild(self.childName, 'ColourChannel', str(self.LEInChannel.text()))

				if self.CBAuto.checkState() == QtCore.Qt.Checked:
					self.Portal.setChild(self.childName, 'AutoCalculateBounds', '1')
					self.LEMaxBound.setReadOnly(True)
					self.LEMinBound.setReadOnly(True)
				else:
					self.Portal.setChild(self.childName, 'AutoCalculateBounds', '0')
					self.LEMaxBound.setReadOnly(False)
					self.LEMinBound.setReadOnly(False)
				
				try:
					minBound = float(self.LEMinBound.text())
					self.Portal.setChild(self.childName, 'MinimumBound', str(minBound))
				except:
					pass

				try:
					scr = int(self.SCR.text())
					if scr > 255:
						scr = 255
					elif scr < 0:
						scr = 0
					self.Portal.setChild(self.childName, 'StartColourR', str(scr))
				except:
					pass
				
				try:
					scg = int(self.SCG.text())
					if scg > 255:
						scg = 255
					elif scg < 0:
						scg = 0
					self.Portal.setChild(self.childName, 'StartColourG', str(scg))
				except:
					pass
				
				try:
					scb = int(self.SCB.text())
					if scb > 255:
						scb = 255
					elif scb < 0:
						scb = 0
					self.Portal.setChild(self.childName, 'StartColourB', str(scb))
				except:
					pass

				try:
					maxBound = float(self.LEMaxBound.text())
					self.Portal.setChild(self.childName, 'MaximumBound', str(maxBound))
				except:
					pass
				
				try:
					ecr = int(self.ECR.text())
					if ecr > 255:
						ecr = 255
					elif ecr < 0:
						ecr = 0
					self.Portal.setChild(self.childName, 'EndColourR', str(ecr))
				except:
					pass
				
				try:
					ecg = int(self.ECG.text())
					if ecg > 255:
						ecg = 255
					elif ecg < 0:
						ecg = 0
					self.Portal.setChild(self.childName, 'EndColourG', str(ecg))
				except:
					pass

				try:
					ecb = int(self.ECB.text())
					if ecb > 255:
						ecb = 255
					elif ecb < 0:
						ecb = 0
					self.Portal.setChild(self.childName, 'EndColourB', str(ecb))
				except:
					pass
					
			elif transform == 3: # RGB from one channel
			
				self.Portal.setChild(self.childName, 'RGBChannel', str(self.LERGBChannel.text()))
			
			elif transform == 4: # RGB from one channel
			
				self.Portal.setChild(self.childName, 'RChannel', str(self.LERChannel.text()))
				self.Portal.setChild(self.childName, 'GChannel', str(self.LEGChannel.text()))
				self.Portal.setChild(self.childName, 'BChannel', str(self.LEBChannel.text()))
			

		else:
			self.refresh()
			self.layout().setSizeConstraint(QtGui.QLayout.SetFixedSize)
			self.update()

	def evStateChanged(self, value):
		self.evValueChanged("dummy")
		
	def refreshTimeout(self):
		if self.CBAuto.checkState() == QtCore.Qt.Checked:
			self.LEMaxBound.setText(self.Portal.getChild(self.childName, 'MaximumBound'))
			self.LEMinBound.setText(self.Portal.getChild(self.childName, 'MinimumBound'))
			
	def close(self):
		if self.haveTimer > 0:
			self.refreshTimer.stop()
			del self.refreshTimer
