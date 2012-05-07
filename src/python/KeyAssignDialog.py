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

class KeyAssignDialog(QtGui.QDialog):
    
	def browseClicked(self):
		dialog = QtGui.QFileDialog()
		
		dialog.setNameFilter('Python script files (*.py)')
		dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
		
		if dialog.exec_() > 0:
			self.fileName.setText(dialog.selectedFiles()[0])
    
	def assignPressedClicked(self):
		fileName = self.fileName.text()
		
		elementNoP = self.listPressedWidget.currentRow()
		elementNoR = self.listReleasedWidget.currentRow()
		
		if elementNoP >= 0:
			self.Portal.set('parm1', str(self.correspondences[elementNoP]))
			self.Portal.set('parm2', str(fileName))
			self.Portal.execCB('SetKeyPressedAssignment')
	
		self.refreshList()
		
		self.listPressedWidget.setCurrentRow(elementNoP)
		self.listReleasedWidget.setCurrentRow(elementNoR)
    
	def clearPressedClicked(self):
		
		elementNoP = self.listPressedWidget.currentRow()
		elementNoR = self.listReleasedWidget.currentRow()
		
		if elementNoP >= 0:
			self.Portal.set('parm1', str(self.correspondences[elementNoP]))
			self.Portal.set('parm2', '')
			self.Portal.execCB('SetKeyPressedAssignment')
	
		self.refreshList()
		
		self.listPressedWidget.setCurrentRow(elementNoP)
		self.listReleasedWidget.setCurrentRow(elementNoR)

	def assignReleasedClicked(self):
		fileName = self.fileName.text()
		
		elementNoP = self.listPressedWidget.currentRow()
		elementNoR = self.listReleasedWidget.currentRow()
		
		if elementNoR >= 0:
			self.Portal.set('parm1', str(self.correspondences[elementNoR]))
			self.Portal.set('parm2', str(fileName))
			self.Portal.execCB('SetKeyReleasedAssignment')
	
		self.refreshList()
		
		self.listPressedWidget.setCurrentRow(elementNoP)
		self.listReleasedWidget.setCurrentRow(elementNoR)
    
	def clearReleasedClicked(self):
		
		elementNoP = self.listPressedWidget.currentRow()
		elementNoR = self.listReleasedWidget.currentRow()
		
		if elementNoR >= 0:
			self.Portal.set('parm1', str(self.correspondences[elementNoR]))
			self.Portal.set('parm2', '')
			self.Portal.execCB('SetKeyReleasedAssignment')
	
		self.refreshList()
		
		self.listPressedWidget.setCurrentRow(elementNoP)
		self.listReleasedWidget.setCurrentRow(elementNoR)

	def __init__(self, parent, Portal):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()
		
	def refreshList(self):
		
		self.listPressedWidget.clear()
		self.listReleasedWidget.clear()
		self.correspondences = list()
		
		position = 0
		for n in range(0, 0xED):
			keyNum = str(n)
			self.Portal.set('parm1', keyNum)
			self.Portal.execCB('GetKeyPressedAssignment')
			keyName = self.Portal.get('parm1')
			scriptName = self.Portal.get('parm2')
			
			if keyName != '':
				self.correspondences.append(position)
				position += 1
				
				if scriptName != '':
					self.listPressedWidget.addItem(keyName + ' - [' + scriptName + ']')
				else:
					self.listPressedWidget.addItem(keyName + ' - [No assignment]')
					
			self.Portal.set('parm1', keyNum)
			self.Portal.execCB('GetKeyReleasedAssignment')
			keyName = self.Portal.get('parm1')
			scriptName = self.Portal.get('parm2')
			
			if keyName != '':
				self.correspondences.append(position)
				position += 1
				
				if scriptName != '':
					self.listReleasedWidget.addItem(keyName + ' - [' + scriptName + ']')
				else:
					self.listReleasedWidget.addItem(keyName + ' - [No assignment]')		
					

	def initUI(self):
        
		# Layout the widget 
		self.gridOut = QtGui.QGridLayout()
        
		self.frameTop = QtGui.QFrame()
		self.frameBot = QtGui.QFrame()
		
		self.gridTop = QtGui.QGridLayout()
		self.frameTop.setLayout(self.gridTop)
		
		self.gridBot = QtGui.QGridLayout()
		self.frameBot.setLayout(self.gridBot)
		
		self.gridOut.addWidget(self.frameTop, 0, 0)
		self.gridOut.addWidget(self.frameBot, 3, 0)

		# Add the refresh button

		button = QtGui.QPushButton('Browse')
		self.gridBot.addWidget(button, 1, 0)
		button.clicked.connect(self.browseClicked)

		button = QtGui.QPushButton('Assign Pressed')
		self.gridBot.addWidget(button, 1, 1)
		button.clicked.connect(self.assignPressedClicked)
		
		button = QtGui.QPushButton('Clear Pressed')
		self.gridBot.addWidget(button, 1, 2)
		button.clicked.connect(self.clearPressedClicked)
		
		button = QtGui.QPushButton('Assign Released')
		self.gridBot.addWidget(button, 1, 3)
		button.clicked.connect(self.assignReleasedClicked)
		
		button = QtGui.QPushButton('Clear Released')
		self.gridBot.addWidget(button, 1, 4)
		button.clicked.connect(self.clearReleasedClicked)		

		# Add the close button
		
		button = QtGui.QPushButton('Close')
		self.gridBot.addWidget(button, 1, 5)
		button.clicked.connect(self.close)

		self.setLayout(self.gridOut)   
			
		# Move the window to where the menu was
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)
		
		self.setWindowTitle('Key Assignments') 
	
		self.gridTop.addWidget(QtGui.QLabel('On key pressed scripts:'), 0, 0)
		self.gridTop.addWidget(QtGui.QLabel('On key released scripts:'), 0, 1)
		
		self.listPressedWidget = QtGui.QListWidget()
		self.gridTop.addWidget(self.listPressedWidget, 1, 0)
		
		self.listReleasedWidget = QtGui.QListWidget()
		self.gridTop.addWidget(self.listReleasedWidget, 1, 1)
		
		self.fileName = QtGui.QLineEdit('')
		self.gridOut.addWidget(QtGui.QLabel('Script file:'), 1, 0)
		self.gridOut.addWidget(self.fileName, 2, 0)
		
		self.refreshList()
		
		# Show the result
		
		self.show()
		
		
	def evListChanged(self, value):
		print('beer')
		#if self.childName == '':
		#	for x in range(len(self.parms)):
				# print(self.parms[x]);
				# print(self.eds[x].text())
		#		self.Portal.set(self.parms[x], str(self.eds[x].text()))
		#else:
		#	for x in range(len(self.parms)):
		#		self.Portal.setChild(self.childName, self.parms[x], str(self.eds[x].text()))
			
