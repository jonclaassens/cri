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

class ListSelectionsDialog(QtGui.QDialog):
	def __init__(self, parent, Portal, r):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
		self.r = r
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()
	
	def scriptClicked(self, fileName, targetName, reticleName):
		
		execfile(fileName)
	
		self.close()
	
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
			
		deleteItems(self.frameBot.layout())
		
		# Add the Keep button

		button = QtGui.QPushButton('Keep')
		self.gridBot.addWidget(button, 0, 0)
		
		# Add the 'Keep all' button
		
		button = QtGui.QPushButton('Keep All')
		self.gridBot.addWidget(button, 1, 0)
		#button.clicked.connect(self.refresh)
		
		# Add the close button
		
		button = QtGui.QPushButton('Close')
		self.gridBot.addWidget(button, 2, 0)
		button.clicked.connect(self.close)
		
		# Add all the scripts
		row = self.listWidget.currentRow()
		
		gridRow = 3
		if row != None:
			typeVal = int(self.Portal.getChild(self.selectables[row], 'TargetType'))
			targetName = self.Portal.getChild(self.selectables[row], 'TargetName')
			reticleName = self.selectables[row]
		
			try:
				scriptDict = self.r.pickTypeToScriptDict[typeVal]
				
				scriptNames = scriptDict.keys()
				for s in scriptNames:
					button = QtGui.QPushButton(s)
					self.gridBot.addWidget(button, gridRow, 0)
					button.clicked.connect(functools.partial(self.scriptClicked, 
						scriptDict[s], targetName, reticleName))
					
					gridRow = gridRow + 1
			except:
				pass
	
	def listChanged(self, currentRow):
		
		self.Portal.setChild(self.selectables[currentRow], 'Inflation', '10')
		
		self.refresh()
	
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

		self.setLayout(self.gridout)   
			
		# Move the window to where the menu was
		
		self.Portal.execCB('getMousePosition')
		mouseX = int(self.Portal.get('parm1'))
		mouseY = int(self.Portal.get('parm2'))
		self.move(mouseX, mouseY)
		
		self.setWindowTitle('Selected items:') 
		
		# Show the result
		self.listWidget = QtGui.QListWidget()
		self.gridTop.addWidget(self.listWidget, 0, 0);
		self.connect(self.listWidget, QtCore.SIGNAL("currentRowChanged(int)"),
				self.listChanged)
		
		children = self.Portal.listChildren()
		
		self.selectables = []
		
		self.number = 0
		for c in children:
			if self.Portal.getChild(c, 'type') == '3':
				self.number = self.number + 1
				self.listWidget.addItem(c)
				self.selectables.append(c)
		
		if self.number == 0:
			self.close()
			return
		
		# Populate the buttons
		self.refresh()
		
		self.show()
		
		
