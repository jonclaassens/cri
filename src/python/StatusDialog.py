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
import rospy

# Lazy importing
from PyQt4 import QtGui
from PyQt4 import QtCore

class StatusDialog(QtGui.QDialog):
    
	def __init__(self, parent, Portal, r):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
		self.r = r
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()

	def refreshList(self):
		self.r.statusMutex.acquire()
		
		if len(self.r.statusMessages) == 0:
			
			self.r.statusMutex.release()
			return
		
		gotoEnd = 0
		if self.listStatuses.count()-1 == self.listStatuses.currentRow():
			gotoEnd = 1
		
		endNo = self.r.statusMessages[-1].no
		curPos = len(self.r.statusMessages) - 1
		insertLocation = self.listStatuses.count()
		
		if not(endNo > (self.lastNo + 1)):
			self.r.statusMutex.release()
			return
		
		focusItem = None
		
		while endNo > (self.lastNo + 1):

			val = self.r.statusMessages[curPos]
					
			self.listItem = QtGui.QListWidgetItem(str(val.no) + " - [" + 
					str(val.time.to_sec()) + "] - " + val.source + " - " + val.message)
			brush = self.listItem.foreground()
		
			if val.statusNo == -2:
				brush.setColor(self.red)
			elif val.statusNo == -1:
				brush.setColor(self.yellow)
			elif val.statusNo == 0:
				brush.setColor(self.green)

			self.listItem.setForeground(brush)
			
			if focusItem == None:
				focusItem = self.listItem
				
			self.listStatuses.insertItem(insertLocation, self.listItem)
			
			endNo = val.no
			curPos = curPos - 1
			if curPos == 0:
				break
		
		self.r.statusMutex.release()
		
		if gotoEnd:
			self.listStatuses.scrollToItem(focusItem)
			self.listStatuses.setCurrentItem(focusItem)
		
		self.lastNo = self.r.statusMessages[-1].no
		
		# Trim so that the list isn't too big
		
		while self.listStatuses.count() > self.r.maxStatusMessages:
			temp = self.listStatuses.item(0)
			self.listStatuses.removeItemWidget(temp)	
		
	def populateList(self):
	
		self.r.statusMutex.acquire()
		
		self.source = ""
		self.message = ""
		self.statusNo = 0
		self.no = 0
		
		self.red = QtGui.QColor()
		self.red.setRgbF(1, 0, 0)
		
		self.yellow = QtGui.QColor()
		self.yellow.setRgbF(0.8, 0.5, 0)
		
		self.green = QtGui.QColor()
		self.green.setRgbF(0, 0.7, 0)
		
		for val in self.r.statusMessages: 
			self.listItem = QtGui.QListWidgetItem(str(val.no) + " - [" + 
				str(val.time.to_sec()) + "] - " + val.source + " - " + val.message)
			brush = self.listItem.foreground()
			
			if val.statusNo == -2:
				brush.setColor(self.red)
			elif val.statusNo == -1:
				brush.setColor(self.yellow)
			elif val.statusNo == 0:
				brush.setColor(self.green)

			self.listItem.setForeground(brush)
			self.listStatuses.addItem(self.listItem)
		
		self.r.statusMutex.release()
		
		if len(self.r.statusMessages) > 0:
		
			self.listStatuses.scrollToItem(self.listItem)
			self.listStatuses.setCurrentItem(self.listItem)
		
			self.lastNo = self.r.statusMessages[-1].no
			
		else:
			self.lastNo = 0

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
		self.gridOut.addWidget(self.frameBot, 1, 0)
 
		self.listStatuses = QtGui.QListWidget()
		self.gridTop.addWidget(self.listStatuses, 0, 0)
		# self.connect(self.listGadgetTypes, QtCore.SIGNAL("currentRowChanged(int)"),
		# 		self.typesListChanged)
	 
		self.setLayout(self.gridOut)
 
		self.setWindowTitle('System Status') 
		
		# Move the window to where the menu was
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)
		
		# Set up focus lost, kill window event (this doesn't work)
		# self.setFocusPolicy(QtCore.Qt.StrongFocus)

		self.populateList()

		# Create a timer to update the bounds line edits
		self.refreshTimer = QtCore.QTimer()
		self.connect(self.refreshTimer, QtCore.SIGNAL("timeout()"), self.refreshList)
		self.refreshTimer.setInterval(1000)
		self.refreshTimer.setSingleShot(False)
		self.refreshTimer.start()
		self.haveTimer = 1

		# Show the result
		self.resize(640, 480)
		self.show()
		
	def close(self):
	
		if self.haveTimer > 0:
			self.refreshTimer.stop()
			del self.refreshTimer
			
		print('run')
