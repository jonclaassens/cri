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


# Our exceptions
class ROSTopicException(Exception):
    """
    Base exception class of rostopic-related errors
    """
    pass

class ROSTopicIOException(ROSTopicException):
    """
    rostopic errors related to network I/O failures
    """
    pass

class TFFramesDialog(QtGui.QDialog):
    
	def __init__(self, parent, Portal, tf):
		QtGui.QDialog.__init__(self, parent)
        
        # We need Portal C++ core and TF
		self.tf = tf
		self.Portal = Portal
		
		# Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
		
		self.initUI()
       
	def attachCamera(self):
		item = self.tree.currentItem()
		if item != None:
			self.Portal.set('TrackFrameName', str(item.text(0)))
			# self.txtFixedFrame.setText(self.Portal.get('fixedFrame'))
			self.Portal.set('CamMode', '2');
       
	def setFixed(self):
		item = self.tree.currentItem()
		if item != None:
			self.Portal.set('FixedFrame', str(item.text(0)))
			self.txtFixedFrame.setText(self.Portal.get('FixedFrame'))
			
	def addGrid(self):	
		item = self.tree.currentItem()
		if item != None:
			self.Portal.set('parm1', "");
			self.Portal.set('parm2', str(item.text(0)))
			self.Portal.execCB('AddGrid')	
        
	def refresh(self):
		
		
		try:
			# Get the frames
			self.frames = self.tf.getFrameStrings()
			
	#  The right error	
		except socket.error:
			raise ROSTopicIOException("Unable to communicate with master!")

		# Clear the current frame tree
		self.tree.clear()

		# Temporary frames list
		frames = self.frames
		
		# The following loop operates as follows.  It continues until the frames list is empty.
		# A for loop runs through the list and checks the frames for a parent.  If there is no 
		# parent they are added to the tree widget root and then removed from the temporary list.  
		# If there is a parent and it is already in the list, the frame is added to the parent's
		# tree item, otherwise the frame is left in the list.  Eventually a representative tree
		# will emerge.
		while len(frames) > 0:
			for f in frames:
				# Ask Portal C++ core to find the frames parent (python TF can't do this)
				self.Portal.set('parm1', f)
				self.Portal.execCB('GetFrameParent')
				parent = self.Portal.get('parm1')
				
				if parent=="":
					# If there is no parent frame, add it to the tree as a root
					element = QtGui.QTreeWidgetItem([f])
					self.tree.addTopLevelItem(element)
					frames.remove(f)
				else:
					# If there is a parent frame, find its name in the tree and add the element to it.
					# Remove the frame from the list.
					matches = self.tree.findItems(parent, QtCore.Qt.MatchContains | QtCore.Qt.MatchRecursive) #QtCore.Qt.MatchFlags())
					if len(matches) > 0:
						element = QtGui.QTreeWidgetItem([f])
						matches[0].addChild(element)
						frames.remove(f)	
			
		self.frameTop.setFrameShape(QtGui.QFrame.StyledPanel)
		
		# Set fixed frame text box to the appropriate string
		self.txtFixedFrame.setText(self.Portal.get('FixedFrame'))
        
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

		self.gridBot.addWidget(QtGui.QLabel('Fixed Frame:'), 0, 0)
		
		self.txtFixedFrame = QtGui.QLineEdit('')
		self.gridBot.addWidget(self.txtFixedFrame, 0, 1)
	
		# Add the refresh button
		button = QtGui.QPushButton('Refresh')
		self.gridBot.addWidget(button, 1, 0)
		self.gridBot.addWidget(QtGui.QLabel(''), 0, 1)
		button.clicked.connect(self.refresh)
		
		# Add the root frame button
		
		button = QtGui.QPushButton('Set Fixed Frame')
		self.gridBot.addWidget(button, 1, 1)
		button.clicked.connect(self.setFixed)
		
		# Add the camera frame button
		
		button = QtGui.QPushButton('Set Camera Frame')
		self.gridBot.addWidget(button, 1, 2)
		button.clicked.connect(self.attachCamera)
		
		# Add the add grid button
		
		button = QtGui.QPushButton('Add Grid')
		self.gridBot.addWidget(button, 1, 3)
		button.clicked.connect(self.addGrid)
		
		# Add the close button
		
		button = QtGui.QPushButton('Close')
		self.gridBot.addWidget(button, 1, 4)
		button.clicked.connect(self.close)

		# Create the tree widget
		self.tree = QtGui.QTreeWidget()
		self.gridTop.addWidget(self.tree)
		self.tree.setHeaderLabel('Frames')
		
		self.setWindowTitle('Available frames:') 

		# Move the window to where the menu was
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)

		# Populate the tree widget
		self.refresh()
		
		# Show the result
		self.show()
	
