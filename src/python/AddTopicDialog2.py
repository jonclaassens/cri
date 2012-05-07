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
import rospy
import rosgraph.masterapi

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
	
def _master_get_topic_types(master):
	try:
		val = master.getTopicTypes()
	except xmlrpclib.Fault:
		#TODO: remove, this is for 1.1
		sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
		val = master.getPublishedTopics('/')
	return val

class AddTopicGUI(QtGui.QDialog):
    
	def __init__(self, parent, Portal):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
             
		# Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()

        
	def topic_type(self, t, topic_types):
		matches = [t_type for t_name, t_type in topic_types if t_name == t]
		if matches:
			return matches[0]
		return 'unknown type'  
        
	def add(self):
		selected = self.TWTopics.currentItem()
		if selected != None:
			# Is this a root (type) item with children?  If not, we may proceed to add the topic
			if selected.childCount() == 0:
				name = selected.text(0)
				topicString = self.topic_type(name, self.topic_types)
				
				self.Portal.set('parm1', str(name))
				self.Portal.set('parm2', topicString)
				self.Portal.execCB('AddTopic')
        
	def refresh(self):
			
		self.master = rosgraph.masterapi.Master('/rostopic')
		
		try:
			# Get the topics		
			state = self.master.getSystemState()
			self.pubs, self.subs, _ = state

		except socket.error:
			raise ROSTopicIOException("Unable to communicate with master!")

		# Clear the TreeWidget
		self.TWTopics.clear()

		self.topic_types = _master_get_topic_types(self.master)
		
		for topic, publishers in self.pubs:
			typeString = self.topic_type(topic, self.topic_types)
			
			# Is there a root element with a type name equal to the current topic type?
			root = self.TWTopics.findItems(typeString, QtCore.Qt.MatchExactly)
			
			# No?  Then add it
			if len(root) == 0:
				root = QtGui.QTreeWidgetItem([typeString])
				self.TWTopics.addTopLevelItem(root)
			# Other use the root to add the new topic details
			else:
				root = root[0]
		
			element = QtGui.QTreeWidgetItem([topic, str(len(publishers))])
			root.addChild(element)
			
		self.TWTopics.sortItems(0, QtCore.Qt.AscendingOrder)
		
		self.TWTopics.header().resizeSection(0, 320)
        
	def initUI(self):
		
		# Layout the widget
        
		self.gridout = QtGui.QGridLayout()
        
        # Create the window frames
		self.frameTop = QtGui.QFrame()
		self.frameBot = QtGui.QFrame()
		
		self.gridTop = QtGui.QGridLayout()
		self.frameTop.setLayout(self.gridTop)
		
		self.gridBot = QtGui.QGridLayout()
		self.frameBot.setLayout(self.gridBot)
		
		self.gridout.addWidget(self.frameTop, 0, 0)
		self.gridout.addWidget(self.frameBot, 1, 0)

		# Add the 'add' button
		button = QtGui.QPushButton('Add')
		self.gridBot.addWidget(button, 0, 0)
		button.clicked.connect(self.add)

		# Add the refresh button

		button = QtGui.QPushButton('Refresh')
		self.gridBot.addWidget(button, 0, 1)
		button.clicked.connect(self.refresh)
		
		# Add the close button
		
		button = QtGui.QPushButton('Close')
		self.gridBot.addWidget(button, 0, 2)
		button.clicked.connect(self.close)

		self.setLayout(self.gridout)   
		
		# Position window at the cursor
		
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)
		
		self.setWindowTitle('Select topic to add:') 

		# Add the topic tree view
		
		self.TWTopics = QtGui.QTreeWidget()
		self.TWTopics.setColumnCount(2)
		self.TWTopics.setHeaderLabels(['Name', 'No. Publishers'])
		self.gridTop.addWidget(self.TWTopics, 0, 0)

		# Update the data
		self.refresh()
		self.resize(640, 480)
		
		# Show the result
		 
		self.show()
