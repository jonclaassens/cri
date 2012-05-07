from PyQt4 import QtGui
from PyQt4 import QtCore

class SettingsMenuDialog(QtGui.QDialog):
    
	def __init__(self, parent, Portal, r):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
		self.r = r
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()
		
	def pickScriptsClicked(self):
		
		# Create a worker thread to initialize the pick script settings dialog
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runPickAssign"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runPickAssign"), self.r.runPickAssign)
		workThread.start()
		
		workThread.wait()
		
		
	def keyboardScriptsClicked(self):
		
		# Create a worker thread to initialize the key-assigned script settings dialog
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runKeyAssign"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runKeyAssign"), self.r.runKeyAssign)
		workThread.start()
		
		workThread.wait()
		
		self.close()
	
	def refresh(self):
		state = int(self.Portal.get('SkyboxOn'))
		if state == 0:
			self.CBSkybox.setCheckState(QtCore.Qt.Unchecked)
		else:
			self.CBSkybox.setCheckState(QtCore.Qt.Checked)
			
		state = int(self.Portal.get('DrawFrameTree'))
		if state == 0:
			self.CBFrameTree.setCheckState(QtCore.Qt.Unchecked)
		else:
			self.CBFrameTree.setCheckState(QtCore.Qt.Checked)
		
	def initUI(self):
		
		# Layout the widget

		self.parentGrid = QtGui.QGridLayout()
        
		row = 0
        # Title: Settings
		label = QtGui.QLabel('General Settings')
		self.parentGrid.addWidget(label, row, 0)
		boldFont = label.font()
		boldFont.setBold(True)
		boldFont.setUnderline(True)
		label.setFont(boldFont);
        
        # Keyboard script settings button
		row = row + 1
		button = QtGui.QPushButton('Keyboard Scripts')
		self.parentGrid.addWidget(button, row, 0)
		button.clicked.connect(self.keyboardScriptsClicked)
		
		# Pick script settings button
		row = row + 1
		button = QtGui.QPushButton('Pick Scripts')
		self.parentGrid.addWidget(button, row, 0)
		button.clicked.connect(self.pickScriptsClicked)
        
        # Frame scale text box
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel('Marker Topic'), row, 0)
		self.LEMarker = QtGui.QLineEdit(str(self.Portal.get('MarkerTopic')))
		self.parentGrid.addWidget(self.LEMarker, row, 1)
		self.connect(self.LEMarker, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evLineEditChanged)
				
		# Frame scale text box
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel('Marker Array Topic'), row, 0)
		self.LEMarkerArray = QtGui.QLineEdit(str(self.Portal.get('MarkerArrayTopic')))
		self.parentGrid.addWidget(self.LEMarkerArray, row, 1)
		self.connect(self.LEMarkerArray, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evLineEditChanged)
				
			# Frame scale text box
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel('Marker Script Topic'), row, 0)
		self.LEMarkerScript = QtGui.QLineEdit(str(self.Portal.get('MarkerScriptTopic')))
		self.parentGrid.addWidget(self.LEMarkerScript, row, 1)
		self.connect(self.LEMarkerScript, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evLineEditChanged)
        
        # Skybox visible check box
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel('Skybox'), row, 0)
		self.CBSkybox = QtGui.QCheckBox('On')
		self.parentGrid.addWidget(self.CBSkybox, row, 1)
		self.connect(self.CBSkybox, QtCore.SIGNAL("stateChanged(int)"),
			self.evStateChanged1)
			
		 # Frame tree visible check box
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel('Draw Frame Tree'), row, 0)
		self.CBFrameTree = QtGui.QCheckBox('On')
		self.parentGrid.addWidget(self.CBFrameTree, row, 1)
		self.connect(self.CBFrameTree, QtCore.SIGNAL("stateChanged(int)"),
			self.evStateChanged2)
			
		# Frame scale text box
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel('Frame and Reticle scale (Arrow length is 1)'), row, 0)
		self.LEFrameScale = QtGui.QLineEdit(str(self.Portal.get('FrameScale')))
		self.parentGrid.addWidget(self.LEFrameScale, row, 1)
		self.connect(self.LEFrameScale, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evLineEditChanged)
		
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel(''), row, 0);
		
		# Title: Renderer statistics
		row = row + 1
		label = QtGui.QLabel('Renderer Statistics')
		self.parentGrid.addWidget(label, row, 0)
		label.setFont(boldFont);
		
		# Frame rate text box
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel('Average frame rate'), row, 0)
		self.LEFrameRate = QtGui.QLineEdit("-")
		self.parentGrid.addWidget(self.LEFrameRate, row, 1)
		self.LEFrameRate.setReadOnly(True)
		
		# Average triangle count text box
		row = row + 1
		self.parentGrid.addWidget(QtGui.QLabel('Moving average triangle count'), row, 0)
		self.LETriangleCount = QtGui.QLineEdit("-")
		self.parentGrid.addWidget(self.LETriangleCount, row, 1)
		self.LETriangleCount.setReadOnly(True)
		
		# Create a timer to update the bounds line edits
		self.refreshTimer = QtCore.QTimer()
		self.connect(self.refreshTimer, QtCore.SIGNAL("timeout()"), self.refreshTimeout)
		self.refreshTimer.setInterval(200)
		self.refreshTimer.setSingleShot(False)
		self.refreshTimer.start()
		self.haveTimer = 1
		
		# The window will use one layout
		self.setLayout(self.parentGrid)  
		self.setWindowTitle('Settings') 
		
		# Move the window to where the menu was
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)
		
		self.resize(640, 400)

		self.refresh()

		# Show the result
		self.show()
		
	def close(self):
		if self.haveTimer > 0:
			self.refreshTimer.stop()
			del self.refreshTimer
	
	def refreshTimeout(self):
		self.LEFrameRate.setText(format(float(self.Portal.get('AverageFPS')), '.2f'))
		self.LETriangleCount.setText(format(float(self.Portal.get('AverageTriangleCount')), '.2f'))
	
	def evStateChanged1(self, value):
		if self.CBSkybox.checkState() == QtCore.Qt.Checked:
			self.Portal.set('SkyboxOn', '1')
		else:
			self.Portal.set('SkyboxOn', '0')
		
	def evStateChanged2(self, value):
		if self.CBFrameTree.checkState() == QtCore.Qt.Checked:
			self.Portal.set('DrawFrameTree', '1')
		else:
			self.Portal.set('DrawFrameTree', '0')

	def evLineEditChanged(self, value):
		
		self.Portal.set('MarkerTopic', str(self.LEMarker.text()))
		self.Portal.set('MarkerArrayTopic', str(self.LEMarkerArray.text()))
		self.Portal.set('MarkerScriptTopic', str(self.LEMarkerScript.text()))
		
		try:
			scale = float(self.LEFrameScale.text())
			
			if scale <= 0:
				scale = 0.001
			
			self.Portal.set('FrameScale', str(scale))
		
		except:
			pass
			
		
		

