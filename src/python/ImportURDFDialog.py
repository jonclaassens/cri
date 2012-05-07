from PyQt4 import QtGui
from PyQt4 import QtCore

class ImportURDFDialog(QtGui.QDialog):
    
	def __init__(self, parent, Portal, r):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
		self.r = r
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()
	
	def refresh(self):
		state = int(self.Portal.get('Skybox On'))
		if state == 0:
			self.CBSkybox.setCheckState(QtCore.Qt.Unchecked)
		else:
			self.CBSkybox.setCheckState(QtCore.Qt.Checked)
		
	def initUI(self):
		
		# Layout the widget

		self.parentGrid = QtGui.QGridLayout()
        
        # Title: Settings
		label = QtGui.QLabel('General Settings')
		self.parentGrid.addWidget(label, 0, 0)
		boldFont = label.font()
		boldFont.setBold(True)
		boldFont.setUnderline(True)
		label.setFont(boldFont);
        
        # Keyboard script settings button
		button = QtGui.QPushButton('Keyboard Scripts')
		self.parentGrid.addWidget(button, 1, 0)
		button.clicked.connect(self.keyboardScriptsClicked)
		
		# Pick script settings button
		button = QtGui.QPushButton('Pick Scripts')
		self.parentGrid.addWidget(button, 2, 0)
		button.clicked.connect(self.pickScriptsClicked)
        
        # Skybox visible check box
		self.parentGrid.addWidget(QtGui.QLabel('Skybox'), 3, 0)
		self.CBSkybox = QtGui.QCheckBox('On')
		self.parentGrid.addWidget(self.CBSkybox, 3, 1)
		self.connect(self.CBSkybox, QtCore.SIGNAL("stateChanged(int)"),
			self.evStateChanged)
			
		# Frame scale text box
		self.parentGrid.addWidget(QtGui.QLabel('Frame and Reticle scale (Arrow length is 1)'), 4, 0)
		self.LEFrameScale = QtGui.QLineEdit(str(self.Portal.get('Frame Scale')))
		self.parentGrid.addWidget(self.LEFrameScale, 4, 1)
		self.connect(self.LEFrameScale, QtCore.SIGNAL("textChanged(const QString&)"),
				self.evLineEditChanged)
		
		self.parentGrid.addWidget(QtGui.QLabel(''), 5, 0);
		
		# Title: Renderer statistics
		label = QtGui.QLabel('Renderer Statistics')
		self.parentGrid.addWidget(label, 6, 0)
		label.setFont(boldFont);
		
		# Frame rate text box
		self.parentGrid.addWidget(QtGui.QLabel('Average frame rate'), 7, 0)
		self.LEFrameRate = QtGui.QLineEdit("-")
		self.parentGrid.addWidget(self.LEFrameRate, 7, 1)
		self.LEFrameRate.setReadOnly(True)
		
		# Average triangle count text box
		self.parentGrid.addWidget(QtGui.QLabel('Moving average triangle count'), 8, 0)
		self.LETriangleCount = QtGui.QLineEdit("-")
		self.parentGrid.addWidget(self.LETriangleCount, 8, 1)
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
		
		# Mouse the window to the mouse cursor positon
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)

		self.refresh()

		# Show the result
		self.show()
		
	def close(self):
		if self.haveTimer > 0:
			self.refreshTimer.stop()
			del self.refreshTimer
	
	def refreshTimeout(self):
		self.LEFrameRate.setText(format(float(self.Portal.get('Average FPS')), '.2f'))
		self.LETriangleCount.setText(format(float(self.Portal.get('Average Triangle Count')), '.2f'))
	
	def evStateChanged(self, value):
		if self.CBSkybox.checkState() == QtCore.Qt.Checked:
			self.Portal.set('Skybox On', '1')
		else:
			self.Portal.set('Skybox On', '0')

	def evLineEditChanged(self, value):
		try:
			scale = float(self.LEFrameScale.text())
			
			if scale <= 0:
				scale = 0.001
			
			self.Portal.set('Frame Scale', str(scale))
			

		except:
			pass
