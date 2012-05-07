from PyQt4 import QtGui
from PyQt4 import QtCore
import lxml.etree as xml

class PickAssignDialog(QtGui.QDialog):
    
	def __init__(self, parent, Portal, r):
		QtGui.QDialog.__init__(self, parent)
        
		self.Portal = Portal
		self.r = r
        
        # Cleanup this widget if it is closed
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        
		self.initUI()
		
	def populateTypes(self):
		self.typeKeys = self.r.typeToName.keys()
		
		# Populate gadget types list
		self.listGadgetTypes.clear()
		
		for val in self.typeKeys: 
			self.listGadgetTypes.addItem(self.r.typeToName[val])
	
	def populateScripts(self):
		row = self.listGadgetTypes.currentRow()
	
		self.listAssignedScripts.clear()
	
		if row != None:
			typeVal = self.typeKeys[row]
			
			try:
				scriptDict = self.r.pickTypeToScriptDict[typeVal]
			
				scriptNames = scriptDict.keys()
				for s in scriptNames:
					self.listAssignedScripts.addItem(s + ' [' + scriptDict[s] + ']')
			except:
				pass
	
	def typesListChanged(self, currentRow):
		self.populateScripts()
	
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
 
		# Add the label and list widgets for the gadget types
		self.gridTop.addWidget(QtGui.QLabel('Gadget type:'), 0, 0)
		self.listGadgetTypes = QtGui.QListWidget()
		self.gridTop.addWidget(self.listGadgetTypes, 1, 0)
		self.connect(self.listGadgetTypes, QtCore.SIGNAL("currentRowChanged(int)"),
				self.typesListChanged)
	
		# Add the label and list widgets for the scripts
		self.gridTop.addWidget(QtGui.QLabel('On pick, enable:'), 0, 1)
		self.listAssignedScripts = QtGui.QListWidget()
		self.gridTop.addWidget(self.listAssignedScripts, 1, 1)
 
		self.setLayout(self.gridOut)
 
		self.setWindowTitle('Gadget picking scripts') 
		
		# Move the window to where the menu was
		mouseX = int(self.Portal.get('MenuStartX'))
		mouseY = int(self.Portal.get('MenuStartY'))
		self.move(mouseX, mouseY)
		
		# Set up focus lost, kill window event (this doesn't work)
		# self.setFocusPolicy(QtCore.Qt.StrongFocus)

		self.populateTypes()

		# Show the result
		self.show()
		
		
	
	
		
