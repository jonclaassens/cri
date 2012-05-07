from PyQt4 import QtGui
from PyQt4 import QtCore
import functools

def launchPropertiesDialog(childName, childType, r):

	# If it is a quiver gadget of some sort, use the specialized settings dialog
	if childType == 15 or childType == 16 or childType == 17 or childType == 18:
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runQuiverSettings"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runQuiverSettings"), functools.partial(r.runQuiverSettings, childName))
		
		workThread.start()
		workThread.wait()

	# If it is a point cloud of some sort, use the specialized settings dialog
	elif childType == 10 or childType == 14:
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runPCSettings"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runPCSettings"), functools.partial(r.runPCSettings, childName))
		
		workThread.start()
		workThread.wait()
	
	# If it is a reticle use the specialized settings dialog
	elif childType == 4:
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runReticleSettings"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runReticleSettings"), functools.partial(r.runReticleSettings, childName))
		
		workThread.start()
		workThread.wait()
		
	# For everything else, use the general dialog
	else:
		class WorkThread(QtCore.QThread):
			def __init__(self):
				QtCore.QThread.__init__(self)
 
			def run(self):
				self.emit( QtCore.SIGNAL("runGeneralSettings"))

				return

		workThread = WorkThread()
		QtCore.QObject.connect(workThread, QtCore.SIGNAL("runGeneralSettings"), functools.partial(r.runGeneralSettings, childName))
		
		workThread.start()
		workThread.wait()
