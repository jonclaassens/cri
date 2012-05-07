
# Create a worker thread for sending a signal to the main thread

class WorkThread(QtCore.QThread):
	def __init__(self):
		QtCore.QThread.__init__(self)
 
	def run(self):

		self.emit( QtCore.SIGNAL("runListSelections()"))
		
		return


workThread = WorkThread()
QtCore.QObject.connect(workThread, QtCore.SIGNAL("runListSelections()"), r.runListSelections)
workThread.start()
