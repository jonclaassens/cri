
# Create a worker thread for sending a signal to the main thread

class WorkThread(QtCore.QThread):
	def __init__(self):
		QtCore.QThread.__init__(self)
 
	def run(self):

		self.emit( QtCore.SIGNAL("runAddTopic()"))
		
		return


workThread = WorkThread()
QtCore.QObject.connect(workThread, QtCore.SIGNAL("runAddTopic()"), r.runAddTopic)
workThread.start()

