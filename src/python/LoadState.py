import lxml.etree as xml

def loadState(Portal, filename):

	# Clear the system of gadgets
	
	# Get children list
	children = Portal.listChildren()
	
	# For each child
	for c in children:
		Portal.set('parm1', c)
		Portal.execCB('DeleteGadget')

	# Parse the xml file

	tree = xml.ElementTree()
	
	try:
		tree.parse(filename)
	except:

		return False
	
	# Load Portal's state
	portalState = tree.find('systemState')
	for k in portalState.keys():
			if k != 'name':
				Portal.set(k, portalState.attrib[k])
	
	# Load the gadgets' states
	gadgetState = tree.find('gadgetState')
	
	if gadgetState == None:
		return False
		
	gadgets = list(gadgetState.iter())
	
	for g in gadgets:
		
		typeNo = g.get('type')
		name = g.get('name')
		
		# If the element has no type number it is not valid
		if typeNo == None:
			continue
		
		if int(typeNo) >= 10:
			# It's a topic, add a subscriber
			
			Portal.set('parm1', name)
			if typeNo == '10':	
				Portal.set('parm2', 'sensor_msgs/PointCloud2')
			
			elif typeNo == '11':
				Portal.set('parm2', 'nav_msgs/Path')
				
			elif typeNo == '12':
				Portal.set('parm2', 'nav_msgs/GridCells')
				
			elif typeNo == '13':
				Portal.set('parm2', 'nav_msgs/OccupancyGrid')
				
			elif typeNo == '14':	
				Portal.set('parm2', 'sensor_msgs/LaserScan')	
			
			Portal.execCB('AddTopic')
		elif int(typeNo) == 4:
			
			Portal.set('parm1', name)
			Portal.execCB('AddReticle')
			
		elif int(typeNo) == 1:
			
			Portal.set('parm1', name)
			Portal.set('parm2', "")
			Portal.execCB('AddGrid')
		else:
			continue
		
		# For topics and all gadgets load the configuration from the file
		for k in g.keys():
			if k != 'name':
				Portal.setChild(name, k, g.attrib[k])
			
	# Load key assignments
	
	keyboardState = tree.find('keyboardState')
	keys = list(keyboardState.iter())
	
	for k in keys:
		code = k.get('code')
		
		if code == None:
			continue
		
		Portal.set('parm1', code)
		Portal.set('parm2', k.attrib['scriptName'])
		Portal.execCB('SetKeyPressedAssignment')
		
	return True
