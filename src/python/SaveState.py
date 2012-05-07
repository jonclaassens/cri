import lxml.etree as xml

def saveState(Portal, fileName):

	portalState = xml.Element('portalState')
		
	# Save portal state
	systemState = xml.Element('systemState')
	props = Portal.listProperties()
		
	# Iterate through the system properties and save their values
	for cp in props:
		systemState.attrib[cp] = Portal.get(cp)
	portalState.append(systemState)
	
	# Save gadgets
	
	gadgetState = xml.Element('gadgetState')
	
	# Get children list
	children = Portal.listChildren()
	elementNo = 0
	
	# For each child
	for c in children:
		# Create XML element and assign all it's attributes to match the child properties
		newElement = xml.Element('ELEMENT' + str(elementNo))
		newElement.attrib['name'] = c
		
		childProps = Portal.listChildProperties(c)
		
		for cp in childProps:
			newElement.attrib[cp] = Portal.getChild(c, cp)
			
		gadgetState.append(newElement)
		
		elementNo = elementNo + 1
	
	portalState.append(gadgetState)
	
	# Save the key assignments
	
	keyboardState = xml.Element('keyboardState')

	for n in range(0, 0xED):
		keyNum = str(n)
		Portal.set('parm1', keyNum)
		Portal.execCB('GetKeyPressedAssignment')
		keyName = Portal.get('parm1')
		scriptName = Portal.get('parm2')
		
		keyNum = 'KEY' + keyNum
		newElement = xml.Element(keyNum)
		newElement.attrib['code'] = str(n)
		newElement.attrib['keyName'] = keyName
		newElement.attrib['scriptName'] = scriptName
		keyboardState.append(newElement)
	
	portalState.append(keyboardState)
	
	
	# Write to file
	try:
		xml.ElementTree(portalState).write(fileName, pretty_print=True)
	
	except:

		return False
	
	return True
	
