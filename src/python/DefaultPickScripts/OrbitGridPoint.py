# Assuming we have self.Portal, fileName, targetName, reticleName

# This script changes the camera mode to 'focus mode' and ensure the camera starts in the same
# place, but focused on the reticle position

x = self.Portal.getChild(reticleName, 'X')
y = self.Portal.getChild(reticleName, 'Y')
z = self.Portal.getChild(reticleName, 'Z')

self.Portal.set('CamXFocus', x)
self.Portal.set('CamYFocus', y)
self.Portal.set('CamZFocus', z)

x = float(x)
y = float(y)
z = float(z)

cx = float(self.Portal.get('CamXPos'))
cy = float(self.Portal.get('CamYPos'))
cz = float(self.Portal.get('CamZPos'))

dist = math.sqrt((cx - x)**2 + (cy - y)**2 + (cz - z)**2)
self.Portal.set('CamDistance', str(dist))
self.Portal.set('CamAzimuth', str(math.atan2((cy - y), (cx - x))))
self.Portal.set('CamZenith', str(math.asin((cz - z) / dist)))

self.Portal.set('CamMode', '1')

# Promote the reticle to fix it in the gadget list?  For now, no...
# self.Portal.setChild(reticleName, 'type', '4')
