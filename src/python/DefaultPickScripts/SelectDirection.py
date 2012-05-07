# Assuming we have self.portal, fileName, targetName, reticleName

self.Portal.setChild(reticleName, 'DirectionMode', '7')

# Promote the reticle to fix it in the gadget list
self.Portal.setChild(reticleName, 'type', '4')
