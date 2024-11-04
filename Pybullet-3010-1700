import pybullet
import pybullet_data
import os
import time

pybullet.connect(pybullet.GUI)
obj = pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, -1)
pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(), "Robot-0411-1400/urdf/Robot-0411-1400.urdf"))
pybullet.setGravity(0,0,-9.8)

while True:
  pybullet.stepSimulation()
  posAndOrn = pybullet.getBasePositionAndOrientation(obj)
  print(posAndOrn)
  time.sleep(1/240)
