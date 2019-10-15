import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
print(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("cat.urdf",cubeStartPos, cubeStartOrientation)
front = p.createCollisionShape(p.GEOM_CYLINDER,radius=0.25,height=1.0)
back = p.createCollisionShape(p.GEOM_CYLINDER,radius=0.25,height=1.0)
for i in range (10000000):
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()
