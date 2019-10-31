import numpy as np
import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt

d = 10
g= 9.8

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
print(pybullet_data.getDataPath())
# p.setGravity(0,0,-g)
# planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robocat.urdf",cubeStartPos, cubeStartOrientation)

theta = np.pi / 2.0;
ix = 1/4*5.0*0.2**2+1/12*5.0*1.0
iy = 1/4*5.0*0.2**2+1/12*5.0*1.0
iz = 1/2*5.0*0.2**2
ixx = (iy + iz -ix) / 2.0
iyy = (ix + iz -iy) / 2.0
izz = (iy + ix -iz) / 2.0
cloud = 1 - (iyy*np.sin(theta/2.0))/(ixx+iyy+izz-(iyy*np.power(np.cos(theta/2.0),2.0)+izz*np.power(np.sin(theta/2.0),2.0)))
t = np.sqrt(2*d/g)
omega = np.pi/(t*cloud)

print("THETA: {:10f}".format(theta))
print("IX: {:10f} IY: {:10f} IZ: {:10f}".format(ix, iy, iz))
print("CLOUD: {:10f}".format(cloud))
print("T: {:10f}".format(t))
print("OMEGA: {:10f}".format(omega))

phi = 0.0
rfps = 240
dfps = 60
tmax = 10
p.setTimeStep(1.0 / rfps)
data = []
for i in range (rfps * tmax):
    start = time.time()
    p.stepSimulation()

    pos, orientation = p.getBasePositionAndOrientation(boxId)
    orientation = p.getEulerFromQuaternion(orientation)
    data.append(pos)
    p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[(np.pi - theta)*np.cos(phi), (np.pi - theta)*np.sin(phi)])
    time.sleep(max(1./dfps - (time.time() - start), 0))
    phi += (omega * 1.0 / rfps)
p.disconnect()
ox, oy, oz = zip(*data)
plt.plot(ox, label="$O_x$")
plt.plot(oy, label="$O_y$")
plt.plot(oz, label="$O_z$")
plt.legend()
plt.show()
