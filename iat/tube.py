import numpy as np
import pybullet as p
import time
import matplotlib.pyplot as plt
import pybullet_data

file = open("box.urdf", 'w')

mass = 1.0
radius=0.1
length=1.0
ix = 1/12.0*mass*(3*radius**2+length**2)
iy = 1/12.0*mass*(3*radius**2+length**2)
iz = 1/2.0*mass*radius**2

file.write("""
<?xml version="1.0"?>
<robot name="physics">
  <link name="back">
    <visual>
      <geometry>
        <cylinder radius="{radius}" length="{length}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0" />
      <material name="blue">
        <color rgba="1.0 1.0 1.0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="{radius}" length="{length}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0" />
    </collision>
    <inertial>
      <mass value="{mass}" />
      <inertia ixx="{ix}" ixy="0.0" ixz="0.0" iyy="{iy}" iyz="0.0" izz="{iz}"/>
      <origin rpy="0 0 0" xyz="0 0 0" />
    </inertial>
  </link>
</robot>
""".format(radius=radius, length=length, mass=mass,ix=ix,iy=iy,iz=iz))
file.close()

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,0)
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("box.urdf",cubeStartPos, cubeStartOrientation)
p.changeDynamics(boxId, -1, angularDamping=0.001)
p.applyExternalTorque(boxId, -1, [0.0, 100.0, 0.0], flags=p.WORLD_FRAME)
p.applyExternalForce(boxId, -1, [0.0, 100.0, 0.0], posObj=[0.0,0.0,1.0], flags=p.WORLD_FRAME)

rfps = 256
dfps = 60
tmax = 10
p.setTimeStep(1.0 / rfps)
data=[]
for i in range (rfps * tmax):
    start = time.time()
    pos, orientation = p.getBasePositionAndOrientation(boxId)
    # orientation = p.getEulerFromQuaternion(orientation)
    data.append(orientation)
    p.stepSimulation()

    time.sleep(max(1./dfps - (time.time() - start), 0))
p.disconnect()
plt.plot(data)
plt.show()
