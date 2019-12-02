import numpy as np
import pybullet as p
import time
import matplotlib.pyplot as plt
import pybullet_data

file = open("box.urdf", 'w')

mh = 1.0
rh = 0.2
lh = 2.0

ixh = 1/12.0*mh*(3*rh**2+lh**2)
iyh = 1/12.0*mh*(3*rh**2+lh**2)
izh = 1/2.0*mh*rh**2

mb = 1.0
rb = 0.2
lb = 1.0

ixb = 1/12.0*mb*(3*rb**2+lb**2)
iyb = 1/12.0*mb*(3*rb**2+lb**2)
izb = 1/2.0*mb*rb**2

file.write("""
<?xml version="1.0"?>
<robot name="physics">
  <link name="handle">
    <visual>
      <geometry>
        <cylinder radius="{rh}" length="{lh}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0" />
      <material name="blue">
        <color rgba="1.0 1.0 1.0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="{rh}" length="{lh}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0" />
    </collision>
    <inertial>
      <mass value="{mh}" />
      <inertia ixx="{ixh}" ixy="0.0" ixz="0.0" iyy="{iyh}" iyz="0.0" izz="{izh}"/>
      <origin rpy="0 0 0" xyz="0 0 0" />
    </inertial>
  </link>
  <link name="bar">
    <visual>
      <geometry>
        <cylinder radius="{rb}" length="{lb}" />
      </geometry>
      <origin rpy="1.5707 0 0" xyz="0 {ty} 0" />
      <material name="blue">
        <color rgba="1.0 1.0 1.0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="{rb}" length="{lb}" />
      </geometry>
      <origin rpy="1.5707 0 0" xyz="0 {ty} 0" />
    </collision>
    <inertial>
      <mass value="{mb}" />
      <inertia ixx="{ixb}" ixy="0.0" ixz="0.0" iyy="{iyb}" iyz="0.0" izz="{izb}"/>
      <origin rpy="1.5707 0 0" xyz="0 {ty} 0" />
    </inertial>
  </link>
  <joint name="joint" type="fixed">
    <parent link="handle" />
    <child link="bar" />
    <origin xyz="0 0 0" />
  </joint>
</robot>
""".format(mh=mh, mb=mb, rh=rh,lh=lh,rb=rb,lb=lb, ty=(lb/2.0), ixh=ixh,iyh=iyh,izh=izh, ixb=ixb,iyb=iyb,izb=izb))
file.close()

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,0)
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("box.urdf",cubeStartPos, cubeStartOrientation)
p.applyExternalTorque(boxId, -1, [0.0, 2000.0, 1], flags=p.WORLD_FRAME)
p.changeDynamics(boxId, -1, angularDamping=0.0)
p.changeDynamics(boxId, 1, angularDamping=0.0)
p.changeDynamics(boxId, 2, angularDamping=0.0)

rfps = 256
dfps = 256
tmax = 1000
p.setTimeStep(1.0 / rfps)
data=[]
pixels = [500, 500]
for i in range (rfps * tmax):
    start = time.time()
    pos, orientation = p.getBasePositionAndOrientation(boxId)
    orientation = p.getEulerFromQuaternion(orientation)
    data.append(orientation)
    p.stepSimulation()

    if i % 1 == 0:
        frame = int(i / 10)
        viewMatrix = p.computeViewMatrix([2,2,2],[0,0,0], [0,0,1])
        projecttionMatrix = p.computeProjectionMatrixFOV(60, pixels[0] / pixels[1], 0.01, 100)
        img_arr = p.getCameraImage(pixels[0], pixels[1], viewMatrix, projecttionMatrix, shadow=1, lightDirection=[1,1,1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
        np_img_arr = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4))
        np_img_arr = np_img_arr * (1.0 / 255.0)
        plt.imsave("images/{}.png".format(frame), np_img_arr)
    time.sleep(max(1./dfps - (time.time() - start), 0))
p.disconnect()
plt.plot(data)
plt.show()
