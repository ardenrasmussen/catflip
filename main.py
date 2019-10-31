import numpy as np
import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt

d = 50
g= 9.8

file = open("robocat.urdf", 'w')

mass = 5.0
length= 1.0
radius= 0.2
theta = np.pi / 2.0;

ixx = 1/4*5.0*0.2**2+1/12*5.0*1.0
iyy = 1/4*5.0*0.2**2+1/12*5.0*1.0
izz = 1/2*5.0*0.2**2

frx, fry, frz = np.pi / 2.0, 0.0, 0.0
ftx, fty, ftz = 0.0, length/2.0 + 0, 0.0
brx, bry, brz = np.pi / 2.0, 0.0, 0.0
btx, bty, btz = 0.0, -length/2.0 - 0, 0.0

file.write("""
<?xml version="1.0"?>
<robot name="robocat">
    <link name="frontCylinder">
    <visual>
        <geometry>
            <cylinder length="{length}" radius="{radius}" />
        </geometry>
        <origin rpy="{frx} {fry} {frz}" xyz="{ftx} {fty} {ftz}" />
        <material name="gen-mat">
            <color rgba="1.0 1.0 1.0 1.0" />
        </material>
    </visual>
    <collision>
        <geometry>
            <cylinder length="{length}" radius="{radius}" />
        </geometry>
        <origin rpy="{frx} {fry} {frz}" xyz="{ftx} {fty} {ftz}" />
    </collision>
    <inertial>
        <mass value="{mass}" />
        <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />
        <origin rpy="{frx} {fry} {frz}" xyz="{ftx} {fty} {ftz}" />
    </inertial>
    </link>
    <link name="frontCylinder-leg">
    <visual>
        <geometry>
            <cylinder length="0.5" radius="0.1" />
        </geometry>
        <material name="gen-mat">
            <color rgba="1.0 1.0 1.0 1.0" />
        </material>
        <origin rpy="0.0 0.0 0.0" xyz="0.0 {fty} {radius}" />
    </visual>
    </link>
    <joint name="frontCylinder-leg-to-frontCylinder" type="fixed">
        <parent link="frontCylinder" />
        <child link="frontCylinder-leg" />
        <origin xyz="0.0 {fty} {radius}" />
    </joint>
    <link name="backCylinder">
    <visual>
        <geometry>
            <cylinder length="{length}" radius="{radius}" />
        </geometry>
        <origin rpy="{brx} {bry} {brz}" xyz="{btx} {bty} {btz}" />
        <material name="gen-mat">
            <color rgba="1.0 1.0 1.0 1.0" />
        </material>
    </visual>
    <collision>
        <geometry>
            <cylinder length="{length}" radius="{radius}" />
        </geometry>
        <origin rpy="{brx} {bry} {brz}" xyz="{btx} {bty} {btz}" />
    </collision>
    <inertial>
        <mass value="{mass}" />
        <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />
        <origin rpy="{brx} {bry} {brz}" xyz="{btx} {bty} {btz}" />
    </inertial>
    </link>
    <link name="backCylinder-leg">
    <visual>
        <geometry>
            <cylinder length="0.5" radius="0.1" />
        </geometry>
        <material name="gen-mat">
            <color rgba="1.0 1.0 1.0 1.0" />
        </material>
        <origin rpy="0.0 0.0 0.0" xyz="0.0 {bty} {radius}" />
    </visual>
    </link>
    <joint name="backCylinder-leg-to-backCylinder" type="fixed">
        <parent link="backCylinder" />
        <child link="backCylinder-leg" />
        <origin xyz="0.0 {bty} {radius}" />
    </joint>
    <link name="centerSphere">
    <visual>
        <geometry>
            <sphere radius="{radius}" />
        </geometry>
        <material name="gen-mat">
            <color rgba="1.0 1.0 1.0 1.0" />
        </material>
        <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0" />
    </visual>
    </link>
    <joint name="front_to_center" type="revolute">
        <axis xyz="1 0 0" />
        <limit effort="0.0" lower="-1.507" upper="1.507" velocity="0.5" />
        <parent link="centerSphere" />
        <child link="frontCylinder" />
        <origin xyz="0 0 0" />
    </joint>
    <joint name="back_to_center" type="revolute">
        <axis xyz="0 0 1" />
        <limit effort="0.0" lower="-1.507" upper="1.507" velocity="0.5" />
        <parent link="centerSphere" />
        <child link="backCylinder" />
        <origin xyz="0 0 0" />
    </joint>
</robot>
""".format(mass=mass,length=length,radius=radius,theta=theta,ixx=ixx,iyy=iyy,izz=izz,frx=frx,fry=fry,frz=frz,ftx=ftx,fty=fty,ftz=ftz,brx=brx,bry=bry,brz=brz,btx=btx,bty=bty,btz=btz))
file.close()

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
print(pybullet_data.getDataPath())
p.setGravity(0,0,-g)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,d]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robocat.urdf",cubeStartPos, cubeStartOrientation)

ic = iyy*np.cos(theta/2.0)**2+izz*np.sin(theta/2.0)**2
cloud = 1 - 2 * izz / (2*ic) * np.sin(theta/2.0)
t = np.sqrt(2*(d-2)/g) # XXX
omega = np.pi/(t*cloud)

print("THETA: {:10f}".format(theta))
print("IXX: {:10f} IYY: {:10f} IZZ: {:10f}".format(ixx, iyy, izz))
print("CLOUD: {:10f}".format(cloud))
print("T: {:10f}".format(t))
print("OMEGA: {:10f}".format(omega))

phi = 0.0
rfps = 240
dfps = 30
tmax = int(t + 2)
p.setTimeStep(1.0 / rfps)
data = []
for i in range (rfps * tmax):
    start = time.time()
    p.stepSimulation()

    pos, orientation = p.getBasePositionAndOrientation(boxId)
    orientation = p.getEulerFromQuaternion(orientation)
    data.append(pos)
    tim = i/rfps
    if (tim < t):
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[(np.pi - theta)*np.cos(phi), (np.pi - theta)*np.sin(phi)])
        phi += (omega * 1.0 / rfps)
    else:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[0,0])
        phi += (omega * 1.0 / rfps)
    time.sleep(max(1./dfps - (time.time() - start), 0))

p.disconnect()
ox, oy, oz = zip(*data)
plt.plot(ox, label="$O_x$")
plt.plot(oy, label="$O_y$")
plt.plot(oz, label="$O_z$")
plt.legend()
plt.show()
