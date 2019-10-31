import numpy as np
import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import plotly.graph_objects as go

d = 50
g = 9.8

file = open("robocat.urdf", 'w')

mass = 50.0
length= 1.0
radius= 0.2
theta = np.pi / 2.0;

ixx = 1/12*mass*(radius**2+length**2)
iyy = 1/12*mass*(radius**2+length**2)
izz = 1/2*mass*radius**2

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

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-g)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,d+5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robocat.urdf",cubeStartPos, cubeStartOrientation)

# p.changeDynamics(boxId, 1, mass=1.0)
# p.changeDynamics(boxId, -1, mass=0.0)

ic = iyy*np.cos(theta/2.0)**2+izz*np.sin(theta/2.0)**2
cloud = 1 - 2 * izz / (2*ic) * np.sin(theta/2.0)
t = np.sqrt(2*d/g) # XXX
omega = np.pi/(t*cloud)
print("IXX: {:10f} IYY: {:10f} IZZ: {:10f}".format(ixx, iyy, izz))
print("IC: {:10f}".format(ic))
print("CLOUD: {:10f}".format(cloud))
print("T: {:10f}".format(t))
print("OMEGA: {:10f}".format(omega))
_, _, (ixx, iyy, izz), _, _, _ ,_, _, _, _, _ = p.getDynamicsInfo(boxId, 2)
ic = iyy*np.cos(theta/2.0)**2+izz*np.sin(theta/2.0)**2
cloud = 1 - 2 * izz / (2*ic) * np.sin(theta/2.0)
t = np.sqrt(2*d/g) # XXX
omega = np.pi/(t*cloud)
print("IXX: {:10f} IYY: {:10f} IZZ: {:10f}".format(ixx, iyy, izz))
print("IC: {:10f}".format(ic))
print("CLOUD: {:10f}".format(cloud))
print("T: {:10f}".format(t))
print("OMEGA: {:10f}".format(omega))

# ic = np.sqrt(iyy**2*np.cos(theta/2.0)**2+izz**2*np.sin(theta/2.0)**2)
cloud = 1 - 2 * izz / (2*ic) * np.sin(theta/2.0)
t = np.sqrt(2*d/g) # XXX
omega = np.pi/(t*cloud)*1.2


phi = 0.0
rfps = 256
dfps = 128
tmax = np.sqrt(2*(d + 5)/g) * 2
p.setTimeStep(1.0 / rfps)
# p.applyExternalTorque(boxId, -1, [100000.0, 0.0, 0.0], flags=p.WORLD_FRAME)
p.changeDynamics(boxId, -1, angularDamping=0.0)
data = []
for i in range (int(rfps * tmax)):
    start = time.time()
    p.stepSimulation()

    pos, ori= p.getBasePositionAndOrientation(boxId)
    ori= p.getEulerFromQuaternion(ori)
    data.append(ori)
    tim = i/float(rfps)
    if (tim < t):
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[(np.pi - theta)*np.cos(phi), (np.pi - theta)*np.sin(phi)], forces=[100,100])
        phi += (omega * 1.0 / rfps)
    else:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[0,0], forces=[100,100])
        phi += (omega * 1.0 / rfps)
    time.sleep(max(1./dfps - (time.time() - start), 0))

p.disconnect()
ox, oy, oz = zip(*data)
fig = go.Figure()
fig.add_trace(go.Scatter(y=ox, x=np.linspace(0, tmax, len(ox)), mode='lines', name='$O_x$'))
fig.add_trace(go.Scatter(y=oy, x=np.linspace(0, tmax, len(oy)), mode='lines', name='$O_y$'))
fig.add_trace(go.Scatter(y=oz, x=np.linspace(0, tmax, len(oz)), mode='lines', name='$O_z$'))
fig.show()
# plt.plot(ox, label="$O_x$")
# plt.plot(oy, label="$O_y$")
# plt.plot(oz, label="$O_z$")
# plt.legend()
# plt.show()
