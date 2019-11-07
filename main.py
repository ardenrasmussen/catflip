import numpy as np
import pybullet as p
import time
import pybullet_data
import matplotlib
import plotly.graph_objects as go

d = 20
g = 9.8

file = open("robocat.urdf", 'w')

mass = 1.0
fa = 1e1
max_force = 1e1
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
cubeStartPos = [0,0,d]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robocat.urdf",cubeStartPos, cubeStartOrientation)

# p.changeDynamics(boxId, 1, mass=1.0)
# p.changeDynamics(boxId, -1, mass=0.0)

# _, _, (ixx, iyy, izz), _, _, _ ,_, _, _, _, _ = p.getDynamicsInfo(boxId, 2)
ic = iyy*np.cos(theta/2.0)**2+izz*np.sin(theta/2.0)**2
cloud = 1 - 2 * izz / (2*ic) * np.sin(theta/2.0)
t = np.sqrt(2*d/g) # XXX
fold_t = 0
flip_t = 2 * t / 10
unfold_t = 8 * t / 10
T = (unfold_t - flip_t)
omega_0 = np.pi**2/(2*T*cloud)

print("OMEGA_0: {}", omega_0)


phi = 0.0
tim = 0
rfps = 1000
dfps = 128
tmax = np.sqrt(2*(d + 5)/g) * 2
p.setTimeStep(1.0 / rfps)
p.changeDynamics(boxId, -1, angularDamping=0.0)

g_slider = p.addUserDebugParameter('gravity', -100, 100, g)
d_slider = p.addUserDebugParameter('drop_height', 0, 50, d)
sim_fps = p.addUserDebugParameter('simulation_fps', 10, 10000, rfps)
dis_fps = p.addUserDebugParameter('display_fps', 10, 10000, dfps)

data = []
pixels = [500, 500]
i = 0
state = 0
while True:
    start = time.time()
    p.stepSimulation()

    if i % 10 == 0:
    #     if g != p.readUserDebugParameter(g_slider):
    #         g = p.readUserDebugParameter(g_slider)
    #         p.setGravity(0.0, 0.0, -g)
    #     if rfps != p.readUserDebugParameter(sim_fps):
    #         rfps = p.readUserDebugParameter(sim_fps)
    #         p.setTimeStep(1.0 / rfps)
    #     if dfps != p.readUserDebugParameter(dis_fps):
    #         dfps = p.readUserDebugParameter(dis_fps)
        keys = p.getKeyboardEvents()
    #     if ord('r') in keys and keys[ord('r')]&p.KEY_WAS_TRIGGERED:
    #         d = p.readUserDebugParameter(d_slider)
    #         p.resetBasePositionAndOrientation(boxId, [0,0,d + 5], p.getQuaternionFromEuler([0,0,0]))
    #         p.resetBaseVelocity(boxId, [0,0,0], [0,0,0])
    #         p.resetJointState(boxId, 0, 0)
    #         p.resetJointState(boxId, 1, 0)
    #         p.resetJointState(boxId, 2, 0)
    #         p.resetJointState(boxId, 3, 0)
    #         tim = 0
    #         phi = 0
    #         i = 0
    #         t = np.sqrt(2*d/g) # XXX
    #         fold_t = 0
    #         flip_t = 1 * t / 4
    #         unfold_t = 3 * t / 4
    #         omega = np.pi/((unfold_t - flip_t)*cloud)*-1.0
        if ord('q') in keys and keys[ord('q')]&p.KEY_WAS_TRIGGERED:
            break

    pos, ori= p.getBasePositionAndOrientation(boxId)
    ori= p.getEulerFromQuaternion(ori)
    data.append(ori)
    if tim < flip_t:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[np.pi - theta,0], forces=[fa, fa])
    elif tim < unfold_t:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[(np.pi - theta)*np.cos(phi), (np.pi - theta)*np.sin(phi)], forces=[max_force, max_force])
        phi += (omega_0*np.sin(np.pi/T*(tim-flip_t)) * 1.0 / rfps)
    else:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[0,0], forces=[fa, fa])

    # Image Rendering
    # viewMatrix = p.computeViewMatrix([5,5,5],[0,0,2], [0,0,1])
    # projecttionMatrix = p.computeProjectionMatrixFOV(60, pixels[0] / pixels[1], 0.01, 100)
    # img_arr = p.getCameraImage(pixels[0], pixels[1], viewMatrix, projecttionMatrix, shadow=1, lightDirection=[1,1,1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # np_img_arr = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4))
    # np_img_arr = np_img_arr * (1.0 / 255.0)
    # matplotlib.image.imsave("images/{}.png".format(i), np_img_arr)
    # print("{}/{}".format(i, int(rfps * tmax)))

    time.sleep(max(1./ dfps - (time.time() - start), 0))
    tim += 1.0 / rfps
    i+=1

p.disconnect()

# Orientation Plot
ox, oy, oz = zip(*data)
fig = go.Figure()
fig.add_trace(go.Scatter(y=ox, x=np.linspace(0, tim, len(ox)), mode='lines', name='$O_x$'))
fig.add_trace(go.Scatter(y=oy, x=np.linspace(0, tim, len(oy)), mode='lines', name='$O_y$'))
fig.add_trace(go.Scatter(y=oz, x=np.linspace(0, tim, len(oz)), mode='lines', name='$O_z$'))
fig.show()
