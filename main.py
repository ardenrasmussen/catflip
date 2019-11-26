import argparse
import numpy as np
import pybullet as p
import time
from types import SimpleNamespace
import pybullet_data
import matplotlib
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import gen_cat

fa = 1e1
max_force = 1e1

argsDict = {
    'd': 10, # GOOD
    'g': 9.8, # GOOD
    'mass': 500.0, # GOOD
    'length': 1.0, # GOOD
    'radius': 0.2, # GOOD
    'theta': np.pi * 0.8 # GOOD
}
parser = argparse.ArgumentParser()
parser.add_argument('-d', '--distance', dest='d', type=float, default=10.0)
parser.add_argument('-g', '--gravity', dest='g', type=float, default=9.8)
parser.add_argument('-m', '--mass', dest='mass', type=float, default=500.0)
parser.add_argument('-l','--length', dest='length', type=float, default=1.0)
parser.add_argument('-r','--radius', dest='radius', type=float,default=0.2)
parser.add_argument('-t','--theta', dest= 'theta', type=float,default=np.pi * 0.5)
args = parser.parse_args()

gen_cat.generate_urdf(args)

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-args.g)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,args.d]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robocat.urdf",cubeStartPos, cubeStartOrientation)


ixx = 1/12*args.mass*(args.radius**2+args.length**2)
iyy = 1/12*args.mass*(args.radius**2+args.length**2)
izz = 1/2*args.mass*args.radius**2
ic = iyy*np.cos(args.theta/2.0)**2+izz*np.sin(args.theta/2.0)**2 #XXX
cloud = 1 - 2 * izz / (2*ic) * np.sin(args.theta/2.0)
t = np.sqrt(2*args.d/args.g)
fold_t = 0
flip_t = 2 * t / 10
unfold_t = 8 * t / 10
dtfold = (flip_t-fold_t)
dtunfold = (t - unfold_t)
T = (unfold_t - flip_t)
omega_0 = np.pi/(T*cloud) * 1.1
omega = lambda t: omega_0*(-np.cos(np.pi/T*t)+1)

print("OMEGA_0: {}", omega_0)


phi = 0.0
tim = 0
rfps = 1000
dfps = 256
p.setTimeStep(1.0 / rfps)
p.changeDynamics(boxId, -1, angularDamping=0.0)

for i in range(0, 4, 2):
    res = p.getDynamicsInfo(boxId, i)
    I_dag = res[2]
    p.changeDynamics(boxId, i, localInertiaDiagonal=[ixx, iyy, izz])
p.changeDynamics(boxId, -1, angularDamping=0, linearDamping=0)
p.changeDynamics(boxId, 0, angularDamping=0, linearDamping=0)
p.changeDynamics(boxId, 1, angularDamping=0, linearDamping=0)
p.changeDynamics(boxId, 2, angularDamping=0, linearDamping=0)
p.changeDynamics(boxId, 3, angularDamping=0, linearDamping=0)
p.changeDynamics(boxId, 4, angularDamping=0, linearDamping=0)

max_force = 8*omega_0**2*(1/4*args.mass*args.radius**2 + 1/3*args.mass*args.length**2)
fa = 2.0 * (np.pi/dtfold)**2*((np.pi-args.theta)/2)*(1/4*args.mass*args.radius**2 + 1/3*args.mass*args.length**2)


data = []
momentum = []
pixels = [500, 500]
i = 0
state = 0
tx, ty = None, None
while True:
    start = time.time()
    p.stepSimulation()

    if i % 10 == 0:
        keys = p.getKeyboardEvents()
        if ord('q') in keys and keys[ord('q')]&p.KEY_WAS_TRIGGERED:
            break

    pos, ori= p.getBasePositionAndOrientation(boxId)
    ori= p.getEulerFromQuaternion(ori)
    data.append(ori)
    momentum.append(calc_momentum())
    if tim < flip_t:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[(np.pi - args.theta)/2*(1-np.cos(np.pi/dtfold*(tim-fold_t))),0], forces=[fa, fa])
    elif tim < unfold_t:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[(np.pi - args.theta)*np.cos(phi), (np.pi - args.theta)*np.sin(phi)], forces=[max_force, max_force])
        phi += (omega(tim-flip_t) * 1.0 / rfps)
    elif tim < t:
        if tx is None:
            j0 = p.getJointState(boxId, 0)
            j1 = p.getJointState(boxId, 2)
            tx = j0[0]
            ty = j1[0]
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[tx/2*(1+np.cos(np.pi/dtunfold*(tim-unfold_t))),ty/2*(1+np.cos(np.pi/dtunfold*(tim-unfold_t)))], forces=[fa, fa])

    # Image Rendering
    if i % 10 == 0:
        frame = i / 10
        viewMatrix = p.computeViewMatrix([5,5,5],[0,0,2], [0,0,1])
        projecttionMatrix = p.computeProjectionMatrixFOV(60, pixels[0] / pixels[1], 0.01, 100)
        img_arr = p.getCameraImage(pixels[0], pixels[1], viewMatrix, projecttionMatrix, shadow=1, lightDirection=[1,1,1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
        np_img_arr = np.reshape(img_arr[2], (img_arr[1], img_arr[0], 4))
        np_img_arr = np_img_arr * (1.0 / 255.0)
        plt.imsave("images/{}.png".format(frame), np_img_arr)

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
fig.add_trace(go.Scatter(y=momentum, x=np.linspace(0, tim, len(momentum)), mode='lines', name='$L$'))
fig.show()
