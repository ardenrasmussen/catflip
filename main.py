import numpy as np
import pybullet as p
import time
from types import SimpleNamespace
import pybullet_data
import matplotlib
import plotly.graph_objects as go
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import gen_cat

fa = 1e1
max_force = 1e1

argsDict = {
    'd': 10,
    'g': 9.8,
    'mass': 500.0,
    'length': 1.0,
    'radius': 0.2,
    'theta': np.pi * 0.8
}
args = SimpleNamespace(**argsDict)

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
T = (unfold_t - flip_t)
omega_0 = np.pi/(T*cloud)
omega = lambda t: omega_0*(-np.cos(np.pi/T*t)+1)

print("OMEGA_0: {}", omega_0)


phi = 0.0
tim = 0
rfps = 1000
dfps = 128
p.setTimeStep(1.0 / rfps)
p.changeDynamics(boxId, -1, angularDamping=0.0)

max_force = 4*omega_0**2*(1/3*args.mass*args.length**2)
fa = max_force

print("FORCE", max_force)

data = []
pixels = [500, 500]
i = 0
state = 0
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
    if tim < flip_t:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[np.pi - args.theta,0], forces=[fa, fa])
    elif tim < unfold_t:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[(np.pi - args.theta)*np.cos(phi), (np.pi - args.theta)*np.sin(phi)], forces=[max_force, max_force])
        phi += (omega(tim-flip_t) * 1.0 / rfps)
    else:
        p.setJointMotorControlArray(boxId, [0,2], p.POSITION_CONTROL, targetPositions=[0,0], forces=[fa, fa])

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
