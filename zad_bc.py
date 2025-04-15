import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Inicializácia simulácie
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# Vytvorenie trajektórie
center = np.array([0.6, 0, 0.5])
radius = 0.2
positions = []
num_positions = 400

for i in range(num_positions):
    amplitude = 0.05
    num_periods = 6
    angle = 2 * np.pi * i / num_positions
    x = center[0] + radius * np.cos(angle)
    y = center[1] + radius * np.sin(angle)
    z = center[2] + amplitude*np.sin(angle*num_periods)
    positions.append([x, y, z])

# Vykreslenie želanej trajektórie
for i in range(len(positions)-1):
    p.addUserDebugLine(positions[i], positions[i+1], [1, 0, 0], lineWidth=2.0)

prev_ee_pos = None
ee_index = 6
num_joints = p.getNumJoints(robot_id)
joint_indices = list(range(num_joints))
initial_target = positions[0]
ee_positions = []

# Pohyb koncového efektora k začiatočnému bodu
for _ in range(300):
    joint_angles = p.calculateInverseKinematics(robot_id, ee_index, initial_target)

    for i, j in enumerate(joint_indices):
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=joint_angles[i], force=500)
    p.stepSimulation()
    time.sleep(1./240.)

# Pohyb koncového efektora po trajektórii pomocou inverznej kinematiky
for pt in positions:
    joint_angles = p.calculateInverseKinematics(robot_id, ee_index, pt)
    for j in range(num_joints):
        p.setJointMotorControl2(bodyIndex=robot_id,
                                jointIndex=j,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_angles[j],
                                force=500)
    ee_pos = p.getLinkState(robot_id, ee_index)[4]
    # Znázornenie dosiahnutej trajektórie
    if prev_ee_pos is not None:
        p.addUserDebugLine(prev_ee_pos, ee_pos, lineColorRGB=[0,0,1], lineWidth=2, lifeTime=0)
    prev_ee_pos = ee_pos

    ee_positions.append(ee_pos)
    p.stepSimulation()
    time.sleep(1./240.)

positions = np.array(positions)
ee_positions = np.array(ee_positions)
input("Press Enter for graph")
p.disconnect()

# Vykreslenie trajektórií v matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Požadovaná trajektória', linestyle='--')
ax.plot(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2], label='Dosiahnutá trajektória', color='red')
ax.set_title('Trajektória koncového efektora robota')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()