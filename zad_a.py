import pybullet as p
import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from etils import epath
from math import ceil

# Vykreslenie pracovného priestoru robota
def draw_workspace():
    max_points = 131072 # maximálny počet bodov v PyBullet
    num_samples = 7 
    num_joints = p.getNumJoints(robot_id)
    joint_ranges = []
    ee_index = 6
    # Výpočet hraníc jednotlivých členov
    for i in range(num_joints):
        lower_limit = p.getJointInfo(robot_id, i)[8]
        upper_limit = p.getJointInfo(robot_id, i)[9]
        joint_ranges.append(np.linspace(lower_limit, upper_limit, num_samples))
    
    n = ceil(num_samples**num_joints/max_points);
    counter = 0;
    points = []
    colors = []
    # Vygenerovanie všetkých kombinácií kĺbových uhlov
    for a in range(num_samples):
        for b in range(num_samples):
            for c in range(num_samples):
                for d in range(num_samples):
                    for e in range(num_samples):
                        for f in range(num_samples):
                            for g in range(num_samples):
                                # Vytvorenie konfigurácie kĺbových uhlov pre všetky členov
                                joint_positions = [
                                    joint_ranges[0][a],
                                    joint_ranges[1][b],
                                    joint_ranges[2][c],
                                    joint_ranges[3][d],
                                    joint_ranges[4][e],
                                    joint_ranges[5][f],
                                    joint_ranges[6][g]
                                ]
                                
                                # Výpočet pozície koncového efektora
                                for joint_idx in range(num_joints):
                                    p.resetJointState(robot_id, joint_idx, joint_positions[joint_idx])
                                position = p.getLinkState(robot_id, ee_index)[4]
                                # Uloženie všetkých n bodov
                                if counter % n == 0:
                                    points.append(position)
                                    colors.append([0, 0, 1])
                                counter += 1
    
    p.addUserDebugPoints(points, colors, pointSize=2)

if __name__ == "__main__":
    # Inicializácia simulácie
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
    
    p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])
    
    # Vykreslenie pracovného priestoru
    draw_workspace()
    
    while True:
        p.stepSimulation()
        time.sleep(1./240.) 