import pybullet as p
import os


class Goal:
    def __init__(self, client, base):
        f_name = os.path.join(os.path.dirname(__file__), '../URDF/simplegoal.urdf')
        p.loadURDF(fileName=f_name,
                   basePosition=[base[0], base[1], 0],
                   physicsClientId=client)


class CylinderGoal:
    def __init__(self, client, base_position, base_orientation):
        f_name = os.path.join(os.path.dirname(__file__), '../URDF/cylinder_goal.urdf')
        p.loadURDF(fileName=f_name,
                   basePosition=[base_position[0], base_position[1], base_position[2]],
                   baseOrientation=[base_orientation[0], base_orientation[1], base_orientation[2],
                                    base_orientation[3]],
                   physicsClientId=client)