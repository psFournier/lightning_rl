import pybullet as p
import os


class Wall:
    def __init__(self, client, base_position, base_orientation):
        f_name = os.path.join(os.path.dirname(__file__), '../URDF/simplewall.urdf')
        p.loadURDF(fileName=f_name,
                   basePosition=[base_position[0], base_position[1], 0],
                   baseOrientation=[base_orientation[0], base_orientation[1], base_orientation[2], base_orientation[3]],
                   physicsClientId=client)