import pybullet as p
import os


class EnclosureWall:
    def __init__(self, client, base_position, base_orientation):
        f_name = os.path.join(os.path.dirname(__file__), '../URDF/enclosurewall.urdf')
        p.loadURDF(fileName=f_name,
                   basePosition=[base_position[0], base_position[1], base_position[2]],
                   baseOrientation=[base_orientation[0], base_orientation[1], base_orientation[2], base_orientation[3]],
                   physicsClientId=client)
