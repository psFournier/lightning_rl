import pybullet as p
import pybullet_data


class Plane:
    def __init__(self, client):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        f_name = "plane.urdf"
        p.loadURDF(fileName=f_name,
                   basePosition=[0, 0, 0],
                   physicsClientId=client)
