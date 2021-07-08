import pybullet as p
# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI)
p.setGravity(0, 0, -10, physicsClientId=client)

import pybullet_data
p.setAdditionalSearchPath(pybullet_data.getDataPath())
carId = p.loadURDF("racecar/racecar.urdf", basePosition=[0,0,0.2])
planeId = p.loadURDF("plane.urdf")

for joint_number in range(p.getNumJoints(carId)):
    info = p.getJointInfo(carId, joint_number)
    print(info[0], ": ", info[1])

angle = p.addUserDebugParameter('Steering', -0.5, 0.5, 0)
throttle = p.addUserDebugParameter('Throttle', 0, 20, 0)

wheel_indices = [2, 3, 5, 7]
hinge_indices = [4, 6]

while True:
    user_angle = p.readUserDebugParameter(angle)
    user_throttle = p.readUserDebugParameter(throttle)
    for joint_index in wheel_indices:
        p.setJointMotorControl2(carId, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttle)
    for joint_index in hinge_indices:
        p.setJointMotorControl2(carId, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_angle)
    p.stepSimulation()