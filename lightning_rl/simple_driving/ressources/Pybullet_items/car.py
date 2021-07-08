import pybullet as p
import math
import os
import numpy as np
import gym


class Car:
    def __init__(self, client, is_discrete=True, horizontal_res=320, vertical_res=240, fov=45, aspect=1, near_val=0.1,
                 far_val=100):
        self.client = client
        self.f_name = os.path.join(os.path.dirname(__file__), '../URDF/simplecar.urdf')

        # Car parameters
        self.car = None
        self.action_list = [[1, 0.6], [1, -0.6], [1, 0],
                            [0, 0]]  # 0: move left ; 1: move right ; 2: straight ; 3: stay still

        # Joint indices as found by p.getJointInfo()
        self.steering_joints = [0, 2]
        self.drive_joints = [1, 3, 4, 5]
        # Joint speed
        self.joint_speed = 0
        # Drag constants
        self.c_rolling = 0.2
        self.c_drag = 0.01
        # Throttle constant increases "speed" of the car
        self.c_throttle = 20

        # Camera resolution
        self.horizontal_res = horizontal_res
        self.vertical_res = vertical_res
        self.fov = fov
        self.aspect = aspect
        self.near_val = near_val
        self.far_val = far_val

        self.is_discrete = is_discrete
        if self.is_discrete:
            self.action_space = self.set_up_discrete_action_space()
        else:
            self.action_space = self.set_up_continuous_action_space()

    def get_ids(self):
        return self.car, self.client

    def load(self, position=(0, 0), base_orientation=(0, 0, 0, 0)):
        self.car = p.loadURDF(fileName=self.f_name,
                              basePosition=[position[0], position[1], 0.1],
                              baseOrientation=[base_orientation[0], base_orientation[1], base_orientation[2],
                                               base_orientation[3]],
                              physicsClientId=self.client)

    def set_up_continuous_action_space(self):
        """
        Set up continuous action space
        """
        return gym.spaces.box.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)

    def set_up_discrete_action_space(self):
        """
        Set up discrete action space
        """
        return gym.spaces.Discrete(len(self.action_list))

    def apply_action(self, action):

        if self.is_discrete:
            # Expects action to be two dimensional
            throttle, steering_angle = self.action_list[action]
        else:
            # Expects action to be two dimensional
            throttle, steering_angle = action

            # Clip throttle and steering angle to reasonable values
            throttle = min(max(throttle, 0), 1)
            steering_angle = max(min(steering_angle, 0.6), -0.6)

        # Set the steering joint positions
        p.setJointMotorControlArray(self.car, self.steering_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=[steering_angle] * 2,
                                    physicsClientId=self.client)

        # Calculate drag / mechanical resistance ourselves
        # Using velocity control, as torque control requires precise models
        friction = -self.joint_speed * (self.joint_speed * self.c_drag +
                                        self.c_rolling)
        acceleration = self.c_throttle * throttle + friction
        # Each time step is 1/240 of a second
        self.joint_speed = self.joint_speed + 1/30 * acceleration
        if self.joint_speed < 0:
            self.joint_speed = 0

        # Set the velocity of the wheel joints directly
        p.setJointMotorControlArray(
            bodyUniqueId=self.car,
            jointIndices=self.drive_joints,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[self.joint_speed] * 4,
            forces=[1.2] * 4,
            physicsClientId=self.client)

    def get_observation(self):
        # Get the position and orientation of the car in the simulation
        pos, ori = p.getBasePositionAndOrientation(self.car, self.client)
        # Get orientation of the car
        ang = p.getEulerFromQuaternion(ori)
        ang = (math.cos(ang[2]), math.sin(ang[2]))
        # Get the velocity of the car
        vel = p.getBaseVelocity(self.car, self.client)[0][0:2]

        # Get camera observations
        # Get projection matrix
        proj_matrix = p.computeProjectionMatrixFOV(fov=self.fov, aspect=self.aspect,
                                                   nearVal=self.near_val, farVal=self.far_val)
        # Get projection View_matrix
        # Get cam position
        cam_pos = np.array(pos)
        cam_pos[2] = 0.3
        cam_pos[0] = cam_pos[0] + 0.05
        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix(cam_pos, cam_pos + camera_vec, up_vec)

        # Display image
        frame = p.getCameraImage(self.horizontal_res, self.vertical_res, view_matrix, proj_matrix)
        true_depth = self.far_val * self.near_val/(self.far_val-(self.far_val-self.near_val)*frame[3])
        # Ignore Z pos of the car
        pos = pos[:2]

        # Concatenate position, orientation, velocity, rgba, depth, segmentation
        observation = (pos, ang, vel, frame[2], frame[3], frame[4], true_depth)

        # Reshape Proj and view matrix
        projection_matrix = np.asarray(proj_matrix).reshape([4, 4], order='F')
        view_matrix = np.asarray(view_matrix).reshape([4, 4], order='F')

        camera_parameters = (projection_matrix, view_matrix)

        return observation, camera_parameters
