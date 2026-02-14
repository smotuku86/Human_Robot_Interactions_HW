import pybullet as p
import numpy as np


# class for an external camera with a fixed base
# this camera does not move during the simulation
class ExternalCamera():

	def __init__(self, cameraDistance=1.0, cameraYaw=45, cameraPitch=-30, cameraRoll=0, cameraTargetPosition=[0,0,0], cameraWidth=256, cameraHeight=256):

		self.cameraDistance = cameraDistance
		self.cameraYaw = cameraYaw
		self.cameraPitch = cameraPitch
		self.cameraRoll = cameraRoll
		self.cameraTargetPosition = cameraTargetPosition
		self.cameraWidth = cameraWidth
		self.cameraHeight = cameraHeight

		self.proj_matrix = p.computeProjectionMatrixFOV(fov=60,
														aspect=self.cameraWidth / self.cameraHeight,
														nearVal=0.01,
														farVal=2.0)

		self.view_matrix = p.computeViewMatrixFromYawPitchRoll(distance=self.cameraDistance,
															yaw=self.cameraYaw,
															pitch=self.cameraPitch,
															roll=self.cameraRoll,
															cameraTargetPosition=self.cameraTargetPosition,
															upAxisIndex=2)

	# returns an rgb image as a numpy array
	def get_image(self):
		
		_, _, rgba, _, _ = p.getCameraImage(width=self.cameraWidth,
												height=self.cameraHeight,
												viewMatrix=self.view_matrix,
												projectionMatrix=self.proj_matrix,
												renderer=p.ER_BULLET_HARDWARE_OPENGL,
												flags=p.ER_NO_SEGMENTATION_MASK)

		rgba = np.array(rgba, dtype=np.uint8).reshape((self.cameraWidth, self.cameraHeight, 4))
		rgb = rgba[:, :, :3]
		return rgb
