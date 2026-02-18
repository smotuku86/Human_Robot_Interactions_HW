import pybullet as p
import pybullet_data
import os


# see list of pybullet objects here:
# https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data
class PyBulletObject():

    def __init__(self, object_name, basePosition=[0.0, 0.0, 0.0], baseOrientation=[0.0, 0.0, 0.0, 1.0], globalScaling=1.0, useFixedBase=False):
        urdfRootPath = pybullet_data.getDataPath()
        self.object = p.loadURDF(os.path.join(urdfRootPath, object_name), basePosition=basePosition, baseOrientation=baseOrientation, globalScaling=globalScaling, useFixedBase=useFixedBase)

    def get_state(self):
        values = p.getBasePositionAndOrientation(self.object)
        state = {}
        state["position"] = values[0]
        state["quaternion"] = values[1]
        state["euler"] = p.getEulerFromQuaternion(state["quaternion"])
        return state

# see available ycb objects in the folder:
# objects/ycb_objects
class YCBObject(PyBulletObject):

    def __init__(self, object_name, basePosition=[0.0, 0.0, 0.0], baseOrientation=[0.0, 0.0, 0.0, 1.0], globalScaling=0.08, useFixedBase=False):
        urdfRootPath = "HW3/objects/ycb_objects/"
        self.object = p.loadURDF(os.path.join(urdfRootPath, object_name), basePosition=basePosition, baseOrientation=baseOrientation, globalScaling=globalScaling, useFixedBase=useFixedBase)