
#

import pybullet as p
import time

p.connect(p.GUI)

p.setAdditionalSearchPath("../URDF_model")

p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)

# sphereRadius = 0.05
# colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
# colBoxId = p.createCollisionShape(p.GEOM_BOX,
#                                   halfExtents=[sphereRadius, sphereRadius, sphereRadius])

# mass = 1
# visualShapeId = -1

cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("single_body.urdf",cubeStartPos, cubeStartOrientation)

# link_Masses = [1]
# linkCollisionShapeIndices = [colBoxId]
# linkVisualShapeIndices = [-1]
# linkPositions = [[0, 0, 0.11]]
# linkOrientations = [[0, 0, 0, 1]]
# linkInertialFramePositions = [[0, 0, 0]]
# linkInertialFrameOrientations = [[0, 0, 0, 1]]
# indices = [0]
# jointTypes = [p.JOINT_REVOLUTE]
# axis = [[0, 0, 1]]

# basePosition = [0,0,0.5]
# baseOrientation = [0, 0, 0, 1]
     
# sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId, basePosition,
#                                       baseOrientation)
# p.changeDynamics(sphereUid,
#                        -1,
#                        spinningFriction=0.001,
#                        rollingFriction=0.001,
#                        linearDamping=0.0)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

# p.getNumJoints(sphereUid)
# for i in range(p.getNumJoints(sphereUid)):
#   p.getJointInfo(sphereUid, i)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
p.disconnect()