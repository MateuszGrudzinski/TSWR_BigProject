import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1.5]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
p.resetDebugVisualizerCamera(cameraDistance =0.5 ,cameraYaw = 0.0, cameraPitch = -89.99, cameraTargetPosition = [1,1,5])
visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,rgbaColor=[0, 1, 1, 1],specularColor=[0.4, .4, 0],halfExtents=[1, 1, .5],
                                    visualFramePosition=[0.0,-0.0,5])
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                            collisionFramePosition=[0.0,-0.0,5],
                                            halfExtents=[1,1,.5])
rangex = 1
rangey = 1
for i in range(rangex):
  for j in range(rangey):
    Object = p.createMultiBody(baseMass=0.001,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[((-rangex / 2) + i) * 5,
                                    (-rangey / 2 + j) * 5, 1],
                      useMaximalCoordinates=True)
p.changeDynamics(Object,linkIndex =  - 1, restitution = 0.02)
for i in range (100000):
    p.stepSimulation()
    time.sleep(1./100.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
