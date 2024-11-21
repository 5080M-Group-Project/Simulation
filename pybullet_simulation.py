import pybullet
import pybullet_data
import os
import time

pybullet.connect(pybullet.GUI)
plane_id = pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot_id = pybullet.loadURDF(os.path.join(os.getcwd(), "robot_description/urdf/simple_robot.urdf"),
                             basePosition=[0, 0, 0.1],
                             useFixedBase=False)
pybullet.setGravity(0, 0, -9.81)

while True:
    pybullet.stepSimulation()
    posAndOrn = pybullet.getBasePositionAndOrientation(robot_id)

    # jointIndex = 1 refers to the left wheel
    pybullet.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=1, controlMode=pybullet.VELOCITY_CONTROL,
                                   targetVelocity=0.0)
    # jointIndex = 0 refers to the right wheel
    pybullet.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=0, controlMode=pybullet.VELOCITY_CONTROL,
                                   targetVelocity=-0.0)
    time.sleep(1 / 240)
