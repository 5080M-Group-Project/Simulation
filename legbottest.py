import pybullet as p
import numpy as np
import pybullet_data
import os
import time
import csv
from scipy.linalg import solve_continuous_are

# Initialize PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Parameters
time_step = 1 / 1000
wheel_radius = 0.05
body_mass = 8.128  # kg
robot_height = 0.1736  # CoM height from axle (for LQR model)
g = 9.81
max_torque = 30  # Nm

# Load plane and robot
plane_id = p.loadURDF("plane.urdf", 0, 0, -1)
project_path = os.path.expanduser('~/PycharmProjects/Simulation')
spawn_height = -0.65
robot_id = p.loadURDF(os.path.join(project_path, "lucasURDF3/urdf/lucasURDF3.urdf"),
                      basePosition=[0, 0, spawn_height])

# Joint indices
HIP_JOINT = 0
KNEE_JOINT = 1
LEFT_WHEEL = 2
RIGHT_WHEEL = 3

# LQR system
A = np.array([[0, 1, 0, 0],
              [0, 0, g / robot_height, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

B = np.array([[0, 0],
              [1 / (body_mass * wheel_radius ** 2), 1 / (body_mass * wheel_radius ** 2)],
              [0, 0],
              [1 / (body_mass * wheel_radius), -1 / (body_mass * wheel_radius)]])

Q = np.diag([200, 1, 180, 40])
R = np.diag([0.1, 0.1])
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

# Control settings
base_pitch_offset = -0.09
desired_velocity = 0.2  # Forward input (positive now = forward)
desired_yaw_rate = 0
desired_knee_angle = 0
desired_hip_angle = 0

# Logging
log = []

# Helper: Compute COM-based pitch
def compute_body_pitch():
    com_pos, _ = p.getDynamicsInfo(robot_id, -1)[3], None
    wheel_pos = np.array(p.getLinkState(robot_id, LEFT_WHEEL)[0])
    com_world = np.array(p.getBasePositionAndOrientation(robot_id)[0]) + com_pos
    vec = com_world - wheel_pos
    pitch = np.arctan2(vec[2], vec[0])  # angle above horizontal
    return pitch

# Control loop
time_elapsed = 0
last_position = p.getBasePositionAndOrientation(robot_id)[0][0]

for i in range(20000):
    time_elapsed += time_step

    # Get robot state
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    head_pitch = p.getEulerFromQuaternion(orn)[1]
    body_pitch = compute_body_pitch()
    linear_velocity, angular_velocity = p.getBaseVelocity(robot_id)
    pitch_rate = angular_velocity[1]
    forward_velocity = (pos[0] - last_position) / time_step
    last_position = pos[0]
    yaw_rate = angular_velocity[2]

    # LQR control
    pitch_error = body_pitch - base_pitch_offset
    velocity_error = forward_velocity - desired_velocity
    yaw_rate_error = yaw_rate - desired_yaw_rate

    state = np.array([body_pitch, pitch_rate, forward_velocity, yaw_rate])
    wheel_velocities = -K @ np.array([pitch_error, pitch_rate, velocity_error, yaw_rate_error])

    # Approximate velocity control using torque control
    for joint, desired_vel in zip([LEFT_WHEEL, RIGHT_WHEEL], wheel_velocities):
        actual_vel = p.getJointState(robot_id, joint)[1]
        vel_error = desired_vel - actual_vel
        torque = np.clip(2.0 * vel_error, -max_torque, max_torque)  # P controller on velocity
        p.setJointMotorControl2(robot_id, jointIndex=joint,
                                controlMode=p.TORQUE_CONTROL,
                                force=torque)

    # Hip and knee
    p.setJointMotorControl2(robot_id, HIP_JOINT, controlMode=p.POSITION_CONTROL,
                            targetPosition=desired_hip_angle)
    p.setJointMotorControl2(robot_id, KNEE_JOINT, controlMode=p.POSITION_CONTROL,
                            targetPosition=desired_knee_angle)

    # Log data
    log.append([time_elapsed, head_pitch, body_pitch, pitch_rate, forward_velocity,
                desired_velocity, wheel_velocities[0], wheel_velocities[1],
                pitch_error, yaw_rate_error])

    p.stepSimulation()
    time.sleep(time_step)

# Save log to CSV
with open("telemetry_log.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", "head_pitch", "body_pitch", "pitch_rate", "forward_velocity",
                     "desired_velocity", "left_cmd", "right_cmd", "pitch_error", "yaw_rate_error"])
    writer.writerows(log)

print("✅ Telemetry saved to telemetry_log.csv")
