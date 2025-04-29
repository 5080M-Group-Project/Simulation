import pybullet as p
import numpy as np
import pybullet_data
import os
import time
import csv
from scipy.linalg import solve_continuous_are

# Initialize simulation environment
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Parameters
time_step = 1 / 1000
wheel_radius = 0.05
body_mass = 8.128  # kg
robot_height = 0.1736  # vertical distance from wheel axis to COM (used for simplified model)
g = 9.81
max_wheel_speed = 20  # rad/s
MAX_TORQUE = 30  # Nm limit for motors

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

# LQR setup
A = np.array([[0, 1, 0, 0],
              [0, 0, g / robot_height, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

B = np.array([[0, 0],
              [1 / (body_mass * wheel_radius ** 2), 1 / (body_mass * wheel_radius ** 2)],
              [0, 0],
              [1 / (body_mass * wheel_radius), -1 / (body_mass * wheel_radius)]])

Q = np.diag([200, 2, 180, 40])
R = np.diag([0.1, 0.1])
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

# Control targets
base_pitch_offset = -0.09
desired_velocity = 0.0  # POSITIVE = forward now
desired_yaw_rate = 0
desired_knee_angle = 0
desired_hip_angle = -0

# Camera
camera_distance = 0.75
camera_yaw = 50
camera_pitch = -30

# Telemetry storage
telemetry_data = []
time_elapsed = 0
last_position = 0

# Utility: calculate pitch from COM to wheel axis
def compute_body_pitch():
    com_pos, _ = p.getLinkState(robot_id, HIP_JOINT, computeForwardKinematics=True)[0:2]
    base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    dx = com_pos[0] - base_pos[0]
    dz = com_pos[2] - base_pos[2]
    return np.arctan2(dz, dx)

# Control function
def lqr_control(state, desired_velocity, desired_yaw_rate, pitch_offset):
    state_error = np.array([
        state[0] - pitch_offset,
        state[1],
        state[2] - desired_velocity,
        state[3] - desired_yaw_rate
    ])
    wheel_velocities = -K @ state_error
    return np.clip(wheel_velocities, -max_wheel_speed, max_wheel_speed)

# Main loop
try:
    for i in range(10000000):
        time_elapsed += time_step

        # Get kinematics
        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        euler = p.getEulerFromQuaternion(base_orn)
        head_pitch = euler[1]
        yaw_angle = euler[2]
        linear_velocity, angular_velocity = p.getBaseVelocity(robot_id)
        pitch_rate = angular_velocity[1]
        yaw_rate = angular_velocity[2]
        forward_velocity = (base_pos[0] - last_position) / time_step
        last_position = base_pos[0]

        # New corrected pitch
        body_pitch = head_pitch #compute_body_pitch()

        # Run LQR
        state = np.array([body_pitch, pitch_rate, forward_velocity, yaw_rate])
        pitch_offset = base_pitch_offset
        left_wheel_speed, right_wheel_speed = lqr_control(state, desired_velocity, desired_yaw_rate, pitch_offset)

        # Apply control
        p.setJointMotorControl2(robot_id, LEFT_WHEEL, controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=-left_wheel_speed, force=MAX_TORQUE)
        p.setJointMotorControl2(robot_id, RIGHT_WHEEL, controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=-right_wheel_speed, force=MAX_TORQUE)

        p.setJointMotorControl2(robot_id, HIP_JOINT, controlMode=p.POSITION_CONTROL,
                                targetPosition=desired_hip_angle)
        p.setJointMotorControl2(robot_id, KNEE_JOINT, controlMode=p.POSITION_CONTROL,
                                targetPosition=desired_knee_angle)

        # Update camera
        p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                     cameraYaw=camera_yaw,
                                     cameraPitch=camera_pitch,
                                     cameraTargetPosition=base_pos)

        # Print for live feedback
        print(f"Time: {time_elapsed:.2f}s | Body Pitch: {np.degrees(body_pitch):.2f}° | "
              f"Head Pitch: {np.degrees(head_pitch):.2f}° | "
              f"Desired V: {desired_velocity:.2f} | Actual V: {forward_velocity:.2f}")

        # Telemetry log
        telemetry_data.append([
            time_elapsed,
            body_pitch,
            head_pitch,
            pitch_rate,
            forward_velocity,
            left_wheel_speed,
            right_wheel_speed,
            desired_velocity,
            body_pitch - pitch_offset,
            yaw_rate - desired_yaw_rate
        ])

        p.stepSimulation()
        time.sleep(time_step)

finally:
    Q_vals = np.diag(Q).astype(int)
    q_string = "_".join(map(str, Q_vals))
    filename = f"telemetry_Q_{q_string}.csv"

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "Body Pitch (rad)", "Head Pitch (rad)", "Pitch Rate (rad/s)",
                         "Velocity (m/s)", "Left Wheel Speed", "Right Wheel Speed",
                         "Desired Velocity", "Pitch Error", "Yaw Rate Error"])
        writer.writerows(telemetry_data)
    print(f"Telemetry saved to {filename}")
