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
robot_height = 0.1736  # height of the center of mass from the wheel axis
g = 9.81  # gravitational acceleration
max_wheel_speed = 20  # Max wheel angular velocity (rad/s)
MAX_TORQUE = 30  # Nm limit for motors

# Load plane and robot model
plane_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, -1)
project_path = os.path.expanduser('~/PycharmProjects/Simulation')
spawn_height = -0.65  # Adjust to ensure proper contact with ground
robot_id = p.loadURDF(os.path.join(project_path, "lucasURDF3/urdf/lucasURDF3.urdf"),
                      basePosition=[0, 0, spawn_height])

# Joint indices
HIP_JOINT = 0
KNEE_JOINT = 1
LEFT_WHEEL = 2
RIGHT_WHEEL = 3

# LQR Controller setup with desired state vector: [pitch, pitch_rate, forward_velocity, yaw_rate]
A = np.array([[0, 1, 0, 0],
              [0, 0, g / robot_height, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

B = np.array([[0, 0],
              [1 / (body_mass * wheel_radius ** 2), 1 / (body_mass * wheel_radius ** 2)],
              [0, 0],
              [1 / (body_mass * wheel_radius), -1 / (body_mass * wheel_radius)]])  # Left & Right wheel influence

Q = np.diag([200, 2, 180, 40])  # Penalties for pitch error, pitch rate, velocity error, and yaw rate error 200,1,180,10 stationary gains
R = np.diag([0.1, 0.1])  # Control effort for left and right wheel speeds

# Solve the continuous-time algebraic Riccati equation
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P  # Compute LQR gain matrix

# Adjustable offsets
base_pitch_offset =  -0.09 #0.07  # Target pitch offset (rad)
desired_velocity = 0  # Forward velocity input neg=forward
desired_yaw_rate = -0.3  # Turning rate input pos=left
desired_knee_angle = 0  # Control crouch height. pos extends, causes forward drift, neg crouches causing backward drift
desired_hip_angle = -0 # Control head tilt. pos tilt back, causes forward drift
#as knee crouches, head needs to tilt back, both crouch = forward drift
#as knee extends, head tilt must tilt forward = backward drift
#can brute force to find pitch offset relationship to correct drift at each crouch lvl

def lqr_control(state, desired_velocity, desired_yaw_rate, pitch_offset):
    state_error = np.array(
        [state[0] - pitch_offset, state[1], state[2] + desired_velocity, state[3] - desired_yaw_rate])
    wheel_velocities = -K @ state_error  # Compute optimal wheel speeds
    return np.clip(wheel_velocities, -max_wheel_speed, max_wheel_speed)  # Clamp to motor limits


# Camera settings
camera_distance = 0.75
camera_yaw = 50
camera_pitch = -30

# Telemetry storage
telemetry_data = []
time_elapsed = 0
last_position = 0

try:
    for i in range(10000000):
        time_elapsed += time_step

        # Alternate desired velocity every 5 seconds
        #if int(time_elapsed) % 10 < 5:
        #    desired_velocity = 0
        #else:
        #    desired_velocity = 0

        # Get robot state
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        euler = p.getEulerFromQuaternion(orn)

        pitch_angle = euler[1]
        yaw_angle = euler[2]
        linear_velocity, angular_velocity = p.getBaseVelocity(robot_id)
        pitch_rate = angular_velocity[1]
        forward_velocity = (pos[0] - last_position) / time_step
        last_position = pos[0]
        yaw_rate = angular_velocity[2]


        # Dynamic pitch offset
        pitch_offset = base_pitch_offset #+ 0.04 * desired_velocity

        # Compute wheel speeds using LQR
        state = np.array([pitch_angle, pitch_rate, forward_velocity, yaw_rate])
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

        # Update camera position
        p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                     cameraYaw=camera_yaw,
                                     cameraPitch=camera_pitch,
                                     cameraTargetPosition=pos)

        print(
            f"Time: {time_elapsed:.2f}s | Pitch: {np.degrees(pitch_angle-base_pitch_offset):.2f}° | Yaw: {np.degrees(yaw_angle):.2f}° | Desired Velocity: {desired_velocity:.2f} m/s | Actual forward velocity: {-1*forward_velocity:.2f} m/s")

        # Telemetry log
        telemetry_data.append([
            time_elapsed,
            np.degrees(pitch_angle-base_pitch_offset),
            pitch_rate,
            -1*forward_velocity,
            desired_velocity,
            np.degrees(yaw_angle)
        ])

        p.stepSimulation()
        time.sleep(time_step)
finally:
    Q_vals = np.diag(Q).astype(int)
    q_string = "_".join(map(str, Q_vals))
    filename = f"telemetry_Q_{q_string}simple.csv"

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "Pitch (rad)", "Pitch Rate (rad/s)",
                         "Velocity (m/s)", "Desired Velocity", "Yaw (°)"])
        writer.writerows(telemetry_data)
    print(f"Telemetry saved to {filename}")