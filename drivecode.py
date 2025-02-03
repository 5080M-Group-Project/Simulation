import pybullet as p
import numpy as np
import pybullet_data
import os
import time


# Initialize simulation environment
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Parameters
time_step = 1 / 240
wheel_radius = 0.05
body_mass = 0.45  # kg
robot_height = 0.15  # height of the center of mass from the wheel axis
g = 9.81  # gravitational acceleration
max_wheel_speed = 20  # Max wheel angular velocity (rad/s)

# Load plane and robot model
plane_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, -1)
project_path = os.path.expanduser('~/PycharmProjects/Simulation')


spawn_height = -0.79  # Ensures wheels touch the ground
robot_id = p.loadURDF(os.path.join(project_path, "Robot-0411-1400/urdf/Robot-0411-1400.urdf"),
                      basePosition=[0, 0, spawn_height])



# LQR Controller parameters
A = np.array([[0, 1, 0, 0],
              [0, 0, g / robot_height, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

B = np.array([[0],
              [1 / (body_mass * wheel_radius ** 2)],
              [0],
              [1 / (body_mass * wheel_radius)]])

Q = np.diag([80, 1, 70, 1])  # Increased penalties for pitch error and velocity error
R = np.array([[0.1]])  # Control cost matrix

# Calculate LQR gain matrix K
P = np.linalg.solve(A.T @ Q + Q @ A - Q @ B @ np.linalg.inv(R) @ B.T @ Q, Q)
K = np.linalg.inv(R) @ B.T @ P


# Adjustable pitch offset
base_pitch_offset = 0  # Target pitch offset (rad)
desired_velocity = 0  # varied by loop

def lqr_control(state, desired_velocity, pitch_offset):
    pitch_error = state[0] - pitch_offset  # Current pitch error
    velocity_error = state[3] - desired_velocity  # Current velocity error
    augmented_state = np.array([pitch_error, state[1], 0, velocity_error])  # x-position not relevant here
    return -K @ augmented_state

# Camera and video settings
camera_distance = 0.75  # Distance from the robot (decrease to zoom in)
camera_yaw = 50        # Horizontal rotation of the camera
camera_pitch = -30     # Vertical angle of the camera
#video_width, video_height = 1920, 1080  # Video resolution
#fps = int(1 / time_step)
#output_file = "simulation_video.mp4"



# Simulation loop
time_elapsed = 0
last_position = 0  # To calculate forward velocity relative to ground

for i in range(10000):
    time_elapsed += time_step

    # Alternate between forward and backward every 5 seconds
    if int(time_elapsed) % 10 < 5:  # First half of a 10-second cycle
        desired_velocity = 2  # Move forward
    else:  # Second half of a 10-second cycle
        desired_velocity = -2  # Move backward

    # Get robot state
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    euler = p.getEulerFromQuaternion(orn)
    pitch_angle = euler[1]  # Pitch angle for balancing
    yaw_angle = euler[2]    # Yaw angle for printing
    linear_velocity, angular_velocity = p.getBaseVelocity(robot_id)
    pitch_rate = angular_velocity[1]  # Pitch angular velocity

    # Calculate forward velocity relative to ground
    forward_velocity = (pos[0] - last_position) / time_step
    last_position = pos[0]

    # Dynamically adjust the target pitch based on the desired velocity
    pitch_offset = base_pitch_offset + 0.03 * desired_velocity  # Empirical adjustment


    # Calculate the LQR input
    state = np.array([pitch_angle, pitch_rate, 0, forward_velocity])  # Adjusted state
    wheel_velocity = float(lqr_control(state, desired_velocity, pitch_offset)[0])  # Extract scalar

    # Clamp wheel velocity
    wheel_velocity = np.clip(wheel_velocity, -max_wheel_speed, max_wheel_speed)

    # Apply the wheel velocities
    p.setJointMotorControl2(robot_id, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=wheel_velocity)
    p.setJointMotorControl2(robot_id, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=wheel_velocity)

    # Follow robot with camera
    p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                 cameraYaw=camera_yaw,
                                 cameraPitch=camera_pitch,
                                 cameraTargetPosition=pos)


    # Print pitch and yaw to console
    print(f"Time: {time_elapsed:.2f}s | Pitch (Tilt): {np.degrees(pitch_angle):.2f}° | Yaw: {np.degrees(yaw_angle):.2f}° | Desired Velocity: {desired_velocity:.2f} m/s")

    # Step simulation
    p.stepSimulation()

    # Sleep to match real-time
    time.sleep(time_step)

    # Stop recording after 10 seconds
    if time_elapsed > 100:
        break

# Release the video writer
