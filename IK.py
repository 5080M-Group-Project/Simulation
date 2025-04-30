import numpy as np

# Link lengths
L1 = 0.165  # Hip to knee
L2 = 0.165  # Knee to wheel

# Desired foot (wheel) position relative to hip
x = 0.0251   # Forward
z = -0.1006  # Downward

# Law of cosines to find knee angle
D = (x**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)

if abs(D) > 1.0:
    raise ValueError("Target out of reach")

theta2 = np.arccos(D)  # knee angle

# Use geometric IK for hip angle
k1 = L1 + L2 * np.cos(theta2)
k2 = L2 * np.sin(theta2)
theta1 = np.arctan2(z, x) - np.arctan2(k2, k1)

# Convert to degrees
hip_angle_deg = np.degrees(theta1)
knee_angle_deg = np.degrees(theta2)

print(f"Hip angle: {hip_angle_deg:.2f}°")
print(f"Knee angle: {knee_angle_deg:.2f}°")
