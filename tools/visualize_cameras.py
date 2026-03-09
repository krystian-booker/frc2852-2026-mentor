"""
Visualize camera and QuestNav positions/orientations on the robot in 3D.
Run: python tools/visualize_cameras.py
Requires: pip install matplotlib numpy
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Build rotation matrix from roll (X), pitch (Y), yaw (Z) in degrees."""
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])

    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [ 0,         1, 0],
                   [-np.sin(p), 0, np.cos(p)]])

    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0,          0,         1]])

    return Rz @ Ry @ Rx

# --- Configuration (matches Constants.java) ---

# Left camera
left_pos = np.array([0.0244348, 0.2737866, 0.6747256])
left_rot = rotation_matrix(0, -30, -45)

# Right camera
right_pos = np.array([0.0394716, -0.263906, 0.6851142])
right_rot = rotation_matrix(0, -30, 45)

# Limelight 4
limelight_pos = np.array([-0.1651, 0.2421636, 0.5263896])
limelight_rot = rotation_matrix(0, 0, 90)

# QuestNav
quest_pos = np.array([-0.14605, 0.2099564, 0.4172712])
quest_rot = rotation_matrix(0, 0, 90)

# --- Plot ---

fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

# Draw robot footprint (approximate frame perimeter)
frame_half_x = 0.4  # ~80cm long robot
frame_half_y = 0.35  # ~70cm wide robot
corners = np.array([
    [ frame_half_x,  frame_half_y, 0],
    [ frame_half_x, -frame_half_y, 0],
    [-frame_half_x, -frame_half_y, 0],
    [-frame_half_x,  frame_half_y, 0],
    [ frame_half_x,  frame_half_y, 0],  # close the loop
])
ax.plot(corners[:, 0], corners[:, 1], corners[:, 2], 'k-', linewidth=2, label='Robot frame')

# Draw front arrow on robot
ax.quiver(frame_half_x, 0, 0, 0.1, 0, 0, color='black', arrow_length_ratio=0.3, linewidth=2)
ax.text(frame_half_x + 0.12, 0, 0, 'FRONT', fontsize=10, fontweight='bold')

# Arrow length for direction vectors
arrow_len = 0.3

def plot_component(ax, pos, rot, color, label):
    """Plot a component's position and viewing direction."""
    # Forward direction (X-axis of the component, transformed by rotation)
    forward = rot @ np.array([1, 0, 0]) * arrow_len

    # Plot position
    ax.scatter(*pos, color=color, s=100, zorder=5)

    # Plot viewing direction arrow
    ax.quiver(pos[0], pos[1], pos[2],
              forward[0], forward[1], forward[2],
              color=color, arrow_length_ratio=0.2, linewidth=2.5, label=label)

plot_component(ax, left_pos, left_rot, 'blue', 'Left Camera')
plot_component(ax, right_pos, right_rot, 'red', 'Right Camera')
plot_component(ax, limelight_pos, limelight_rot, 'orange', 'Limelight 4')
plot_component(ax, quest_pos, quest_rot, 'green', 'QuestNav')

# Robot center
ax.scatter(0, 0, 0, color='black', s=150, marker='+', linewidths=3, label='Robot center')

# Labels and formatting
ax.set_xlabel('X (Forward) [m]')
ax.set_ylabel('Y (Left) [m]')
ax.set_zlabel('Z (Up) [m]')
ax.set_title('Robot Component Positions & Orientations\n(WPILib: X=Forward, Y=Left, Z=Up, CCW+)')

# Equal aspect ratio
all_pts = np.vstack([left_pos, right_pos, limelight_pos, quest_pos, [0, 0, 0]])
max_range = np.max(np.ptp(all_pts, axis=0)) / 2 * 1.5
mid = np.mean(all_pts, axis=0)
ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
ax.set_zlim(0, max_range * 2)

ax.legend(loc='upper left')

# Set a good default viewing angle
ax.view_init(elev=30, azim=-60)

plt.tight_layout()
plt.show()
