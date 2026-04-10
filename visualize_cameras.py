"""
Visualize robot camera and sensor placements.
Uses values from Constants.java and TunerConstants.java.
Run: python visualize_cameras.py
Requires: pip install matplotlib numpy
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# --- Robot frame ---
# Swerve modules at +/-10.875" from center -> frame ~21.75" square
# With bumpers, assume ~25.75" square (2" bumper each side)
FRAME_HALF_X = 10.875  # inches (to wheel center)
FRAME_HALF_Y = 10.875
BUMPER_THICKNESS = 2.5  # inches per side
BUMPER_HALF_X = FRAME_HALF_X + BUMPER_THICKNESS
BUMPER_HALF_Y = FRAME_HALF_Y + BUMPER_THICKNESS

# --- Swerve module positions (inches from robot center) ---
swerve_modules = {
    "FL": (10.875, 10.875),
    "FR": (10.875, -10.875),
    "BL": (-10.875, 10.875),
    "BR": (-10.875, -10.875),
}

# --- Camera positions (inches from robot center) ---
# From Constants.java - Translation3d(X, Y, Z), Rotation3d(roll, pitch, yaw)
cameras = {
    "LEFT_CAMERA": {
        "x": 4.988, "y": 11.903, "z": 5.120,
        "roll": 40, "pitch": 0, "yaw": 90,
        "color": "blue",
    },
    "RIGHT_CAMERA": {
        "x": 4.988, "y": -11.903, "z": 5.120,
        "roll": -40, "pitch": 0, "yaw": -90,
        "color": "red",
    },
    "LIMELIGHT": {
        "x": -10.941, "y": 8.250, "z": 15.934,
        "roll": 0, "pitch": 0, "yaw": 180,
        "color": "green",
    },
}

# --- Quest Nav position (inches from robot center) ---
quest_nav = {
    "x": -9.213, "y": 8.250, "z": 13.303,
    "yaw": 180, "pitch": 0, "roll": 0,
    "color": "purple",
}


def draw_fov_wedge(ax, x, y, yaw_deg, fov_deg=70, length=6, color="blue", alpha=0.15):
    """Draw a camera FOV wedge on the top-down view."""
    start_angle = yaw_deg - fov_deg / 2
    end_angle = yaw_deg + fov_deg / 2
    wedge = patches.Wedge(
        (x, y), length, start_angle, end_angle,
        alpha=alpha, color=color, ec=color, linewidth=0.5
    )
    ax.add_patch(wedge)


def main():
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    fig.suptitle("FRC 2852 Robot - Camera & Sensor Placement", fontsize=14, fontweight="bold")

    # ===================== TOP-DOWN VIEW (X-Y) =====================
    ax_top = axes[0]
    ax_top.set_title("Top-Down View (X = forward, Y = left)")
    ax_top.set_aspect("equal")
    ax_top.set_xlabel("X (inches) - Forward →")
    ax_top.set_ylabel("Y (inches) ← Left")
    ax_top.grid(True, alpha=0.3)

    # Draw bumper perimeter
    bumper = patches.FancyBboxPatch(
        (-BUMPER_HALF_X, -BUMPER_HALF_Y),
        2 * BUMPER_HALF_X, 2 * BUMPER_HALF_Y,
        boxstyle="round,pad=0.5",
        facecolor="lightgray", edgecolor="gray", linewidth=2, alpha=0.4,
        label="Bumpers",
    )
    ax_top.add_patch(bumper)

    # Draw frame perimeter
    frame = patches.Rectangle(
        (-FRAME_HALF_X, -FRAME_HALF_Y),
        2 * FRAME_HALF_X, 2 * FRAME_HALF_Y,
        facecolor="whitesmoke", edgecolor="black", linewidth=2,
        label="Frame",
    )
    ax_top.add_patch(frame)

    # Draw front arrow
    ax_top.annotate(
        "FRONT", xy=(FRAME_HALF_X + 1, 0), fontsize=9, fontweight="bold",
        ha="left", va="center", color="darkgreen",
    )
    ax_top.arrow(FRAME_HALF_X - 2, 0, 3, 0, head_width=1, head_length=0.5,
                 fc="darkgreen", ec="darkgreen")

    # Draw swerve modules
    for name, (sx, sy) in swerve_modules.items():
        ax_top.plot(sx, sy, "ks", markersize=8)
        ax_top.annotate(name, (sx, sy), textcoords="offset points",
                        xytext=(0, -10), ha="center", fontsize=7, color="black")

    # Draw robot center
    ax_top.plot(0, 0, "k+", markersize=12, markeredgewidth=2)

    # Draw cameras (top-down)
    for name, cam in cameras.items():
        ax_top.plot(cam["x"], cam["y"], "o", color=cam["color"], markersize=10,
                    markeredgecolor="black", markeredgewidth=1, zorder=5)
        ax_top.annotate(
            f'{name}\n({cam["x"]:.1f}, {cam["y"]:.1f})',
            (cam["x"], cam["y"]),
            textcoords="offset points", xytext=(8, 8),
            fontsize=7, color=cam["color"], fontweight="bold",
        )
        # FOV wedge - yaw angle in WPILib: 0=forward(+X), 90=left(+Y)
        draw_fov_wedge(ax_top, cam["x"], cam["y"], cam["yaw"], color=cam["color"])

    # Draw Quest Nav (top-down)
    ax_top.plot(quest_nav["x"], quest_nav["y"], "D", color=quest_nav["color"],
                markersize=10, markeredgecolor="black", markeredgewidth=1, zorder=5)
    ax_top.annotate(
        f'QUEST NAV\n({quest_nav["x"]:.1f}, {quest_nav["y"]:.1f})',
        (quest_nav["x"], quest_nav["y"]),
        textcoords="offset points", xytext=(8, 8),
        fontsize=7, color=quest_nav["color"], fontweight="bold",
    )

    ax_top.set_xlim(-22, 22)
    ax_top.set_ylim(-22, 22)

    # ===================== SIDE VIEW (X-Z) =====================
    ax_side = axes[1]
    ax_side.set_title("Side View (X = forward, Z = up)")
    ax_side.set_aspect("equal")
    ax_side.set_xlabel("X (inches) - Forward →")
    ax_side.set_ylabel("Z (inches) - Up ↑")
    ax_side.grid(True, alpha=0.3)

    # Draw ground line
    ax_side.axhline(y=0, color="brown", linewidth=2, linestyle="--", alpha=0.5, label="Ground")

    # Draw frame as a simple rectangle (side profile)
    frame_height = 4  # approximate frame rail height in inches
    frame_side = patches.Rectangle(
        (-FRAME_HALF_X, 0), 2 * FRAME_HALF_X, frame_height,
        facecolor="whitesmoke", edgecolor="black", linewidth=2, label="Frame",
    )
    ax_side.add_patch(frame_side)

    # Draw wheels
    wheel_radius = 2
    for sx, _ in [(-10.875, 0), (10.875, 0)]:
        circle = patches.Circle((sx, wheel_radius), wheel_radius,
                                facecolor="darkgray", edgecolor="black", linewidth=1.5)
        ax_side.add_patch(circle)

    # Draw cameras (side view: X vs Z)
    for name, cam in cameras.items():
        ax_side.plot(cam["x"], cam["z"], "o", color=cam["color"], markersize=10,
                     markeredgecolor="black", markeredgewidth=1, zorder=5)
        ax_side.annotate(
            f'{name}\nZ={cam["z"]:.1f}"',
            (cam["x"], cam["z"]),
            textcoords="offset points", xytext=(8, 8),
            fontsize=7, color=cam["color"], fontweight="bold",
        )

    # Draw Quest Nav (side view)
    ax_side.plot(quest_nav["x"], quest_nav["z"], "D", color=quest_nav["color"],
                 markersize=10, markeredgecolor="black", markeredgewidth=1, zorder=5)
    ax_side.annotate(
        f'QUEST NAV\nZ={quest_nav["z"]:.1f}"',
        (quest_nav["x"], quest_nav["z"]),
        textcoords="offset points", xytext=(8, 8),
        fontsize=7, color=quest_nav["color"], fontweight="bold",
    )

    ax_side.set_xlim(-22, 22)
    ax_side.set_ylim(-2, 22)

    # ===================== LEGEND =====================
    legend_elements = [
        patches.Patch(facecolor="whitesmoke", edgecolor="black", label="Frame (21.75\" sq)"),
        patches.Patch(facecolor="lightgray", edgecolor="gray", label="Bumpers"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="blue",
                   markeredgecolor="black", markersize=10, label="Left Camera"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="red",
                   markeredgecolor="black", markersize=10, label="Right Camera"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="green",
                   markeredgecolor="black", markersize=10, label="Limelight"),
        plt.Line2D([0], [0], marker="D", color="w", markerfacecolor="purple",
                   markeredgecolor="black", markersize=10, label="Quest Nav"),
        plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="black",
                   markersize=8, label="Swerve Modules"),
    ]
    fig.legend(handles=legend_elements, loc="lower center", ncol=4, fontsize=9,
              framealpha=0.9, bbox_to_anchor=(0.5, 0.01))

    plt.tight_layout(rect=[0, 0.08, 1, 0.96])
    plt.savefig("robot_camera_layout.png", dpi=150, bbox_inches="tight")
    print("Saved: robot_camera_layout.png")
    plt.show()


if __name__ == "__main__":
    main()
