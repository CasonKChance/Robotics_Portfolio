### GENERATED CODE - DO NOT EDIT MANUALLY ###

import os
import sys

# Configure Matplotlib for headless execution before importing pyplot
import matplotlib
matplotlib.use('Agg')

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Apply aesthetic style
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 10

def normalize_angle(angle_rad):
    """Normalize angles to the range [-pi, pi]."""
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi

def analyze_simulation(csv_path: str, world_csv_path: str, output_image_path: str, trajectory_image_path: str, output_text_path: str):
    # -------------------------------------------------------------------------
    # 1. Load Telemetry Data
    # -------------------------------------------------------------------------
    if not os.path.exists(csv_path):
        sys.exit(1)

    df = pd.read_csv(csv_path)

    required_cols = {'time', 'x', 'y', 'theta', 'linear_velocity', 'angular_velocity'}
    if not required_cols.issubset(df.columns):
        sys.exit(1)

    # Extract series
    t = df['time'].to_numpy()
    x = df['x'].to_numpy()
    y = df['y'].to_numpy()
    theta = df['theta'].to_numpy()
    v = df['linear_velocity'].to_numpy()
    w = df['angular_velocity'].to_numpy()

    n_samples = len(t)
    if n_samples == 0:
        sys.exit(1)

    # -------------------------------------------------------------------------
    # 2. Compute Target Reference Pose & Error Accumulation
    # -------------------------------------------------------------------------
    x_ref = np.zeros(n_samples)
    y_ref = np.zeros(n_samples)
    theta_ref = np.zeros(n_samples)

    # Initial condition alignment
    x_ref[0] = x[0]
    y_ref[0] = y[0]
    theta_ref[0] = theta[0]

    # Integrate commanded velocities forward to obtain expected trajectory
    for i in range(1, n_samples):
        dt = t[i] - t[i-1]
        theta_ref[i] = theta_ref[i-1] + w[i-1] * dt
        x_ref[i] = x_ref[i-1] + v[i-1] * np.cos(theta_ref[i-1]) * dt
        y_ref[i] = y_ref[i-1] + v[i-1] * np.sin(theta_ref[i-1]) * dt

    # Normalize theta values to [-pi, pi]
    theta_norm = normalize_angle(theta)
    theta_ref_norm = normalize_angle(theta_ref)

    # Error metrics vs ideal reference trajectory
    pos_error = np.sqrt((x - x_ref)**2 + (y - y_ref)**2)
    heading_error = normalize_angle(theta - theta_ref)

    # -------------------------------------------------------------------------
    # 3. Compute Metrics for Text Report
    # -------------------------------------------------------------------------
    dx = np.diff(x)
    dy = np.diff(y)
    step_distances = np.sqrt(dx**2 + dy**2)
    
    total_distance = np.sum(step_distances)
    simulation_duration = t[-1] - t[0] if len(t) > 0 else 0.0
    max_v = np.max(np.abs(v))
    max_w = np.max(np.abs(w))
    avg_v = np.mean(v)

    final_x, final_y, final_theta = x[-1], y[-1], theta[-1]
    
    dt_arr = np.diff(t)
    integrated_distance = np.sum(v[:-1] * dt_arr) if len(dt_arr) > 0 else 0.0
    integration_error = abs(total_distance - integrated_distance)

    final_heading_error_rad = heading_error[-1]

    report_lines = [
        "=" * 55,
        "        SIMULATION METRICS & PERFORMANCE REPORT        ",
        "=" * 55,
        f" Simulation Duration      : {simulation_duration:.4f} s",
        f" Total Distance Traveled  : {total_distance:.4f} m",
        f" Final Position (X, Y)    : ({final_x:.4f} m, {final_y:.4f} m)",
        f" Final Position Error     : {pos_error[-1]:.4f} m (vs target ref)",
        f" Final Heading (Theta)    : {final_theta:.4f} rad ({np.degrees(final_theta):.2f}°)",
        f" Final Heading Error      : {abs(final_heading_error_rad):.4f} rad ({abs(np.degrees(final_heading_error_rad)):.2f}°)",
        f" Maximum Linear Speed     : {max_v:.4f} m/s",
        f" Average Linear Speed     : {avg_v:.4f} m/s",
        f" Maximum Angular Speed    : {max_w:.4f} rad/s",
        f" Numerical Integration Err: {integration_error:.6e} m",
        "=" * 55,
    ]
    report_content = "\n".join(report_lines)

    text_dir = os.path.dirname(output_text_path)
    if text_dir and not os.path.exists(text_dir):
        os.makedirs(text_dir)

    with open(output_text_path, 'w', encoding='utf-8') as f:
        f.write(report_content + "\n")

    # Color palette
    c_primary = '#1f77b4'   # Muted Blue
    c_secondary = '#ff7f0e' # Muted Orange
    c_tertiary = '#2ca02c'  # Muted Green
    c_accent = '#d62728'    # Red
    c_dark = '#333333'      # Dark Neutral

    # -------------------------------------------------------------------------
    # 4. Generate Standalone Trajectory Plot (trajectory.png)
    # -------------------------------------------------------------------------
    fig_traj, ax_traj = plt.subplots(figsize=(8, 8), dpi=120)

    # --- Load World Data Log (Obstacles, Goal, World Bounds) ---
    world_bounds = None
    if os.path.exists(world_csv_path):
        world_df = pd.read_csv(world_csv_path)
        world_df.columns = world_df.columns.str.strip()

        obstacle_labeled = False
        goal_labeled = False

        for _, row in world_df.iterrows():
            wx, wy = row['x'], row['y']
            r = row['radius']
            is_goal = int(row['is_goal'])
            is_bounds = int(row['is_world_bounds'])

            if is_bounds == 1:
                world_bounds = (wx, wy)
            elif is_goal == 1:
                label = 'Goal Region' if not goal_labeled else None
                goal_circle = patches.Circle((wx, wy), r, color='gold', alpha=0.6, ec='darkgoldenrod', lw=2, label=label)
                ax_traj.add_patch(goal_circle)
                goal_labeled = True
            else:
                label = 'Obstacle' if not obstacle_labeled else None
                obs_circle = patches.Circle((wx, wy), r, color='red', alpha=0.4, ec='darkred', lw=1.5, label=label)
                ax_traj.add_patch(obs_circle)
                obstacle_labeled = True

    # --- Plot Continuous Trajectory Line ---
    ax_traj.plot(x, y, color=c_primary, linestyle='-', linewidth=1.5, alpha=0.4, label='Path Trajectory')

    # --- Extract 1-Second Interval Indices ---
    second_indices = [
        i for i, time_val in enumerate(t) 
        if np.isclose(time_val % 1.0, 0.0, atol=1e-5) or np.isclose(time_val % 1.0, 1.0, atol=1e-5)
    ]
    
    intermediate_indices = [idx for idx in second_indices if idx != 0 and idx != len(t) - 1]

    # Shared arrow styling properties
    arrow_scale = 3.5
    arrow_width = 0.005
    arrow_headwidth = 4
    arrow_headlength = 5
    arrow_headaxislength = 4.5

    # --- Plot Intermediate 1-Second Pose Arrows (Blue) ---
    if intermediate_indices:
        u_inter = np.cos(theta[intermediate_indices])
        v_inter = np.sin(theta[intermediate_indices])

        ax_traj.quiver(
            x[intermediate_indices], y[intermediate_indices], u_inter, v_inter,
            color=c_primary, angles='xy', scale_units='xy', scale=arrow_scale,
            width=arrow_width, headwidth=arrow_headwidth, headlength=arrow_headlength,
            headaxislength=arrow_headaxislength, label='1s Interval Pose'
        )

    # --- Plot Start Position Arrow (Green) ---
    ax_traj.quiver(
        x[0], y[0], np.cos(theta[0]), np.sin(theta[0]),
        color='green', angles='xy', scale_units='xy', scale=arrow_scale,
        width=arrow_width, headwidth=arrow_headwidth, headlength=arrow_headlength,
        headaxislength=arrow_headaxislength, zorder=5, label='Start Pose'
    )

    # --- Plot End Position Arrow (Red) ---
    ax_traj.quiver(
        x[-1], y[-1], np.cos(theta[-1]), np.sin(theta[-1]),
        color='red', angles='xy', scale_units='xy', scale=arrow_scale,
        width=arrow_width, headwidth=arrow_headwidth, headlength=arrow_headlength,
        headaxislength=arrow_headaxislength, zorder=5, label='End Pose'
    )

    # --- Apply World Limits and Boundary Box ---
    if world_bounds:
        max_x, max_y = world_bounds
        ax_traj.set_xlim([0, max_x])
        ax_traj.set_ylim([0, max_y])
        
        # Draw solid black world boundary box
        rect = patches.Rectangle((0, 0), max_x, max_y, linewidth=2, edgecolor='black', facecolor='none', linestyle='-', label='World Boundary')
        ax_traj.add_patch(rect)

    ax_traj.set_title("2D Spatial Trajectory with Obstacles and Orientation", fontweight='bold')
    ax_traj.set_xlabel("X Position [m]")
    ax_traj.set_ylabel("Y Position [m]")
    ax_traj.set_aspect('equal', adjustable='box')
    ax_traj.legend(loc='upper right', frameon=True, fontsize=8)
    ax_traj.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    traj_img_dir = os.path.dirname(trajectory_image_path)
    if traj_img_dir and not os.path.exists(traj_img_dir):
        os.makedirs(traj_img_dir)
    plt.savefig(trajectory_image_path, dpi=300)
    plt.close(fig_traj)

    # -------------------------------------------------------------------------
    # 5. Generate 2x2 Performance Metrics Grid (simulation_analysis_plot.png)
    # -------------------------------------------------------------------------
    fig, axs = plt.subplots(2, 2, figsize=(13, 9), dpi=120)
    fig.suptitle("Robot Simulation Analysis Summary", fontsize=14, fontweight='bold', y=0.98)

    # --- Plot 1: Heading Orientation vs Time (Normalized Radians [-pi, pi]) ---
    ax1 = axs[0, 0]
    ax1.plot(t, theta_norm, color=c_tertiary, linewidth=1.8, label='θ Actual [rad]')
    ax1.plot(t, theta_ref_norm, color=c_tertiary, linestyle='--', alpha=0.7, label='θ Target [rad]')
    ax1.set_title("Heading Orientation (θ) vs. Time", fontweight='bold')
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Orientation [rad]")
    ax1.set_ylim([-np.pi - 0.2, np.pi + 0.2])
    ax1.legend(loc='best', frameon=True)
    ax1.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 2: Velocity Commands vs Time ---
    ax2 = axs[0, 1]
    ax2.plot(t, v, color=c_primary, linestyle='--', linewidth=1.8, label='Linear Speed (v) [m/s]')
    ax2_twin = ax2.twinx()
    ax2_twin.plot(t, w, color=c_accent, linestyle='--', linewidth=1.5, label='Angular Speed (ω) [rad/s]')
    ax2.set_title("Command Velocities vs. Time", fontweight='bold')
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Linear Velocity [m/s]", color=c_primary)
    ax2_twin.set_ylabel("Angular Velocity [rad/s]", color=c_accent)
    ax2.tick_params(axis='y', labelcolor=c_primary)
    ax2_twin.tick_params(axis='y', labelcolor=c_accent)
    ax2.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 3: Heading Error vs. Time ---
    ax3 = axs[1, 0]
    ax3.plot(t, heading_error, color=c_accent, linewidth=1.8, label='Heading Error [rad]')
    ax3.axhline(0.0, color=c_dark, linestyle=':', linewidth=1.5, label='Target (0 rad)')
    ax3.set_title("Heading Error vs. Time", fontweight='bold')
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Error [rad]")
    ax3.set_ylim([-np.pi - 0.2, np.pi + 0.2])
    ax3.legend(loc='best', frameon=True)
    ax3.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 4: Position Error vs. Time ---
    ax4 = axs[1, 1]
    ax4.plot(t, pos_error, color=c_secondary, linewidth=1.8, label='Position Error [m]')
    ax4.axhline(0.0, color=c_dark, linestyle=':', linewidth=1.5, label='Target (0 m)')
    ax4.set_title("Position Error vs. Time", fontweight='bold')
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Error [m]")
    ax4.legend(loc='best', frameon=True)
    ax4.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    output_img_dir = os.path.dirname(output_image_path)
    if output_img_dir and not os.path.exists(output_img_dir):
        os.makedirs(output_img_dir)

    plt.savefig(output_image_path, dpi=300)
    plt.close(fig)

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    
    csv_file = os.path.join(project_root, "build", "output", "SimulatorDataLog.csv")
    world_csv_file = os.path.join(project_root, "build", "output", "WorldDataLog.csv")
    output_plot = os.path.join(script_dir, "simulation_analysis_plot.png")
    trajectory_plot = os.path.join(script_dir, "trajectory.png")
    output_text = os.path.join(script_dir, "simulation_analysis_report.txt")

    analyze_simulation(csv_file, world_csv_file, output_plot, trajectory_plot, output_text)