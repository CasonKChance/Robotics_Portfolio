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
from matplotlib.lines import Line2D

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
    required_cols = {'time', 'x', 'y', 'theta', 'command_linear_velocity', 'actual_linear_velocity', 'command_angular_velocity', 'actual_angular_velocity'}
    if not required_cols.issubset(df.columns) or len(df) == 0:
        sys.exit(1)

    t, x, y, theta = df['time'].to_numpy(), df['x'].to_numpy(), df['y'].to_numpy(), df['theta'].to_numpy()
    v_cmd, v_act = df['command_linear_velocity'].to_numpy(), df['actual_linear_velocity'].to_numpy()
    w_cmd, w_act = df['command_angular_velocity'].to_numpy(), df['actual_angular_velocity'].to_numpy()

    # -------------------------------------------------------------------------
    # 2. Compute Target Reference Pose & Error Metrics
    # -------------------------------------------------------------------------
    dt_arr = np.diff(t)
    theta_ref = np.concatenate([[theta[0]], theta[0] + np.cumsum(w_cmd[:-1] * dt_arr)])
    x_ref = np.concatenate([[x[0]], x[0] + np.cumsum(v_cmd[:-1] * np.cos(theta_ref[:-1]) * dt_arr)])
    y_ref = np.concatenate([[y[0]], y[0] + np.cumsum(v_cmd[:-1] * np.sin(theta_ref[:-1]) * dt_arr)])

    theta_norm = normalize_angle(theta)
    theta_ref_norm = normalize_angle(theta_ref)
    pos_error = np.sqrt((x - x_ref)**2 + (y - y_ref)**2)
    heading_error = normalize_angle(theta - theta_ref)

    # -------------------------------------------------------------------------
    # 3. Save Text Report
    # -------------------------------------------------------------------------
    step_distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    total_distance = np.sum(step_distances)
    integrated_distance = np.sum(v_act[:-1] * dt_arr) if len(dt_arr) > 0 else 0.0
    final_heading_err = heading_error[-1]

    report_content = "\n".join([
        "=" * 55,
        "        SIMULATION METRICS & PERFORMANCE REPORT        ",
        "=" * 55,
        f" Simulation Duration      : {t[-1] - t[0]:.4f} s",
        f" Total Distance Traveled  : {total_distance:.4f} m",
        f" Final Position (X, Y)    : ({x[-1]:.4f} m, {y[-1]:.4f} m)",
        f" Final Position Error     : {pos_error[-1]:.4f} m (vs target ref)",
        f" Final Heading (Theta)    : {theta[-1]:.4f} rad ({np.degrees(theta[-1]):.2f}°)",
        f" Final Heading Error      : {abs(final_heading_err):.4f} rad ({abs(np.degrees(final_heading_err)):.2f}°)",
        f" Maximum Linear Speed     : {np.max(np.abs(v_act)):.4f} m/s",
        f" Average Linear Speed     : {np.mean(v_act):.4f} m/s",
        f" Maximum Angular Speed    : {np.max(np.abs(w_act)):.4f} rad/s",
        f" Numerical Integration Err: {abs(total_distance - integrated_distance):.6e} m",
        "=" * 55,
    ])

    os.makedirs(os.path.dirname(output_text_path) or '.', exist_ok=True)
    with open(output_text_path, 'w', encoding='utf-8') as f:
        f.write(report_content + "\n")

    c_primary, c_secondary, c_tertiary, c_accent, c_dark = '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#333333'

    # -------------------------------------------------------------------------
    # 4. Generate Standalone Trajectory Plot (trajectory.png)
    # -------------------------------------------------------------------------
    fig_traj, ax_traj = plt.subplots(figsize=(8, 8), dpi=120)

    world_bounds = None
    legend_handles = []

    if os.path.exists(world_csv_path):
        world_df = pd.read_csv(world_csv_path)
        world_df.columns = world_df.columns.str.strip()
        obs_lbl, goal_lbl = False, False

        for _, row in world_df.iterrows():
            wx, wy, r = row['x'], row['y'], row['radius']
            if row['is_world_bounds'] == 1:
                world_bounds = (wx, wy)
            elif row['is_goal'] == 1:
                p = patches.Circle((wx, wy), r, color='gold', alpha=0.6, ec='darkgoldenrod', lw=2)
                ax_traj.add_patch(p)
                if not goal_lbl:
                    legend_handles.append(patches.Patch(facecolor='gold', edgecolor='darkgoldenrod', alpha=0.6, label='Goal Region'))
                    goal_lbl = True
            else:
                p = patches.Circle((wx, wy), r, color='red', alpha=0.4, ec='darkred', lw=1.5)
                ax_traj.add_patch(p)
                if not obs_lbl:
                    legend_handles.append(patches.Patch(facecolor='red', edgecolor='darkred', alpha=0.4, label='Obstacle'))
                    obs_lbl = True

    line_path, = ax_traj.plot(x, y, color=c_primary, linestyle='-', linewidth=2.0, alpha=0.9)
    legend_handles.append(Line2D([0], [0], color=c_primary, lw=2.0, label='Path Trajectory'))

    # Arrow plotting helper
    q_kw = dict(angles='xy', scale_units='xy', scale=3.5, width=0.005, headwidth=4, headlength=5, headaxislength=4.5)
    
    sec_idx = [i for i, val in enumerate(t) if np.isclose(val % 1.0, 0.0, atol=1e-5) or np.isclose(val % 1.0, 1.0, atol=1e-5)]
    inter_idx = [idx for idx in sec_idx if idx not in (0, len(t) - 1)]

    if inter_idx:
        ax_traj.quiver(x[inter_idx], y[inter_idx], np.cos(theta[inter_idx]), np.sin(theta[inter_idx]), color=c_primary, **q_kw)
        legend_handles.append(Line2D([0], [0], color=c_primary, marker='>', linestyle='None', markersize=7, label='1s Interval Pose'))

    ax_traj.quiver(x[0], y[0], np.cos(theta[0]), np.sin(theta[0]), color='green', **q_kw)
    legend_handles.append(Line2D([0], [0], color='green', marker='>', linestyle='None', markersize=7, label='Start Pose'))

    ax_traj.quiver(x[-1], y[-1], np.cos(theta[-1]), np.sin(theta[-1]), color='red', **q_kw)
    legend_handles.append(Line2D([0], [0], color='red', marker='>', linestyle='None', markersize=7, label='End Pose'))

    if world_bounds:
        max_x, max_y = world_bounds
        ax_traj.set_xlim([0, max_x])
        ax_traj.set_ylim([0, max_y])
        ax_traj.add_patch(patches.Rectangle((0, 0), max_x, max_y, linewidth=2, edgecolor='black', facecolor='none'))
        legend_handles.append(patches.Patch(facecolor='none', edgecolor='black', lw=2, label='World Boundary'))

    ax_traj.set_title("2D Spatial Trajectory with Obstacles and Orientation", fontweight='bold', loc='left')
    ax_traj.set_xlabel("X Position [m]")
    ax_traj.set_ylabel("Y Position [m]")
    ax_traj.set_aspect('equal', adjustable='box')
    ax_traj.legend(handles=legend_handles, bbox_to_anchor=(1.0, 1.15), loc='upper right', frameon=True, fontsize=8, ncol=2)
    ax_traj.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    os.makedirs(os.path.dirname(trajectory_image_path) or '.', exist_ok=True)
    plt.savefig(trajectory_image_path, dpi=300)
    plt.close(fig_traj)

    # -------------------------------------------------------------------------
    # 5. Generate 2x2 Performance Metrics Grid (simulation_analysis_plot.png)
    # -------------------------------------------------------------------------
    fig, axs = plt.subplots(2, 2, figsize=(13, 9.5), dpi=120)
    fig.suptitle("Robot Simulation Analysis", fontsize=14, fontweight='bold', y=0.99)

    # Plot 1: Heading Orientation vs Time
    ax1 = axs[0, 0]
    ax1.plot(t, theta_norm, color=c_tertiary, linewidth=2.0, label='θ Actual [rad]')
    ax1.plot(t, theta_ref_norm, color=c_tertiary, linestyle='--', alpha=0.8, linewidth=1.5, label='θ Target [rad]')
    ax1.set_title("Heading Orientation (θ) vs. Time", fontweight='bold', loc='left')
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Orientation [rad]")
    ax1.set_xlim(left=0)
    ax1.set_ylim([-np.pi - 0.2, np.pi + 0.2])
    ax1.legend(bbox_to_anchor=(1.0, 1.15), loc='upper right', frameon=True, fontsize=8)
    ax1.grid(True, linestyle='--', alpha=0.6)

    # Plot 2: Velocities vs Time
    ax2 = axs[0, 1]
    ax2_twin = ax2.twinx()
    l1 = ax2.plot(t, v_act, color=c_primary, linestyle='-', linewidth=2.2, label='Actual Linear Velocity [m/s]')
    l2 = ax2.plot(t, v_cmd, color=c_primary, linestyle='--', linewidth=1.8, label='Command Linear Velocity [m/s]')
    l3 = ax2_twin.plot(t, w_act, color=c_accent, linestyle='-', linewidth=2.2, label='Actual Angular Velocity [rad/s]')
    l4 = ax2_twin.plot(t, w_cmd, color=c_accent, linestyle='--', linewidth=1.8, label='Command Angular Velocity [rad/s]')

    ax2.set_title("Velocities vs. Time", fontweight='bold', loc='left')
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Linear Velocity [m/s]", color=c_primary)
    ax2_twin.set_ylabel("Angular Velocity [rad/s]", color=c_accent)
    ax2.tick_params(axis='y', labelcolor=c_primary)
    ax2_twin.tick_params(axis='y', labelcolor=c_accent)
    ax2.set_xlim(left=0)
    ax2.axhline(0, color='#cccccc', linewidth=1.0)
    ax2.axvline(0, color='#cccccc', linewidth=1.0)

    ax2.legend(l1 + l2 + l3 + l4, [l.get_label() for l in l1 + l2 + l3 + l4], bbox_to_anchor=(1.0, 1.18), loc='upper right', frameon=True, fontsize=7, ncol=2)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2_twin.grid(False)

    # Plot 3: Heading Error vs. Time
    ax3 = axs[1, 0]
    ax3.plot(t, heading_error, color=c_accent, linewidth=2.0, label='Heading Error [rad]')
    ax3.axhline(0.0, color=c_dark, linestyle=':', linewidth=1.5, label='Target (0 rad)')
    ax3.set_title("Heading Error vs. Time", fontweight='bold', loc='left')
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Error [rad]")
    ax3.set_xlim(left=0)
    ax3.set_ylim([-np.pi - 0.2, np.pi + 0.2])
    ax3.legend(bbox_to_anchor=(1.0, 1.15), loc='upper right', frameon=True, fontsize=8)
    ax3.grid(True, linestyle='--', alpha=0.6)

    # Plot 4: Position Error vs. Time
    ax4 = axs[1, 1]
    ax4.plot(t, pos_error, color=c_secondary, linewidth=2.0, label='Position Error [m]')
    ax4.set_title("Position Error vs. Time", fontweight='bold', loc='left')
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Error [m]")
    ax4.set_xlim(left=0)
    ax4.set_ylim(bottom=0)
    ax4.legend(bbox_to_anchor=(1.0, 1.15), loc='upper right', frameon=True, fontsize=8)
    ax4.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(os.path.dirname(output_image_path) or '.', exist_ok=True)
    plt.savefig(output_image_path, dpi=300)
    plt.close(fig)

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))

    analyze_simulation(
        os.path.join(project_root, "build", "output", "SimulatorDataLog.csv"),
        os.path.join(project_root, "build", "output", "WorldDataLog.csv"),
        os.path.join(script_dir, "simulation_analysis_plot.png"),
        os.path.join(script_dir, "trajectory.png"),
        os.path.join(script_dir, "simulation_analysis_report.txt")
    )