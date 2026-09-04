import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Apply a clean aesthetic style
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 10

def analyze_simulation(csv_path: str, output_image_path: str):
    # -------------------------------------------------------------------------
    # 1. Load Data
    # -------------------------------------------------------------------------
    if not os.path.exists(csv_path):
        print(f"Error: Target CSV file not found at '{csv_path}'")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    required_cols = {'time', 'x', 'y', 'theta', 'linear_velocity', 'angular_velocity'}
    if not required_cols.issubset(df.columns):
        print(f"Error: CSV missing required headers. Found: {list(df.columns)}")
        sys.exit(1)

    # Extract series
    t = df['time'].to_numpy()
    x = df['x'].to_numpy()
    y = df['y'].to_numpy()
    theta = df['theta'].to_numpy()
    v = df['linear_velocity'].to_numpy()
    w = df['angular_velocity'].to_numpy()

    # -------------------------------------------------------------------------
    # 2. Compute Metrics
    # -------------------------------------------------------------------------
    dt = np.diff(t)
    
    # Differential motion deltas
    dx = np.diff(x)
    dy = np.diff(y)
    step_distances = np.sqrt(dx**2 + dy**2)
    
    # Core Summary Statistics
    total_distance = np.sum(step_distances)
    simulation_duration = t[-1] - t[0] if len(t) > 0 else 0.0
    max_v = np.max(np.abs(v))
    max_w = np.max(np.abs(w))
    avg_v = np.mean(v)

    # Final positions & simple target comparison (assuming origin/start target reference)
    final_x, final_y, final_theta = x[-1], y[-1], theta[-1]
    
    # Expected distance from kinematic integration of linear speed vs actual path step sum
    integrated_distance = np.sum(v[:-1] * dt)
    integration_error = abs(total_distance - integrated_distance)

    # Heading error wrap to [-pi, pi] relative to 0 endpoint target reference
    final_heading_error_rad = (final_theta + np.pi) % (2 * np.pi) - np.pi

    # Print Formatted Log
    print("=" * 50)
    print("        SIMULATION METRICS & ANALYSIS LOG        ")
    print("=" * 50)
    print(f" Simulation Duration      : {simulation_duration:.4f} s")
    print(f" Total Distance Traveled  : {total_distance:.4f} m")
    print(f" Final Position (X, Y)    : ({final_x:.4f} m, {final_y:.4f} m)")
    print(f" Final Position Error     : {np.sqrt(final_x**2 + final_y**2):.4f} m (from origin)")
    print(f" Final Heading (Theta)    : {final_theta:.4f} rad ({np.degrees(final_theta):.2f}°)")
    print(f" Final Heading Error      : {abs(final_heading_error_rad):.4f} rad ({abs(np.degrees(final_heading_error_rad)):.2f}°)")
    print(f" Maximum Linear Speed     : {max_v:.4f} m/s")
    print(f" Average Linear Speed     : {avg_v:.4f} m/s")
    print(f" Maximum Angular Speed    : {max_w:.4f} rad/s")
    print(f" Numerical Integration Err: {integration_error:.6e} m")
    print("=" * 50)

    # -------------------------------------------------------------------------
    # 3. Generate Plots
    # -------------------------------------------------------------------------
    fig, axs = plt.subplots(2, 2, figsize=(13, 9), dpi=120)
    fig.suptitle("Robot Simulation Analysis Summary", fontsize=14, fontweight='bold', y=0.98)

    # Color palette
    c_primary = '#1f77b4'   # Muted Blue
    c_secondary = '#ff7f0e' # Muted Orange
    c_tertiary = '#2ca02c'  # Muted Green
    c_accent = '#d62728'    # Red

    # --- Plot 1: Trajectory (X vs Y) ---
    ax1 = axs[0, 0]
    ax1.plot(x, y, color=c_primary, linewidth=2, label='Robot Path')
    ax1.plot(x[0], y[0], 'go', markersize=8, label='Start')
    ax1.plot(x[-1], y[-1], 'ro', markersize=8, label='End')
    ax1.set_title("2D Spatial Trajectory", fontweight='semibold')
    ax1.set_xlabel("X Position [m]")
    ax1.set_ylabel("Y Position [m]")
    ax1.axis('equal')
    ax1.legend(loc='best', frameon=True)
    ax1.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 2: Position (X & Y vs Time) ---
    ax2 = axs[0, 1]
    ax2.plot(t, x, color=c_primary, linewidth=1.8, label='X [m]')
    ax2.plot(t, y, color=c_secondary, linewidth=1.8, label='Y [m]')
    ax2.set_title("Position Components vs. Time", fontweight='semibold')
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Position [m]")
    ax2.legend(loc='best', frameon=True)
    ax2.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 3: Orientation (Theta vs Time) ---
    ax3 = axs[1, 0]
    ax3.plot(t, np.degrees(theta), color=c_tertiary, linewidth=1.8, label='θ [deg]')
    ax3.plot(t, theta, color=c_tertiary, linestyle=':', alpha=0.5, label='θ [rad]')
    ax3.set_title("Heading Orientation (θ) vs. Time", fontweight='semibold')
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Orientation [deg / rad]")
    ax3.legend(loc='best', frameon=True)
    ax3.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 4: Velocity Commands vs Time ---
    ax4 = axs[1, 1]
    ax4.plot(t, v, color=c_primary, linewidth=1.8, label='Linear Speed (v) [m/s]')
    ax4_twin = ax4.twinx()
    ax4_twin.plot(t, w, color=c_accent, linestyle='--', linewidth=1.5, label='Angular Speed (ω) [rad/s]')
    
    ax4.set_title("Command Velocities vs. Time", fontweight='semibold')
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Linear Velocity [m/s]", color=c_primary)
    ax4_twin.set_ylabel("Angular Velocity [rad/s]", color=c_accent)
    ax4.tick_params(axis='y', labelcolor=c_primary)
    ax4_twin.tick_params(axis='y', labelcolor=c_accent)
    ax4.grid(True, linestyle='--', alpha=0.6)

    # Save and output layout
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    output_dir = os.path.dirname(output_image_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    plt.savefig(output_image_path, dpi=300)
    print(f"\n[INFO] Plot successfully generated and saved to: {output_image_path}")
    plt.show()

if __name__ == "__main__":
    # Relative path anchors
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    
    csv_file = os.path.join(project_root, "build", "output", "SimulatorDataLog.csv")
    output_plot = os.path.join(script_dir, "simulation_analysis_plot.png")

    analyze_simulation(csv_file, output_plot)