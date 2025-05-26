import subprocess
import time
import itertools
import signal
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import csv
import seaborn as sns

# --- Configuration ---
percentage_lost = [0.0, 0.1, 1.0, 5.0, 10.0]
communication_delay = [0.0, 0.025, 0.050, 0.100, 0.150]  # in milliseconds
simulation_time = 20  # seconds
safety_distance = 0.5  # meters
n_iter = 5
n_drones = 6  # Adjust to match your number of drones

# Generate all combinations
combinations = list(itertools.product(percentage_lost, communication_delay))

# Data storage for results:
# Keys are (packet_loss, delay), values are lists for all iterations
mean_velocities_dict = {combo: [] for combo in combinations}
min_distances_dict = {combo: [] for combo in combinations}

# --- Helper function to clean up lingering nodes ---
def kill_ros_nodes():
    patterns = ["map_builder_node", "agent_node"]
    for pattern in patterns:
        print(f"[INFO] Killing any remaining '{pattern}' processes...")
        subprocess.run(["pkill", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# --- Helper function to load and process trajectory data ---
def process_trajectories(sim_id):
    trajectories = []
    avg_velocities = []
    violations = False
    min_distance = float('inf')

    for drone_id in range(n_drones):
        file_path = f"state_hist_{drone_id}.csv"
        if not os.path.exists(file_path):
            print(f"[WARN] Missing file: {file_path}")
            continue

        columns = ["timestamp", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az"]
        df = pd.read_csv(file_path, names=columns, skiprows=1)
        df = df.apply(pd.to_numeric, errors='coerce').dropna()

        # Compute velocity magnitude
        df["speed"] = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)

        # Trim trajectory
        start_idx = df[df["speed"] > 0.1].index.min()
        final_pos = df[["x", "y", "z"]].iloc[-1].values
        df["dist_to_final"] = np.sqrt(np.sum((df[["x", "y", "z"]] - final_pos)**2, axis=1))
        end_idx = df[df["dist_to_final"] > 0.01].index.max()

        df_trimmed = df.loc[start_idx:end_idx].copy()
        avg_velocity = df_trimmed["speed"].mean()
        avg_velocities.append(avg_velocity)

        print(f"[RESULT] Drone {drone_id}: Avg velocity = {avg_velocity:.2f} m/s")
        trajectories.append(df_trimmed)

    # Calculate mean velocity over all drones
    mean_velocity_all = np.mean(avg_velocities) if avg_velocities else 0.0

    # Align trajectories by timestamp and check safety violations + min distance
    if len(trajectories) == n_drones:
        # Find overlapping timestamp range
        start_time = max(traj["timestamp"].min() for traj in trajectories)
        end_time = min(traj["timestamp"].max() for traj in trajectories)

        if start_time >= end_time:
            print("[WARN] No overlapping time range between trajectories. Skipping safety check.")
            return mean_velocity_all, 0.0, violations  # min_distance=0 if no overlap

        # Create common time vector (e.g., 100 points)
        common_times = np.linspace(start_time, end_time, 100)

        # Interpolate positions for each drone at common_times
        interp_positions = []
        for traj in trajectories:
            interp_x = np.interp(common_times, traj["timestamp"], traj["x"])
            interp_y = np.interp(common_times, traj["timestamp"], traj["y"])
            interp_z = np.interp(common_times, traj["timestamp"], traj["z"])
            interp_positions.append(np.vstack([interp_x, interp_y, interp_z]).T)

        # Check distance between drones at each common timestamp
        for i, t in enumerate(common_times):
            for d1 in range(n_drones):
                for d2 in range(d1 + 1, n_drones):  # avoid duplicate pairs and self-comparison
                    pos1 = interp_positions[d1][i]
                    pos2 = interp_positions[d2][i]
                    dist = np.linalg.norm(pos1 - pos2)
                    if dist < safety_distance:
                        print(f"[ALERT] Safety violation at timestamp {t:.3f}s between drones {d1} and {d2}: Distance = {dist:.2f} m")
                        violations = True
                    if dist < min_distance:
                        min_distance = dist

    else:
        min_distance = 0.0  # No data for all drones

    return mean_velocity_all, min_distance, violations

# --- Loop through each configuration and iteration ---
min_dist_tot = 100.0;
for i, (loss, delay_sec) in enumerate(combinations):
    for j in range(n_iter):
        print(f"\n[INFO] Running simulation {i+1}/{len(combinations)} (iteration {j+1}/{n_iter}) "
              f"with packet_loss={loss}%, delay={delay_sec} ms")

        # Launch the ROS 2 launch file with args
        process = subprocess.Popen([
            "ros2", "launch", "multi_agent_planner", "multi_agent_planner_circle.launch.py",
            f"packet_loss_percentage:={loss}",
            f"communication_delay:={delay_sec}"
        ])

        try:
            time.sleep(simulation_time)
            print("[INFO] Killing launch process...")
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("[WARN] Forcing process to terminate...")
                process.kill()
            time.sleep(3)
            kill_ros_nodes()
            time.sleep(2)

            # Analyze the resulting trajectory data
            print("[INFO] Processing trajectory data...")
            mean_vel, min_dist, violation = process_trajectories(f"sim_{i}_iter_{j}")
            min_dist_tot = min(min_dist_tot, min_dist)

            # Store results
            mean_velocities_dict[(loss, delay_sec)].append(mean_vel)
            min_distances_dict[(loss, delay_sec)].append(min_dist)
            print(f"mean_vel {mean_vel:.3f}; min_distance = {min_dist:.3f}")
            print(f"min_distance_tot = {min_dist_tot:.3f}")

        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user. Stopping...")
            process.terminate()
            kill_ros_nodes()
            break

        time.sleep(30)

print("\n[INFO] All simulations completed.")

# --- Post-processing and plotting heatmaps ---

# Prepare matrices for heatmap plotting
vel_matrix = np.zeros((len(percentage_lost), len(communication_delay)))
dist_matrix = np.zeros_like(vel_matrix)

for i_loss, loss in enumerate(percentage_lost):
    for j_delay, delay in enumerate(communication_delay):
        vals_vel = mean_velocities_dict.get((loss, delay), [])
        vals_dist = min_distances_dict.get((loss, delay), [])
        if vals_vel:
            vel_matrix[i_loss, j_delay] = np.mean(vals_vel)
        else:
            vel_matrix[i_loss, j_delay] = 0.0
        if vals_dist:
            dist_matrix[i_loss, j_delay] = np.mean(vals_dist)
        else:
            dist_matrix[i_loss, j_delay] = 0.0

# Prepare data for CSV: one row per combination with columns: packet_loss, communication_delay, avg_velocity
csv_rows = []
for i_loss, loss in enumerate(percentage_lost):
    for j_delay, delay in enumerate(communication_delay):
        avg_vel = vel_matrix[i_loss, j_delay]
        csv_rows.append([loss, delay, avg_vel])

csv_file = "average_velocity_results.csv"
with open(csv_file, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["packet_loss", "communication_delay", "average_velocity"])
    writer.writerows(csv_rows)

print(f"[INFO] Average velocity data saved to {csv_file}")

# Plot heatmap for average velocity
# Read the CSV file
df = pd.read_csv("average_velocity_results.csv")

# Pivot the DataFrame for the heatmap
heatmap_data = df.pivot(index='communication_delay', columns='packet_loss', values='average_velocity')

# Set custom colormap for the heatmap (green for high, red for low)
cmap = sns.color_palette("RdYlGn", as_cmap=True)

# Create the heatmap
plt.figure(figsize=(12, 10)) # Adjust figure size for larger fonts
ax = sns.heatmap(
    heatmap_data,
    annot=True,
    fmt=".2f",
    cmap=cmap,
    linewidths=.5,
    linecolor='gray',
    annot_kws={"size": 20}, # Font size for the annotations (values inside cells)
    cbar_kws={"label": "Average Velocity"} # Set label for color bar, fontsize set separately
)

# Set labels and title with explicit fontsize
plt.xlabel("Packet loss (%)", fontsize=20)
plt.ylabel("Communication Delay (seconds)", fontsize=20)
plt.title("Average Velocity Heatmap", fontsize=24) # A bit larger for the main title

# Invert the y-axis to put (0.0, 0.0) at the bottom
ax.invert_yaxis()

# Adjust tick label font sizes for x and y axes
ax.tick_params(axis='x', labelsize=20)
ax.tick_params(axis='y', labelsize=20)

# Adjust font size for the color bar label and ticks
cbar = ax.collections[0].colorbar
cbar.ax.tick_params(labelsize=20) # Font size for color bar tick labels
cbar.set_label("Average Velocity", fontsize=20) # Font size for color bar label

# Show plot
plt.show()
