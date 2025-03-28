import pandas as pd
import numpy as np
import os
import re
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from collections import defaultdict
import matplotlib.pyplot as plt

# Function to get all traj##.csv files from a directory
def get_traj_files(directory):
    return [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]


def extract_params(filename):
    # Pattern for x=Xy=Yz=Zsecs=SECS(_reverse)?
    pattern_xyz = r"x=(-?\d+\.\d+)y=(-?\d+\.\d+)z=(-?\d+\.\d+)secs=(\d+\.\d+)(_reverse)?"

    match_xyz = re.search(pattern_xyz, filename)
    if match_xyz:
        x, y, z, secs, reverse = match_xyz.groups()
        reverse = bool(reverse)  # True if "_reverse" is present, otherwise False
        return (float(x), float(y), float(z), float(secs), reverse)
    
    return None


def preprocess_rotation_data(data, num_samples, is_real_data=False):
    """
    Preprocess the rotation data, ensuring alignment and interpolation.

    Args:
    - data (DataFrame): The trajectory data with 'rot_00' to 'rot_22' columns (rotation matrix).
    - num_samples (int): Number of samples for interpolation.
    - is_real_data (bool): If True, applies a rotation to align real data axes with simulation axes.

    Returns:
    - interpolated_angles (ndarray): The processed interpolated rotation angles (Nx1).
    """
    # Extract rotation matrix components and build 3x3 rotation matrices
    rot_matrices = data[['rot_00', 'rot_01', 'rot_02',
                         'rot_10', 'rot_11', 'rot_12',
                         'rot_20', 'rot_21', 'rot_22']].values
    rot_matrices = rot_matrices.reshape(-1, 3, 3)

    # Create Rotation objects from the rotation matrices
    rotations = R.from_matrix(rot_matrices)

    # If real data, rotate around X-axis by -90 degrees to align Y-axis rotation to Z-axis
    if is_real_data:
        # Create rotation object for -90 degrees around X-axis
        rotation_fix = R.from_euler('x', -90, degrees=True)
        # Compose the rotations by multiplying rotation_fix with rotations
        rotations = rotation_fix * rotations

    # Extract yaw angles (rotation around Z-axis) from rotation matrices
    rotation_angles = rotations.as_euler('zyx', degrees=False)[:, 0]  # Extract Z-axis rotation (yaw)

    # Unwrap angles to avoid discontinuities
    rotation_angles = np.unwrap(rotation_angles)

    # Align starting angle to zero
    rotation_angles -= rotation_angles[0]

    # Interpolation to a fixed number of samples
    time_norm = np.linspace(0, data['timestamp'].max(), num_samples)
    interp_func = interp1d(data['timestamp'], rotation_angles, kind='linear',
                           bounds_error=False, fill_value="extrapolate")
    interpolated_angles = interp_func(time_norm)

    if is_real_data:
        interpolated_angles *= -1

    return interpolated_angles



def plot_trajectory_stats(real_dir, simulation_dir, simulation_light_dir=None):
    real_groups = defaultdict(list)
    simulation_groups = defaultdict(list)
    simulation_light_groups = defaultdict(list)
    all_stats = []  # To store statistics for each parameter set

    # Get the list of files from all directories
    real_files = get_traj_files(real_dir)
    simulation_files = get_traj_files(simulation_dir)
    if simulation_light_dir is not None:
        simulation_light_files = get_traj_files(simulation_light_dir)

    # Group real files
    for real_file in real_files:
        params = extract_params(real_file)
        if params:
            real_groups[params].append(real_file)

    # Group simulation files
    for sim_file in simulation_files:
        params = extract_params(sim_file)
        if params:
            simulation_groups[params].append(sim_file)

    # Group light simulation files if provided
    if simulation_light_dir is not None:
        for sim_light_file in simulation_light_files:
            params = extract_params(sim_light_file)
            if params:
                simulation_light_groups[params].append(sim_light_file)

    # Sort the real groups by parameters
    sorted_real_groups = sorted(real_groups.items(), key=lambda item: (
        (item[0][0], item[0][1], item[0][2], item[0][3])
    ))

    num_samples = 1000  # Number of samples for interpolation

    # Iterate over the sorted real groups and check for corresponding simulation groups
    for params, real_group in sorted_real_groups:
        sim_group = simulation_groups.get(params)
        sim_light_group = simulation_light_groups.get(params) if simulation_light_dir is not None else None

        if not sim_group:
            print(f"No corresponding simulation files found for real files with params {params}")
            continue
        if simulation_light_dir is not None and not sim_light_group:
            print(f"No corresponding light simulation files found for real files with params {params}")
            continue

        all_real_interpolated = []
        all_sim_interpolated = []
        all_sim_light_interpolated = []
        real_total_rotations = []

        # Process simulation trajectory files
        for sim_file in sim_group:
            file_path_simulation = os.path.join(simulation_dir, sim_file)
            df_simulation = pd.read_csv(file_path_simulation)
            df_simulation['timestamp'] -= df_simulation['timestamp'].iloc[0]

            # Preprocess simulation data
            processed_sim = preprocess_rotation_data(df_simulation, is_real_data=False, num_samples=num_samples)
            all_sim_interpolated.append(processed_sim)

        # Process light simulation trajectory files if provided
        if simulation_light_dir is not None:
            for sim_light_file in sim_light_group:
                file_path_simulation_light = os.path.join(simulation_light_dir, sim_light_file)
                df_simulation_light = pd.read_csv(file_path_simulation_light)
                df_simulation_light['timestamp'] -= df_simulation_light['timestamp'].iloc[0]

                # Preprocess light simulation data
                processed_sim_light = preprocess_rotation_data(df_simulation_light, is_real_data=False, num_samples=num_samples)
                all_sim_light_interpolated.append(processed_sim_light)

        # Process real trajectory files
        for real_file in real_group:
            file_path_real = os.path.join(real_dir, real_file)
            df_real = pd.read_csv(file_path_real)
            df_real['timestamp'] -= df_real['timestamp'].iloc[0]
            df_real['timestamp'] /= 1000  # Assuming real data timestamp is in milliseconds

            # Preprocess real data
            processed_real = preprocess_rotation_data(df_real, is_real_data=True, num_samples=num_samples)
            all_real_interpolated.append(processed_real)

            total_rotation = processed_real[-1] - processed_real[0]
            real_total_rotations.append(total_rotation)

        # Compute the averages for the aligned real rotations and simulation rotations
        avg_real_angles = np.mean(all_real_interpolated, axis=0)
        avg_sim_angles = np.mean(all_sim_interpolated, axis=0)
        if simulation_light_dir is not None:
            avg_sim_light_angles = np.mean(all_sim_light_interpolated, axis=0)

        # Calculate total rotation for average trajectories
        total_rotation_real = avg_real_angles[-1] - avg_real_angles[0]
        total_rotation_sim = avg_sim_angles[-1] - avg_sim_angles[0]

        # Calculate absolute and relative errors for normal simulation
        absolute_error = np.abs(total_rotation_real - total_rotation_sim)
        relative_error_percentage = (absolute_error / np.abs(total_rotation_real)) * 100 if total_rotation_real != 0 else 0

        # For light simulation if provided
        if simulation_light_dir is not None:
            total_rotation_sim_light = avg_sim_light_angles[-1] - avg_sim_light_angles[0]
            absolute_error_light = np.abs(total_rotation_real - total_rotation_sim_light)
            relative_error_percentage_light = (absolute_error_light / np.abs(total_rotation_real)) * 100 if total_rotation_real != 0 else 0
        else:
            total_rotation_sim_light = None
            absolute_error_light = None
            relative_error_percentage_light = None

        differences_from_sim = [np.abs(rot - total_rotation_sim) for rot in real_total_rotations]
        std_dev = np.std(differences_from_sim)

        # Convert radians to degrees for better interpretability
        total_rotation_real_deg = np.degrees(total_rotation_real)
        total_rotation_sim_deg = np.degrees(total_rotation_sim)
        absolute_error_deg = np.degrees(absolute_error)
        std_dev_deg = np.degrees(std_dev)

        if simulation_light_dir is not None:
            total_rotation_sim_light_deg = np.degrees(total_rotation_sim_light)
            absolute_error_light_deg = np.degrees(absolute_error_light)
        else:
            total_rotation_sim_light_deg = None
            absolute_error_light_deg = None

        # Store statistics in all_stats
        stats_entry = {
            'params': f'x={params[0]} y={params[1]} z={params[2]} secs={params[3]}',
            'real_diff_deg': total_rotation_real_deg,
            'sim_diff_deg': total_rotation_sim_deg,
            'abs_error_deg': absolute_error_deg,  # In degrees
            'rel_error': relative_error_percentage
        }

        if simulation_light_dir is not None:
            stats_entry.update({
                'sim_light_diff_deg': total_rotation_sim_light_deg,
                'abs_error_light_deg': absolute_error_light_deg,
                'rel_error_light': relative_error_percentage_light
            })

        all_stats.append(stats_entry)

        print()
        print(f"{params}:")
        print(f"Real Data (Aligned) total rotation: {total_rotation_real_deg:.4f} degrees")
        print(f"Simulation Data total rotation: {total_rotation_sim_deg:.4f} degrees")
        print(f"Absolute error between real and simulated rotation: {absolute_error_deg:.4f} degrees")
        print(f"Relative error: {relative_error_percentage:.2f}%")

        if simulation_light_dir is not None:
            print(f"Simulation Light Data total rotation: {total_rotation_sim_light_deg:.4f} degrees")
            print(f"Absolute error between real and simulated light rotation: {absolute_error_light_deg:.4f} degrees")
            print(f"Relative error (Light): {relative_error_percentage_light:.2f}%")

        if len(differences_from_sim) > 1:
            print(f"Standard deviation of differences from simulation: {std_dev_deg:.4f} degrees")
        print()

        # Plotting
        plt.figure(figsize=(12, 6))
        time_real = np.linspace(0, df_real['timestamp'].max(), num_samples)
        time_sim = np.linspace(0, df_simulation['timestamp'].max(), num_samples)

        plt.plot(time_real, np.degrees(avg_real_angles), 'b-', label='Real (Avg)')
        plt.plot(time_sim, np.degrees(avg_sim_angles), 'r-', label='Simulated (Avg)')
        if simulation_light_dir is not None:
            time_sim_light = np.linspace(0, df_simulation_light['timestamp'].max(), num_samples)
            plt.plot(time_sim_light, np.degrees(avg_sim_light_angles), 'm-', label='Simulated Light (Avg)')

        plt.title('Averaged Rotations (degrees)')
        plt.xlabel('Time (s)')
        plt.ylabel('Rotation (degrees)')
        plt.legend()
        plt.grid(True)
        plt.suptitle(f'Rotations for params {params}')
        plt.show()

    # Create summary plots for all statistics at the end
    param_labels = [stat['params'] for stat in all_stats]
    real_diffs_deg = [stat['real_diff_deg'] for stat in all_stats]
    sim_diffs_deg = [stat['sim_diff_deg'] for stat in all_stats]
    abs_errors_deg = [stat['abs_error_deg'] for stat in all_stats]
    rel_errors = [stat['rel_error'] for stat in all_stats]

    if simulation_light_dir is not None:
        sim_light_diffs_deg = [stat['sim_light_diff_deg'] for stat in all_stats]
        abs_errors_light_deg = [stat['abs_error_light_deg'] for stat in all_stats]
        rel_errors_light = [stat['rel_error_light'] for stat in all_stats]
        
    # eliminate nan values by if there is a nan value, replace it with the mean of the list excluding the nan value
    for i in range(len(real_diffs_deg)):
        if np.isnan(real_diffs_deg[i]):
            real_diffs_deg[i] = np.nanmean(real_diffs_deg)
        if np.isnan(sim_diffs_deg[i]):
            sim_diffs_deg[i] = np.nanmean(sim_diffs_deg)
        if np.isnan(abs_errors_deg[i]):
            abs_errors_deg[i] = np.nanmean(abs_errors_deg)
        if np.isnan(rel_errors[i]):
            rel_errors[i] = np.nanmean(rel_errors)
        if simulation_light_dir is not None:
            if np.isnan(sim_light_diffs_deg[i]):
                sim_light_diffs_deg[i] = np.nanmean(sim_light_diffs_deg)
            if np.isnan(abs_errors_light_deg[i]):
                abs_errors_light_deg[i] = np.nanmean(abs_errors_light_deg)
            if np.isnan(rel_errors_light[i]):
                rel_errors_light[i] = np.nanmean(rel_errors_light)

    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Plot Real and Simulated Total Rotations
    axs[0].plot(param_labels, real_diffs_deg, 'bo-', label="Real Rotation")
    axs[0].plot(param_labels, sim_diffs_deg, 'ro-', label="Sim Rotation")
    if simulation_light_dir is not None:
        axs[0].plot(param_labels, sim_light_diffs_deg, 'mo-', label="Sim Light Rotation")
    axs[0].set_ylabel('Total Rotation (degrees)')
    axs[0].legend()
    axs[0].grid(True)

    # Plot Absolute Error
    axs[1].plot(param_labels, abs_errors_deg, 'go-', label="Absolute Error (Sim)")
    if simulation_light_dir is not None:
        axs[1].plot(param_labels, abs_errors_light_deg, 'co-', label="Absolute Error (Sim Light)")
    axs[1].set_ylabel('Absolute Error (degrees)')
    axs[1].legend()
    axs[1].grid(True)

    # Plot Relative Error
    axs[2].plot(param_labels, rel_errors, 'mo-', label="Relative Error (Sim)")
    if simulation_light_dir is not None:
        axs[2].plot(param_labels, rel_errors_light, 'yo-', label="Relative Error (Sim Light)")
    axs[2].set_ylabel('Relative Error (%)')
    axs[2].legend()
    axs[2].grid(True)

    relative_error_stddev = np.std(rel_errors)
    if simulation_light_dir is not None:
        relative_error_light_stddev = np.std(rel_errors_light)
        
    print("Mean relative errors:", np.mean(rel_errors))
    if simulation_light_dir is not None:
        print("Mean relative errors light:", np.mean(rel_errors_light))
    
    print("Relative error stddev:", relative_error_stddev)
    if simulation_light_dir is not None:
        print("Relative error light stddev:", relative_error_light_stddev)
        
    # Format x-axis
    plt.xticks(rotation=-90)
    plt.xlabel('Parameter Groups')
    plt.tight_layout()
    plt.show()
