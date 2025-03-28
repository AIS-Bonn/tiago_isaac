import pandas as pd
import numpy as np
import os
import re
from scipy.interpolate import interp1d
from collections import defaultdict
import matplotlib.pyplot as plt

# Function to get all traj##.csv files from a directory
def get_traj_files(directory):
    return [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]


def extract_params(filename):
    # Pattern for x=Xy=Yz=Zsecs=SECS(_reverse)?
    pattern_xyz = r"x=(-?\d+\.\d+)y=(-?\d+\.\d+)z=(-?\d+\.\d+)secs=(\d+\.\d+)(_reverse)?"
    # Pattern for square_trajectory_speed=SPEEDsecs=SECS
    pattern_square = r"square_trajectory_speed=(\d+\.\d+)secs=(\d+\.\d+)"

    match_xyz = re.search(pattern_xyz, filename)
    if match_xyz:
        x, y, z, secs, reverse = match_xyz.groups()
        reverse = bool(reverse)  # True if "_reverse" is present, otherwise False
        return (float(x), float(y), float(z), float(secs), reverse)
    
    match_square = re.search(pattern_square, filename)
    if match_square:
        speed, secs = match_square.groups()
        return ("square_trajectory", float(speed), float(secs))
    
    return None


# Find movement dimensions and interpolate
def find_movement_dimension(points):
    ranges = points.ptp(axis=0)
    min_movement_dim = np.argmin(ranges)
    return [dim for dim in range(3) if dim != min_movement_dim]
    
def preprocess_trajectory_data(data, num_samples, ensure_x_direction=False, subsample_step=99999999, mirror=False):
    """
    Preprocess the trajectory data, with optional alignment of initial movement to x direction.
    
    Args:
    - data (DataFrame): The trajectory data with 'pos_x', 'pos_y', 'pos_z' columns.
    - num_samples (int): Number of samples for interpolation.
    - ensure_x_direction (bool): If True, rotates initial movement to align with the x direction.
    - subsample_step (int): Step size for subsampling to determine initial movement direction.
    
    Returns:
    - interpolated_points (ndarray): The processed 2D interpolated trajectory data (Nx2).
    """
    # Align the starting point to (0, 0)
    data['pos_x'] -= data['pos_x'].iloc[0]
    data['pos_y'] -= data['pos_y'].iloc[0]
    data['pos_z'] -= data['pos_z'].iloc[0]

    # Determine the dimensions of greatest movement for 2D projection
    points = np.vstack((data['pos_x'], data['pos_y'], data['pos_z'])).T
    plot_dims = find_movement_dimension(points)

    # Interpolation to a fixed number of samples
    time_norm = np.linspace(0, data['timestamp'].max(), num_samples)
    interp_func_1 = interp1d(data['timestamp'], points[:, plot_dims[0]], kind='linear', bounds_error=False, fill_value="extrapolate")
    interp_func_2 = interp1d(data['timestamp'], points[:, plot_dims[1]], kind='linear', bounds_error=False, fill_value="extrapolate")
    interpolated_points = np.vstack((interp_func_1(time_norm), interp_func_2(time_norm))).T

    # Subsample to determine initial movement direction
    if ensure_x_direction and len(interpolated_points) > subsample_step:
        initial_vector = interpolated_points[subsample_step] - interpolated_points[0]
        angle = np.arctan2(initial_vector[1], initial_vector[0])

        # Rotate to align with the x-axis
        rotation_matrix = np.array([[np.cos(-angle), -np.sin(-angle)], 
                                    [np.sin(-angle),  np.cos(-angle)]])
        
        # Apply rotation
        interpolated_points = interpolated_points @ rotation_matrix.T

    if mirror:
        # Apply mirroring along the x-axis
        mirror_matrix = np.array([[1, 0], [0, -1]])
        interpolated_points = interpolated_points @ mirror_matrix.T

    return interpolated_points

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
        (item[0][2], item[0][1]) if item[0][0] == "square_trajectory" else (item[0][0], item[0][1], item[0][2], item[0][3])
    ))

    num_samples = 1000  # Number of samples for interpolation
    subsample_step = 100  # The id of point p such that we align the vector p with the x-axis

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
        real_start_to_end_differences = []

        # Process simulation trajectory files
        for sim_file in sim_group:
            file_path_simulation = os.path.join(simulation_dir, sim_file)
            df_simulation = pd.read_csv(file_path_simulation)
            df_simulation['timestamp'] -= df_simulation['timestamp'].iloc[0]

            # Preprocess simulation data, ensuring movement starts in x direction
            processed_sim = preprocess_trajectory_data(df_simulation, ensure_x_direction=True, num_samples=num_samples, mirror=False, subsample_step=subsample_step)
            all_sim_interpolated.append(processed_sim)

        # Process light simulation trajectory files if provided
        if simulation_light_dir is not None:
            for sim_light_file in sim_light_group:
                file_path_simulation_light = os.path.join(simulation_light_dir, sim_light_file)
                df_simulation_light = pd.read_csv(file_path_simulation_light)
                df_simulation_light['timestamp'] -= df_simulation_light['timestamp'].iloc[0]

                # Preprocess light simulation data
                processed_sim_light = preprocess_trajectory_data(df_simulation_light, ensure_x_direction=True, num_samples=num_samples, mirror=False, subsample_step=subsample_step)
                all_sim_light_interpolated.append(processed_sim_light)

        # Process real trajectory files
        for real_file in real_group:
            file_path_real = os.path.join(real_dir, real_file)
            df_real = pd.read_csv(file_path_real)
            df_real['timestamp'] -= df_real['timestamp'].iloc[0]

            # Preprocess real data
            processed_real = preprocess_trajectory_data(df_real, ensure_x_direction=True, num_samples=num_samples, mirror=True, subsample_step=subsample_step)
            all_real_interpolated.append(processed_real)

            start_to_end_diff = np.linalg.norm(processed_real[-1] - processed_real[0])
            real_start_to_end_differences.append(start_to_end_diff)

        # Generate ground truth trajectory for xyz,secs trajectories using Euler's method, including rotation
        if params[0] != "square_trajectory":
            x, y, z, secs, reverse = params
            num_euler_steps = 1000  # Number of time steps for Euler's method
            dt = secs / num_euler_steps  # Small time step

            # Compute the initial forward speed
            forward_speed = np.sqrt(x**2 + y**2)  # Magnitude of movement in the x-y plane
            angular_velocity = z  # Rotation in radians per second

            # Initialize ground truth trajectory
            ground_truth_points = np.zeros((num_euler_steps, 2))
            position = np.array([0.0, 0.0])  # Start at the origin
            angle = np.arctan2(y, x)  # Initial angle based on x and y velocities

            for i in range(num_euler_steps):
                # Update angle based on angular velocity
                angle += angular_velocity * dt

                # Calculate new direction based on the updated angle
                direction = np.array([np.cos(angle), np.sin(angle)])

                # Update position using the forward speed in the new direction
                position += direction * forward_speed * dt
                ground_truth_points[i] = position

            # Rotate the ground truth trajectory to align the first step with the positive x-axis
            initial_direction_angle = np.arctan2(ground_truth_points[1, 1] - ground_truth_points[0, 1],
                                                ground_truth_points[1, 0] - ground_truth_points[0, 0])
            rotation_matrix = np.array([[np.cos(-initial_direction_angle), -np.sin(-initial_direction_angle)],
                                        [np.sin(-initial_direction_angle),  np.cos(-initial_direction_angle)]])

            # Apply rotation to the entire trajectory
            ground_truth_points = ground_truth_points @ rotation_matrix.T

        # Compute the averages for the aligned real trajectories and simulation trajectories
        avg_real_points = np.mean(all_real_interpolated, axis=0)
        avg_sim_points = np.mean(all_sim_interpolated, axis=0)
        if simulation_light_dir is not None:
            avg_sim_light_points = np.mean(all_sim_light_interpolated, axis=0)

        # Calculate statistics
        total_difference_real = np.linalg.norm(avg_real_points[-1] - avg_real_points[0])
        total_difference_sim = np.linalg.norm(avg_sim_points[-1] - avg_sim_points[0])

        absolute_error = np.abs(total_difference_real - total_difference_sim)
        relative_error_percentage = (absolute_error / total_difference_real) * 100

        # For light simulation if provided
        if simulation_light_dir is not None:
            total_difference_sim_light = np.linalg.norm(avg_sim_light_points[-1] - avg_sim_light_points[0])
            absolute_error_light = np.abs(total_difference_real - total_difference_sim_light)
            relative_error_percentage_light = (absolute_error_light / total_difference_real) * 100
        else:
            total_difference_sim_light = None
            absolute_error_light = None
            relative_error_percentage_light = None

        differences_from_sim = [abs(diff - total_difference_sim) for diff in real_start_to_end_differences]
        std_dev = np.std(differences_from_sim)

        # Store statistics in all_stats
        stats_entry = {
            'params': f'x={params[0]} y={params[1]} z={params[2]} secs={params[3]}' if params[0] != "square_trajectory" else f'square_trajectory_speed={params[1]}secs={params[2]}',
            'real_diff': total_difference_real,
            'sim_diff': total_difference_sim,
            'abs_error': absolute_error * 100,  # Convert to cm
            'rel_error': relative_error_percentage
        }

        if simulation_light_dir is not None:
            stats_entry.update({
                'sim_light_diff': total_difference_sim_light,
                'abs_error_light': absolute_error_light * 100,  # Convert to cm
                'rel_error_light': relative_error_percentage_light
            })

        all_stats.append(stats_entry)

        # Plotting
        plt.figure(figsize=(8, 6))

        # Plot Simulation and Real (Average) Trajectories
        plt.subplot(1, 1, 1)
        plt.plot(avg_real_points[:, 0], avg_real_points[:, 1], 'b-', label='Real')
        plt.plot(avg_sim_points[:, 0], avg_sim_points[:, 1], 'r-', label='Simulated')
        if simulation_light_dir is not None:
            plt.plot(avg_sim_light_points[:, 0], avg_sim_light_points[:, 1], 'm-', label='Simulated (Light)')
        # if params[0] != "square_trajectory":
        #     plt.plot(ground_truth_points[:, 0], ground_truth_points[:, 1], 'g-', label='Ideal (w/o acceleration)')
        # plt.title('Averaged Trajectories (2D)')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend(loc='upper right', bbox_to_anchor=(1.5, 1), fontsize=14)
        plt.axis('equal')
        plt.grid(True)
        
        
        if not os.path.exists("saved_plots"):
            os.makedirs("saved_plots")
        
        plt.savefig(f"saved_plots/{params}.pgf", format="pgf", bbox_inches="tight")  # Save as PGF (for LaTeX)
        plt.savefig(f"saved_plots/{params}.png", bbox_inches="tight")

        # plt.suptitle(f'Trajectories for params {params}')
        plt.show()

    # Create summary plots for all statistics at the end
    param_labels = [stat['params'] for stat in all_stats]
    real_diffs = [stat['real_diff'] for stat in all_stats]
    sim_diffs = [stat['sim_diff'] for stat in all_stats]
    abs_errors = [stat['abs_error'] for stat in all_stats]
    rel_errors = [stat['rel_error'] for stat in all_stats]
    
    

    if simulation_light_dir is not None:
        sim_light_diffs = [stat['sim_light_diff'] for stat in all_stats]
        abs_errors_light = [stat['abs_error_light'] for stat in all_stats]
        rel_errors_light = [stat['rel_error_light'] for stat in all_stats]

    # eliminate nan values by if there is a nan value, replace it with the mean of the list excluding the nan value
    for i in range(len(real_diffs)):
        if np.isnan(real_diffs[i]):
            real_diffs[i] = np.nanmean(real_diffs)
        if np.isnan(sim_diffs[i]):
            sim_diffs[i] = np.nanmean(sim_diffs)
        if simulation_light_dir is not None:
            if np.isnan(sim_light_diffs[i]):
                sim_light_diffs[i] = np.nanmean(sim_light_diffs)
        if np.isnan(abs_errors[i]):
            abs_errors[i] = np.nanmean(abs_errors)
        if simulation_light_dir is not None:
            if np.isnan(abs_errors_light[i]):
                abs_errors_light[i] = np.nanmean(abs_errors_light)
        if np.isnan(rel_errors[i]):
            rel_errors[i] = np.nanmean(rel_errors)
        if simulation_light_dir is not None:
            if np.isnan(rel_errors_light[i]):
                rel_errors_light[i] = np.nanmean(rel_errors_light)
        
    
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Plot Total Distances
    axs[0].plot(param_labels, real_diffs, 'bo-', label="Real Difference")
    axs[0].plot(param_labels, sim_diffs, 'ro-', label="Sim Difference")
    if simulation_light_dir is not None:
        axs[0].plot(param_labels, sim_light_diffs, 'mo-', label="Sim Light Difference")
    axs[0].set_ylabel('Total Distance (m)')
    axs[0].legend()
    axs[0].grid(True)

    # Plot Absolute Error
    axs[1].plot(param_labels, abs_errors, 'go-', label="Absolute Error (Sim) (cm)")
    if simulation_light_dir is not None:
        axs[1].plot(param_labels, abs_errors_light, 'co-', label="Absolute Error (Sim Light) (cm)")
    axs[1].set_ylabel('Absolute Error (cm)')
    axs[1].legend()
    axs[1].grid(True)
    

    # Plot Relative Error
    axs[2].plot(param_labels, rel_errors, 'go-', label="Relative Error (Sim) (%)")
    if simulation_light_dir is not None:
        axs[2].plot(param_labels, rel_errors_light, 'co-', label="Relative Error (Sim Light) (%)")
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
