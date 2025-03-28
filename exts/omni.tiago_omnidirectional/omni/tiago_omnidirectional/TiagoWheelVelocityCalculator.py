import numpy as np


def calculate_tiago_wheel_velocities(vx, vy, omega,
                                     wheel_radius=0.0762,
                                     wheel_positions=[0.244, 0.22317]):
    """
    Compute the wheel angular velocities for a four-wheeled mecanum robot considering mecanum angles.
    """


    Lx, Ly = wheel_positions

    
    # Define the inverse kinematic matrix
    kinematic_matrix = np.array([
        [1, 1, -(Lx + Ly)],
        [1, -1,  (Lx + Ly)],
        [1, -1, -(Lx + Ly)],
        [1, 1,  (Lx + Ly)]
    ])

    # Define the velocity vector (robot frame)
    velocity_vector = np.array([vx, vy, omega])

    # Compute wheel velocities (multiply the kinematic matrix with the velocity vector)
    wheel_velocities = np.dot(kinematic_matrix, velocity_vector)

    # Convert from linear velocity to angular velocity by dividing by the wheel radius
    angular_wheel_velocities = wheel_velocities / wheel_radius

    return angular_wheel_velocities


