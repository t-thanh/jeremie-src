#!/usr/bin/env python
import numpy as np
import sys

def modify_last_value_in_text_file(file_path):
    """
    Reads a text file line by line, modifies the last value of each line to 5, 
    and writes the updated values back to the file.

    Args:
        file_path (str): The path to the .txt file.
    """

    data = []  # List to store the lines of data

    # Read the file line by line
    with open(file_path, 'r') as file:
        for line in file:
            values = [float(x.strip()) for x in line.split(' ')]  # Convert values to integers
            data.append(values)

    # Create a NumPy array from the data
    array = np.array(data)

    # Modify the last value of each row
    speed=float(sys.argv[2])
    array[:1, -1] = speed/2
    array[1:, -1] = speed

    # Write the updated values back to the file
    with open(file_path, 'w') as file:
        for row in array:
            line = ' '.join(map(str, row))  # Convert back to comma-separated string
            file.write(line + '\n')

# Example usage
shape=sys.argv[1]

file_path = "/home/jeremie/Master-Thesis/Quadrotor/ros1_ws/src/uav_system/simulation/ros_packages/mrs_gazebo_common_resources/models/Target_2/trajectories/{0}_traj.txt".format(shape)  # Replace with your file path
modify_last_value_in_text_file(file_path)

