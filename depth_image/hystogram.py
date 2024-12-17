import numpy as np
import matplotlib.pyplot as plt
import os

execution_path = os.path.dirname(os.path.abspath(__file__))



# file_path = os.path.join(execution_path, 'data', f'deprojected_points2.txt')
file_path = os.path.join(execution_path, 'data', f'combinated_deprojected_points.txt')
output_path = os.path.join(execution_path, 'data', f'histogram.png')


matrix = np.loadtxt(file_path, delimiter=',', skiprows=0)
# # Flatten the matrix to get the values
values = matrix.flatten()

# # Create a histogram of the values
# Filter out the zero values
non_zero_values = values[values != 0]

plt.hist(non_zero_values, bins=50, edgecolor='black')
plt.title('Histogram of Non-Zero Matrix Values')
plt.xlabel('Value (mm)')
plt.ylabel('Frequency')
plt.savefig(output_path)