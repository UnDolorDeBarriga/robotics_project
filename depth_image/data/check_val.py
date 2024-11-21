import np
import numpy as np

# Load the data from the text file
data = np.loadtxt('/reference_points_image0.txt')

# Print the maximum value of every column
max_values = np.max(data, axis=0)
print(max_values)