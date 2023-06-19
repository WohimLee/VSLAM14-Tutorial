
import numpy as np



# Baseline distance between the two camera viewpoints
baseline = 0.1  # Specify the baseline distance in meters

# Disparity value (pixel difference) between the corresponding points
disparity = 10  # Specify the disparity value in pixels

# Focal length of the cameras (assumed to be the same)
focal_length = 500  # Specify the focal length in pixels

# Calculate the depth (z-coordinate) of the 3D point
z = (baseline * focal_length) / disparity

print("Depth (z-coordinate):", z)






if __name__ == '__main__':
    baseline