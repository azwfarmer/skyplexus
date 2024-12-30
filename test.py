import numpy as np

# Define the Tensor and Truth matrices
tensor = np.array([
    [0.2248, 0.9024, -0.3676, 1.0000],
    [-0.3502, 0.4269, 0.8337, 2.0000],
    [0.9093, -0.0587, 0.4120, 3.0000]
])

truth = np.array([
    [0.4120, -0.8337, -0.3676, 1.0000],
    [-0.0587, -0.4269, 0.9024, 2.0000],
    [-0.9093, -0.3502, -0.2248, 3.0000]
])

# Extract rotational components (first 3 columns)
rot_tensor = tensor[:, :3]
rot_truth = truth[:, :3]

# Solve for the transformation matrix (rotation or reflection)
# M is the transformation matrix such that: rot_truth = rot_tensor @ M
M = np.linalg.inv(rot_tensor) @ rot_truth

# Verify the transformation
reconstructed_truth = rot_tensor @ M

# Print results
print("Transformation matrix (M):")
print(M)
print("\nReconstructed Truth (using M):")
print(reconstructed_truth)

# Check if the reconstructed matrix matches the original truth matrix
if np.allclose(reconstructed_truth, rot_truth):
    print("\nThe transformation matrix is valid!")
else:
    print("\nThe transformation matrix does not perfectly align!")
