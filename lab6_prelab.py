import numpy as np

def point_registration(A, B):
    # Compute centroids
    centroid_A = np.mean(A, axis=1, keepdims=True)
    centroid_B = np.mean(B, axis=1, keepdims=True)
    A_prime = A - centroid_A
    B_prime = B - centroid_B
    
    # Compute the correlation matrix
    H = A_prime @ B_prime.T
    
    # Perform SVD
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # Reflection case
    print("Determiniate of R:")
    print(np.linalg.det(R))
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Translation Vector
    t = centroid_B - R @ centroid_A

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    
    return T

# def calculate_rmse(A_t, B):
#     return np.sqrt(np.mean((A_t - B) ** 2))

# A = np.array([[681.2, 526.9, 914.8],
#               [542.3, 381.0, 876.5],
#               [701.2, 466.3, 951.4],
#               [598.4, 556.8, 876.9],
#               [654.3, 489.0, 910.2]]).T

# B = np.array([[110.1, 856.3, 917.8],
#               [115.1, 654.9, 879.5],
#               [167.1, 827.5, 954.4],
#               [ 30.4, 818.8, 879.9],
#               [117.9, 810.4, 913.2]]).T

# T = point_registration(A, B)

# # Transform A to robot coordinates
# A_homogeneous = np.vstack((A, np.ones((1, A.shape[1]))))
# transformed_A = T @ A_homogeneous
# transformed_A = transformed_A[:3, :]

# rmse = calculate_rmse(transformed_A, B)
# print("Transformation Matrix T:\n", T)
# print("RMSE:", rmse)