import numpy as np

def get_dh_row_mat(arr):
    return np.array([[np.cos(arr[0]),  -np.sin(arr[0])*np.cos(arr[3]),    np.sin(arr[0])*np.sin(arr[3]),         arr[2]*np.cos(arr[0])],
                        [np.sin(arr[0]),   np.cos(arr[0])*np.cos(arr[3]),   -np.cos(arr[0])*np.sin(arr[3]),         arr[2]*np.sin(arr[0])],
                        [0,                np.sin(arr[3]),                   np.cos(arr[3]),                        arr[1]               ],
                        [0,                0,                                0,                                     1                    ]])

def get_int_mat(j):
    return np.array([get_dh_row_mat([j[0],                               77,   0, -np.pi/2]),
                     get_dh_row_mat([j[1]-(np.pi/2 -np.arcsin(24/130)),   0, 130,        0]),
                     get_dh_row_mat([j[2]+(np.pi/2 -np.arcsin(24/130)),   0, 124,        0]),
                     get_dh_row_mat([j[3],                                0, 126,        0]),])

def get_acc_mat(j):
    a = get_int_mat(j)
    return np.array([a[0],
                    a[0] @ a[1],
                    a[0] @ a[1] @ a[2],
                    a[0] @ a[1] @ a[2] @ a[3]])

def get_jacobian(j):
    acc_mats = get_acc_mat(j)  

    o1 = acc_mats[0][:3, 3]  # Position of frame 1
    o2 = acc_mats[1][:3, 3]  # Position of frame 2
    o3 = acc_mats[2][:3, 3]  # Position of frame 3
    o4 = acc_mats[3][:3, 3]  # Position of frame 3


    z0 = [0,0,1]
    z1 = acc_mats[0][:3, 2]  
    z2 = acc_mats[1][:3, 2]  
    z3 = acc_mats[2][:3, 2]  

    Jv0 = np.cross(z0, o4)
    Jv1 = np.cross(z1, o4 - o1)
    Jv2 = np.cross(z2, o4 - o2)
    Jv3 = np.cross(z3, o4 - o3)

    # Combine the linear (Jv) and angular (Jw) components
    Jv = np.column_stack(np.radians((Jv0, Jv1, Jv2, Jv3)))
    Jw = np.column_stack((z0, z1, z2, z3))

    return np.vstack((Jv, Jw))

J = get_jacobian(np.radians([0, -10.62, -79.38, 0]))
print("Case: [0, -10.62, -79.38, 0]:")
print(J)
print(np.linalg.det(J[:3, :3]))
print()
J1 = get_jacobian([0, 0, 0, 0])
print("Case: [0, 0, 0, 0]:")
print(J1)
print(np.linalg.det(J1[:3, :3]))