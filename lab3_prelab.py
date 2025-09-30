import numpy as np

def get_ik(p):
    l1 = 77
    l2 = 130
    l3 = 124
    l4 = 126
    l21 = 128
    l22 = 24
    r = np.sqrt(p[0]**2 + p[1]**2)
    rw = r - l4*np.cos(np.radians(p[3]))
    zw = p[2] - l1 - l4*np.sin(np.radians(p[3]))
    dw = np.sqrt(rw**2 + zw**2)

    u = np.arctan2(zw, rw)
    cosb = (l2**2 + l3**2 - dw**2) / (2*l2*l3)
    sinb = np.sqrt(1 - cosb**2)
    cosg = (dw**2 + l2**2 - l3**2) / (2*dw*l2)
    sing = np.sqrt(1 - cosg**2)
    
    b = np.arctan2(sinb, cosb)
    g = np.arctan2(sing, cosg)
    d = np.arctan2(l22, l21)

    bb = np.arctan2(-sinb, cosb)
    gg = np.arctan2(-sing, cosg)
    
    # Solution 1
    t1 = np.arctan2(p[1], p[0])
    t2 = np.pi/2 - d - g - u
    t3 = np.pi/2 + d - b
    t4 = -np.radians(p[3]) - t2 - t3

    # Solution 2
    tt1 = np.arctan2(p[1], p[0])
    tt2 = np.pi/2 - d - gg- u
    tt3 = np.pi/2 + d - bb
    tt4 = -np.radians(p[3]) - tt2 - tt3

    return [t1, t2, t3, t4], [tt1, tt2, tt3, tt4]

# Print Solutions
print(np.rad2deg(get_ik([274, 0, 204, 0])))
print(np.rad2deg(get_ik([16, 4, 336, 15])))
print(np.rad2deg(get_ik([0, -270, 106, 0])))