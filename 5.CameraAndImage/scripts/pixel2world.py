import numpy as np

def genIntrinsicMatrix(fx=500, fy=500, cx=320, cy=240):
    '''
    Get Intrinsic Matrix K
    Parameters:
        fx: Focal length along the x-axis
        fy: Focal length along the y-axis
        cx: Principal point x-coordinate
        cy: Principal point y-coordinate
    '''
    # Camera instrinsic matrix K
    K = np.array([[fx, 0 , cx],
                  [0 , fy, cy],
                  [0 , 0 , 1 ]])
    return K

def getTransformMatrix(R, t):
    # Convert to homogeneous coordinates
    vec = np.array([0,0,0,1])
    T   = np.vstack((np.hstack((R, t)), vec))
    return T


def pixel2camera(pixel, depth, K):
    # Convert to homogenous coordinates
    P_homogenous = np.append(pixel, 1)
    Pc = depth * np.linalg.inv(K) @ P_homogenous
    return Pc

def camera2world(Pc, Twc):
    # Convert to homogenous coordinates
    P_homogenous = np.append(Pc, 1)
    Pw = Twc @ P_homogenous
    Pw = Pw[:3] / Pw[3]
    return Pw



if __name__ == '__main__':
    pixel = np.array([360.6694888888889, 599.1167555555555]).astype(np.int8)
    depth = 9 # Zc
    K     = genIntrinsicMatrix()

    # Camera extrinsic parameters (rotation matrix and translation vector)
    R = np.array([[0.8660254, -0.5     , 0],
                  [0.5      , 0.8660254, 0],
                  [0        , 0        , 1]])
    t = np.array([1, 2, 3]).reshape(3, 1)

    Tcw = getTransformMatrix(R, t)
    Twc = np.linalg.inv(Tcw)
    Pc = pixel2camera(pixel, depth, K)
    Pw = camera2world(Pc, Twc)
    print("Puv: (u, v)    = ({},{})".format(*pixel))
    print("to: ")
    print("Pc : (X, Y, Z) = ({},{},{})".format(*Pc))
    print("Pw : (X, Y, Z) = ({},{},{})".format(*Pw))
    print("\n")
    
    
    
