import numpy as np

def getIntrinsicMatrix(fx=500, fy=500, cx=320, cy=240):
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

def world2camera(Pw, Tcw):
    # Convert to homogeneous coordinates
    P_homogeneous = np.append(Pw, 1) 
    # 1 World to camera (homogenous coordinates)
    Pc  = Tcw @ P_homogeneous
    return Pc

def camera2pixel(Pc, K):
    # 2 Camera to pixel
    Puv = K @ Pc[:3]
    Puv = Puv[:2] / Puv[2]
    return Puv.astype(np.int8)
    
if __name__ == '__main__':
    # Camera extrinsic parameters (rotation matrix and translation vector)
    R = np.array([[0.8660254, -0.5     , 0],
                  [0.5      , 0.8660254, 0],
                  [0        , 0        , 1]])
    t = np.array([1, 2, 3]).reshape(3, 1)


    K   = getIntrinsicMatrix()
    Tcw = getTransformMatrix(R, t)

    Pw  = np.array([2, 4, 6])
    Pc  = world2camera(Pw, Tcw)
    Puv = camera2pixel(Pc, K)
    print("Pw : (X, Y, Z) = ({},{},{})".format(*Pw))
    print("to:")
    print("Pc : (X, Y, Z) = ({},{},{})".format(*Pc))
    print("Puv: (u, v)    = ({},{})".format(*Puv))
    print("\n")


