
import numpy as np

def getRawP(Pc, focal_length=500):
    X, Y, Z = Pc
    Xp = focal_length*(X/Z)
    Yp = focal_length*(Y/Z)
    return Xp, Yp


def getPixel(P_raw, alpha=1, beta=1, cx=320, cy=240):
    Xp, Yp = P_raw
    u = alpha * Xp + cx
    v = beta  * Yp + cy
    return u, v


def point2pixel(Pc, focal_length=500, alpha=1, beta=1, cx=320, cy=240):
    X, Y, Z = Pc
    fx = alpha * focal_length
    fy = beta  * focal_length
    u = fx * (X/Z) + cx
    v = fy * (Y/Z) + cy
    return u, v

def normalize(P):
    return P / P[2]

if __name__ == '__main__':
    # Pc = np.array([0.48803386666666704,4.309401066666666,6.0])
    Pc = np.array([0.7320507999999999,6.464101599999999,9.0])

    
    # 1 Step by Step
    P_raw = getRawP(Pc)
    u, v = getPixel(P_raw)
    print(f"Pixel coordinates: (u, v) = ({u}, {v})")
    
    # 2 Integrate
    u, v = point2pixel(Pc)
    print("Pc : (X, Y, Z) = ({},{},{})".format(*Pc))
    print("Pp : (u, v)    = ({},{})".format(u, v))
    
        