
import numpy as np

def rodrigues_rotation(n, theta):
    # n 旋转轴[3x1]
    # theta 为旋转角度
    # 旋转是过原点的，n 是旋转轴
    n = np.array(n).reshape(3, 1)
    nx, ny, nz = n[:, 0]
    M = np.array([
        [0, -nz, ny],
        [nz, 0, -nx],
        [-ny, nx, 0]
    ])
    R = np.eye(4)
    R[:3, :3] = np.cos(theta) * np.eye(3) +        \
                (1 - np.cos(theta)) * n @ n.T +    \
                np.sin(theta) * M
    return R

if __name__ == "__main__":
    n = np.array([0, 0, 1])
    theta = 30 / 180 * np.pi
    R = rodrigues_rotation(n, theta)
    print(R)