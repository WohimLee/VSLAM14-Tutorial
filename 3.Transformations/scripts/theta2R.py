import numpy as np

def R2theta(R):

    # Compute the angle of rotation
    angle = np.arccos(0.5*(np.trace(R) - 1))

    # Compute the eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eig(R)

    # Find the eigenvalue closest to 1
    index = np.argmin(np.abs(eigenvalues - 1))
    eigenvalue = eigenvalues[index]
    eigenvector = eigenvectors[:, index]

    # Normalize the eigenvector
    rotation_axis = eigenvector / np.linalg.norm(eigenvector)

    return angle, rotation_axis

if __name__ == '__main__':
    theta = 30 / 180 * np.pi
    # Define the rotation matrix
    R = np.array([
        [0.8660254, -0.5     , 0, 0],
        [0.5      , 0.8660254, 0, 0],
        [0        , 0        , 1, 0],
        [0        , 0        , 0, 1]])

    # Extract the rotation submatrix
    R_submatrix = R[:3, :3]

    # Convert the angle-axis to the angle-axis representation
    angle, axis = R2theta(R_submatrix)
    print("Real rotation angle: ", theta)
    print("Rotation axis: ", axis)
    print("Angle of rotation: ", angle)
