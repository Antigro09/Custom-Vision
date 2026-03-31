import numpy as np
import cv2

def tag_object_points(tag_size_m: float):
    s = tag_size_m / 2.0
    return np.array([
        [-s, -s, 0.0],
        [ s, -s, 0.0],
        [ s,  s, 0.0],
        [-s,  s, 0.0]
    ], dtype=np.float64)

def euler_from_rvec(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = (R[0,0]**2 + R[1,0]**2) ** 0.5
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0.0
    return np.degrees([x, y, z])

def estimate_pose(corners, K, D, tag_size_m):
    obj_pts = tag_object_points(tag_size_m)
    img_pts = np.array(corners, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    euler = euler_from_rvec(rvec)
    dist = float(np.linalg.norm(tvec))
    return rvec, tvec, euler, dist