import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import math
import json

def load_config(filename='config.json'):
    try:
        with open('config.json', 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f"Error: The config file '{filename}' was not found.")
        return {}
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON format in '{filename}': {e}.")
        return {}

config_file = load_config()

# ---------------------- Configuration ----------------------
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_id = config_file["ARUCO_ID"]
marker_length = config_file["ARUCO_LENGTH"]

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, config_file["RESOLUTION_X"], config_file["RESOLUTION_Y"], rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, config_file["RESOLUTION_X"], config_file["RESOLUTION_Y"], rs.format.z16, 30)

profile = pipeline.start(config)

# Get camera intrinsics
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                           [0, intrinsics.fy, intrinsics.ppy],
                           [0, 0, 1]])
dist_coeffs = np.array(intrinsics.coeffs)

align = rs.align(rs.stream.color)

# ---------------------- Helper Functions ----------------------

def get_depth_at_pixel(depth_frame, x, y):
    """Return depth in meters at (x,y) pixel, or 0 if invalid."""
    return depth_frame.get_distance(x, y)

def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix to a quaternion [w, x, y, z].
    """
    q = np.empty((4,))
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        q[0] = 0.25 / s
        q[1] = (R[2, 1] - R[1, 2]) * s
        q[2] = (R[0, 2] - R[2, 0]) * s
        q[3] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = 0.25 * s
            q[2] = (R[0, 1] + R[1, 0]) / s
            q[3] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[0] = (R[0, 2] - R[2, 0]) / s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = 0.25 * s
            q[3] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[0] = (R[1, 0] - R[0, 1]) / s
            q[1] = (R[0, 2] + R[2, 0]) / s
            q[2] = (R[1, 2] + R[2, 1]) / s
            q[3] = 0.25 * s
    return q  # [w, x, y, z]

def rigid_transform_3D(A, B):
    """
    Find rotation R and translation t that minimizes sum ||B - (R*A + t)||^2.
    A and B are 3xN arrays of corresponding points.
    Returns R (3x3) and t (3) as a numpy array.
    """
    assert A.shape == B.shape
    # Centroids
    centroid_A = np.mean(A, axis=1, keepdims=True)
    centroid_B = np.mean(B, axis=1, keepdims=True)
    # Covariance matrix
    H = (A - centroid_A) @ (B - centroid_B).T
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    # Special reflection case
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    return R, t.flatten()

# ---------------------- Main Loop ----------------------
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(color_image, aruco_dict)

        if ids is not None and marker_id in ids[0]:
            index = np.where(ids == marker_id)[0][0]
            marker_corners = corners[index].reshape((4, 2))  # 4 corners in pixel coordinates

            # --- Step 1: Get depth at each corner (in meters) ---
            corner_depths = []
            valid_corners = True
            for (x, y) in marker_corners:
                # Ensure pixel coordinates are integers
                x_int, y_int = int(round(x)), int(round(y))
                depth = get_depth_at_pixel(depth_frame, x_int, y_int)
                if depth <= 0:   # invalid depth
                    valid_corners = False
                    break
                corner_depths.append(depth)

            if valid_corners and len(corner_depths) == 4:
                # --- Step 2: Back-project corners to 3D camera coordinates ---
                # Using pinhole model: X = (u - cx) * Z / fx , Y = (v - cy) * Z / fy
                points_3d = []   # list of [x, y, z] in camera coordinates
                for i, (x, y) in enumerate(marker_corners):
                    Z = corner_depths[i]
                    X = (x - intrinsics.ppx) * Z / intrinsics.fx
                    Y = (y - intrinsics.ppy) * Z / intrinsics.fy
                    points_3d.append([X, Y, Z])
                points_3d = np.array(points_3d).T   # shape (3,4)

                # --- Step 3: Define model points of the marker in its own coordinate frame ---
                # Marker centered at (0,0,0) with normal along +Z, corners in order:
                # (top-left, top-right, bottom-right, bottom-left) - typical ArUco order.
                half = marker_length / 2.0
                model_points = np.array([
                    [-half, -half, 0],
                    [ half, -half, 0],
                    [ half,  half, 0],
                    [-half,  half, 0]
                ]).T   # shape (3,4)

                # --- Step 4: Solve for rigid transformation (R, t) aligning model to measured points ---
                R, t = rigid_transform_3D(model_points, points_3d)

                # --- Step 5: Convert rotation matrix to quaternion ---
                quat = rotation_matrix_to_quaternion(R)   # [w, x, y, z]

                # --- Display results ---
                cv2.putText(color_image, f"Depth-based pose:", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.putText(color_image, f"t = [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                cv2.putText(color_image, f"q = [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

                # Optional: draw the marker corners and their depth values
                for i, (x, y) in enumerate(marker_corners):
                    cv2.circle(color_image, (int(x), int(y)), 3, (0,255,0), -1)
                    cv2.putText(color_image, f"{corner_depths[i]*1000:.1f}mm", (int(x)+5, int(y)-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)

            else:
                # Fallback: use geometric ArUco pose if depth is invalid at any corner
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[index], marker_length, camera_matrix, dist_coeffs
                )
                rvec = rvec[0][0]
                tvec = tvec[0][0]
                cv2.putText(color_image, f"Geometric pose (depth invalid):", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                cv2.putText(color_image, f"t = [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}] m", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        else:
            cv2.putText(color_image, "Marker not detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        cv2.imshow('RealSense + Depth-Based Pose', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()