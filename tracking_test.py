import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import math

# ---------------------- Configuration ----------------------
# ArUco dictionary and marker parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_id = 23                 # The ID of your marker
marker_length = 0.05            # Physical marker side length in meters (e.g., 5 cm)

# RealSense streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)

# Get camera intrinsics (used for pose estimation)
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
# Convert RealSense intrinsics to OpenCV camera matrix format
camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                           [0, intrinsics.fy, intrinsics.ppy],
                           [0, 0, 1]])
dist_coeffs = np.array(intrinsics.coeffs)  # Distortion coefficients (if any)

# Align depth to color
align = rs.align(rs.stream.color)

# ---------------------- Helper Functions ----------------------
def get_depth_at_pixel(depth_frame, x, y):
    """Return depth in meters at (x,y) pixel, or 0 if invalid."""
    return depth_frame.get_distance(x, y)

def draw_axis(img, rvec, tvec, camera_matrix, dist_coeffs, length=0.03):
    """Draw 3D axis on image."""
    # Project 3D points of axis to image plane
    axis_points = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]]).reshape(-1,3)
    img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    img_points = img_points.astype(int)
    origin = tuple(img_points[0].ravel())
    img = cv2.line(img, origin, tuple(img_points[1].ravel()), (0,0,255), 3)  # X in red
    img = cv2.line(img, origin, tuple(img_points[2].ravel()), (0,255,0), 3)  # Y in green
    img = cv2.line(img, origin, tuple(img_points[3].ravel()), (255,0,0), 3)  # Z in blue
    return img

# ---------------------- Main Loop ----------------------
try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # ---------------------- ArUco Detection ----------------------
        corners, ids, rejected = aruco.detectMarkers(color_image, aruco_dict)

        if ids is not None and marker_id in ids[0]:
            # Get index of our marker
            index = np.where(ids == marker_id)[0][0]
            marker_corners = corners[index].reshape((4, 2))  # 4 corner points

            # Estimate pose (requires camera matrix, distortion, and marker length)
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners[index], marker_length, camera_matrix, dist_coeffs
            )
            rvec = rvec[0][0]   # shape (3,)
            tvec = tvec[0][0]   # shape (3,)

            # Option 1: Use tvec from pose estimation (gives X, Y, Z in camera coordinates)
            # Option 2: Get depth at marker center for a potentially more accurate Z
            # Let's compute marker center in pixels
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            depth_at_center = get_depth_at_pixel(depth_frame, center_x, center_y)

            # Overlay marker border and ID
            aruco.drawDetectedMarkers(color_image, corners, ids)

            # Draw 3D axis
            color_image = draw_axis(color_image, rvec, tvec, camera_matrix, dist_coeffs)

            # Display 3D position from pose estimation
            pos_text = f"Pose: X={tvec[0]:.3f}m Y={tvec[1]:.3f}m Z={tvec[2]:.3f}m"
            cv2.putText(color_image, pos_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0,255,255), 2)

            # If depth is valid, show it too
            if depth_at_center > 0:
                depth_text = f"Depth at center: {depth_at_center:.3f}m"
                cv2.putText(color_image, depth_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255,255,0), 2)
        else:
            cv2.putText(color_image, "Marker not detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # Show the image
        cv2.imshow('RealSense + ArUco Pose Tracking', color_image)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()