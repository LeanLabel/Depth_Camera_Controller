import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import json

class PoseDetector:
    def __init__(self, config_file='config.json'):
        # Load configuration
        with open(config_file) as f:
            self.config = json.load(f)

        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.marker_id = self.config["ARUCO_ID"]
        self.marker_length = self.config["ARUCO_LENGTH"]

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color,
                             self.config["RESOLUTION_X"],
                             self.config["RESOLUTION_Y"],
                             rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth,
                             self.config["RESOLUTION_X"],
                             self.config["RESOLUTION_Y"],
                             rs.format.z16, 30)
        self.profile = self.pipeline.start(config)

        # Get color intrinsics
        color_stream = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        self.camera_matrix = np.array([[self.intrinsics.fx, 0, self.intrinsics.ppx],
                                        [0, self.intrinsics.fy, self.intrinsics.ppy],
                                        [0, 0, 1]])
        self.dist_coeffs = np.array(self.intrinsics.coeffs)

        # Align depth to color
        self.align = rs.align(rs.stream.color)

    def get_frames(self):
        """Wait for aligned color and depth frames."""
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        return color_image, depth_frame   # depth_frame is the rs.frame, not the numpy array

    def detect_pose(self, color_image, depth_frame, return_annotated=False):
        """
        Detect the ArUco marker and compute its 6-DoF pose.
        If return_annotated is True, returns (success, t, q, annotated_image).
        Otherwise returns (success, t, q).
        """
        corners, ids, _ = aruco.detectMarkers(color_image, self.aruco_dict)

        if return_annotated:
            annotated = color_image.copy()

        if ids is None or self.marker_id not in ids:
            if return_annotated:
                cv2.putText(annotated, "Marker not detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                return False, None, None, annotated
            else:
                return False, None, None

        index = np.where(ids == self.marker_id)[0][0]
        marker_corners = corners[index].reshape((4, 2))

        # Get depth at each corner
        corner_depths = []
        valid_corners = True
        for (x, y) in marker_corners:
            x_int, y_int = int(round(x)), int(round(y))
            depth = depth_frame.get_distance(x_int, y_int)
            if depth <= 0:
                valid_corners = False
                break
            corner_depths.append(depth)

        if valid_corners and len(corner_depths) == 4:
            # Depth-based pose
            points_3d = []
            for i, (x, y) in enumerate(marker_corners):
                Z = corner_depths[i]
                X = (x - self.intrinsics.ppx) * Z / self.intrinsics.fx
                Y = (y - self.intrinsics.ppy) * Z / self.intrinsics.fy
                points_3d.append([X, Y, Z])
            points_3d = np.array(points_3d).T

            half = self.marker_length / 2.0
            model_points = np.array([
                [-half, -half, 0],
                [ half, -half, 0],
                [ half,  half, 0],
                [-half,  half, 0]
            ]).T

            R, t = self._rigid_transform_3D(model_points, points_3d)
            quat = self._rotation_matrix_to_quaternion(R)
            t = t.tolist()
            quat = quat.tolist()

            # Annotate if requested
            if return_annotated:
                cv2.putText(annotated, "Depth-based pose:", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.putText(annotated, f"t = [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                cv2.putText(annotated, f"q = [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                for i, (x, y) in enumerate(marker_corners):
                    cv2.circle(annotated, (int(x), int(y)), 3, (0,255,0), -1)
                    cv2.putText(annotated, f"{corner_depths[i]*1000:.1f}mm", (int(x)+5, int(y)-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)

            if return_annotated:
                return True, t, quat, annotated
            else:
                return True, t, quat

        else:
            # Fallback to geometric pose
            success, t, quat = self._fallback_pose(corners[index])
            if return_annotated:
                cv2.putText(annotated, "Geometric pose (depth invalid):", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                cv2.putText(annotated, f"t = [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                return True, t, quat, annotated
            else:
                return True, t, quat

    def _fallback_pose(self, marker_corners):
        """Fallback to OpenCV's solvePnP (geometric) when depth is invalid."""
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, self.marker_length, self.camera_matrix, self.dist_coeffs
        )
        rvec = rvec[0][0]   # shape (3,)
        tvec = tvec[0][0]   # shape (3,)
        R, _ = cv2.Rodrigues(rvec)
        quat = self._rotation_matrix_to_quaternion(R)
        return True, tvec.tolist(), quat.tolist()

    @staticmethod
    def _rigid_transform_3D(A, B):
        """Find R,t minimizing ||B - (R*A + t)||^2."""
        centroid_A = np.mean(A, axis=1, keepdims=True)
        centroid_B = np.mean(B, axis=1, keepdims=True)
        H = (A - centroid_A) @ (B - centroid_B).T
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = centroid_B - R @ centroid_A
        return R, t.flatten()

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """Convert 3x3 rotation matrix to quaternion [w,x,y,z]."""
        # (your existing implementation)
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
        return q   # [w, x, y, z]

    def stop(self):
        self.pipeline.stop()