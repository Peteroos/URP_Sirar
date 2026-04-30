#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
import os


def get_aruco_dict():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)


def get_aruco_detector(aruco_dict):
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

        # 一些稳一点的参数
    parameters.adaptiveThreshWinSizeMin = 5
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(aruco_dict, parameters)
    else:
        return (aruco_dict, parameters)


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Only detect this ArUco ID (can override from CLI).
        self.declare_parameter('target_id', 27)
        self.target_id = int(self.get_parameter('target_id').value)
        # Real marker side length [m]. Must match the printed marker size.
        self.declare_parameter('marker_length_m', 0.19)
        self.marker_length = float(self.get_parameter('marker_length_m').value)

        # Pose estimation method:
        #   - aruco_pnp: use marker corners + intrinsics (recommended)
        #   - depth_center: use aligned depth at marker center (legacy behavior)
        self.declare_parameter('pose_method', 'aruco_pnp')
        self.pose_method = str(self.get_parameter('pose_method').value).strip().lower()
        if self.pose_method not in ('aruco_pnp', 'depth_center'):
            self.get_logger().warn(
                f"Unsupported pose_method='{self.pose_method}', fallback to 'aruco_pnp'."
            )
            self.pose_method = 'depth_center'

        # Pose publish convention:
        #   - optical: ROS optical frame convention (x right, y down, z forward)
        #   - camera_link: ROS body frame convention (x forward, y left, z up)
        self.declare_parameter('publish_coordinate_convention', 'optical')
        self.publish_coordinate_convention = str(
            self.get_parameter('publish_coordinate_convention').value
        ).strip().lower()
        if self.publish_coordinate_convention not in ('optical', 'camera_link'):
            self.get_logger().warn(
                f"Unsupported publish_coordinate_convention='{self.publish_coordinate_convention}', "
                "fallback to 'optical'."
            )
            self.publish_coordinate_convention = 'optical'

        # Optional output frame override. If empty and camera_link mode is selected,
        # the node auto-derives a non-optical frame name from camera_info frame_id.
        self.declare_parameter('publish_frame_id', '')
        self.publish_frame_id = str(self.get_parameter('publish_frame_id').value).strip()

        # Quick debug toggle requested by user: swap published y/z translation.
        self.declare_parameter('swap_yz_translation', False)
        self.swap_yz_translation = bool(self.get_parameter('swap_yz_translation').value)
        # Quick debug toggle requested by user: invert published z translation.
        self.declare_parameter('flip_z_translation', False)
        self.flip_z_translation = bool(self.get_parameter('flip_z_translation').value)

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, '/sirar/realsense/color/image_raw', self.image_callback, 10
        )
        self.camerainfo_sub = self.create_subscription(
            CameraInfo, '/sirar/realsense/color/camera_info', self.camerainfo_callback, 10
        )
        # NEW: aligned depth (registered to color)
        self.depth_sub = self.create_subscription(
            Image, '/sirar/realsense/aligned_depth_to_color/image_raw',
            self.depth_callback, 10
        )

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_tracker/pose', 10)
        self.debug_img_pub = self.create_publisher(Image, '/aruco_tracker/debug_image', 10)

        # Optional TF broadcasting so marker can be viewed as a frame in RViz.
        self.declare_parameter('publish_tf_frame', True)
        self.declare_parameter('marker_frame_name', 'aruco_tag')
        self.publish_tf_frame = bool(self.get_parameter('publish_tf_frame').value)
        self.marker_frame_name = str(self.get_parameter('marker_frame_name').value)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf_frame else None

        self.bridge = CvBridge()

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.frame_id = "camera_color_optical_frame"
        self._warned_frame_mismatch = False
        self._warned_lr_inversion = False
        self._warned_depth_identity_orientation = False
        self._warned_swap_yz_orientation = False
        self._warned_flip_z_orientation = False
        self._last_debug_log_t = 0.0
        self.declare_parameter('debug_log_period_s', 1.0)
        self.debug_log_period_s = float(self.get_parameter('debug_log_period_s').value)

        # Depth buffer
        self.depth_image = None
        self.depth_encoding = None
        self.depth_stamp = None

        # Visualization control
        self.declare_parameter('show_window', True)
        self.declare_parameter('publish_debug_image', True)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.gui_ok = self.show_window and ('DISPLAY' in os.environ)
        if self.show_window and not self.gui_ok:
            self.get_logger().warn("No DISPLAY detected, disabling window; will only publish debug_image.")
            self.show_window = False

        if self.show_window:
            cv2.namedWindow('Aruco Debug', cv2.WINDOW_NORMAL)

        # FPS
        self._last_t = None
        self._fps = 0.0

        # ArUco detector
        self.aruco_dict = get_aruco_dict()
        self.aruco_detector = get_aruco_detector(self.aruco_dict)

        self.get_logger().info(
            "ArucoDetectorNode started. "
            f"Using color + aligned depth; ID={self.target_id}; "
            f"pose_method={self.pose_method}; "
            f"marker_length_m={self.marker_length:.4f}; "
            f"publish_coordinate_convention={self.publish_coordinate_convention}; "
            f"publish_frame_id={self.publish_frame_id if self.publish_frame_id else '<auto>'}; "
            f"swap_yz_translation={self.swap_yz_translation}; "
            f"flip_z_translation={self.flip_z_translation}; "
            f"publish_tf_frame={self.publish_tf_frame}; "
            f"marker_frame_name={self.marker_frame_name}."
        )

    # ---------------- CameraInfo callback ----------------
    def camerainfo_callback(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        self.camera_matrix = K
        self.dist_coeffs = D
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    # ---------------- Depth callback ----------------
    def depth_callback(self, msg: Image):
        """
        Store latest aligned depth frame.
        """
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"depth_bridge_failed: {e}")
            return

        self.depth_image = depth
        self.depth_encoding = msg.encoding
        self.depth_stamp = msg.header.stamp

    # Small helper: get depth in meters around (u, v)
    def get_depth_at(self, u, v, window=5):
        """
        u, v: pixel coordinates (float/int)
        returns: depth in meters or None if invalid
        """
        if self.depth_image is None:
            return None

        h, w = self.depth_image.shape[:2]
        u = int(round(u))
        v = int(round(v))
        if not (0 <= u < w and 0 <= v < h):
            return None

        depth = self.depth_image

        # Direct reading
        d = depth[v, u]

        # Handle encoding and invalid values
        if self.depth_encoding in ['16UC1', 'mono16']:
            # RealSense depth: usually in millimeters
            if d == 0:
                d_m = 0.0
            else:
                d_m = float(d) * 0.001
        elif self.depth_encoding in ['32FC1']:
            d_m = float(d)
        else:
            # Fallback assumption
            d_m = float(d)

        if d_m > 0.05:  # valid depth > 5cm
            return d_m

        # If invalid, search in a small window around the center
        half = window // 2
        vals = []
        for yy in range(max(0, v - half), min(h, v + half + 1)):
            for xx in range(max(0, u - half), min(w, u + half + 1)):
                dd = depth[yy, xx]
                if self.depth_encoding in ['16UC1', 'mono16']:
                    if dd == 0:
                        continue
                    d_mm = float(dd) * 0.001
                elif self.depth_encoding in ['32FC1']:
                    d_mm = float(dd)
                else:
                    d_mm = float(dd)
                if d_mm > 0.05:
                    vals.append(d_mm)

        if len(vals) == 0:
            return None
        return float(np.median(vals))

    # ---------------- Image callback (color) ----------------
    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        if msg.header.frame_id and msg.header.frame_id != self.frame_id and not self._warned_frame_mismatch:
            self.get_logger().warn(
                f"image_raw frame_id ({msg.header.frame_id}) != camera_info frame_id ({self.frame_id}). "
                "Using camera_info frame_id for pose/TF publishing."
            )
            self._warned_frame_mismatch = True

        # FPS
        now = time.time()
        if self._last_t is not None:
            dt = max(1e-6, now - self._last_t)
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
        self._last_t = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if isinstance(self.aruco_detector, tuple):
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_detector[1]
            )
        else:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        vis = frame.copy()
        publish_frame_id = self.get_publish_frame_id()

        if ids is not None and len(ids) > 0:
            try:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            except Exception:
                pass

            # Only ID = target_id
            sel_idx = [i for i, mid in enumerate(ids.flatten()) if int(mid) == self.target_id]
            if sel_idx:
                sel_corners = [corners[i] for i in sel_idx]

                if self.pose_method == 'aruco_pnp':
                    try:
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            sel_corners,
                            self.marker_length,
                            self.camera_matrix.astype(np.float64),
                            self.dist_coeffs.astype(np.float64),
                        )
                    except Exception as e:
                        self.get_logger().warn(f"estimatePoseSingleMarkers failed: {e}")
                        rvecs, tvecs = None, None

                    if rvecs is not None and tvecs is not None:
                        for i, c in enumerate(sel_corners):
                            pts = c.reshape(-1, 2)
                            u_center = float(pts[:, 0].mean())
                            v_center = float(pts[:, 1].mean())

                            rvec = rvecs[i][0]
                            tvec = tvecs[i][0]
                            tvec_optical = np.array(tvec, dtype=np.float64).reshape(3,)
                            rot_mtx, _ = cv2.Rodrigues(rvec)
                            self.check_left_right_consistency(u_center, float(tvec_optical[0]))
                            tvec_pub, rot_pub = self.convert_pose_from_optical(tvec_optical, rot_mtx)
                            self._debug_log_pose(tvec_optical, tvec_pub, publish_frame_id)
                            tvec_pub = self.maybe_swap_yz_translation(tvec_pub)
                            tvec_pub = self.maybe_flip_z_translation(tvec_pub)
                            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(rot_pub)

                            if self.swap_yz_translation and not self._warned_swap_yz_orientation:
                                self.get_logger().warn(
                                    "swap_yz_translation=True: only position axes are swapped; orientation is unchanged."
                                )
                                self._warned_swap_yz_orientation = True
                            if self.flip_z_translation and not self._warned_flip_z_orientation:
                                self.get_logger().warn(
                                    "flip_z_translation=True: only position Z is inverted; orientation is unchanged."
                                )
                                self._warned_flip_z_orientation = True

                            pose_msg = PoseStamped()
                            pose_msg.header.stamp = msg.header.stamp
                            pose_msg.header.frame_id = publish_frame_id
                            pose_msg.pose.position.x = float(tvec_pub[0])
                            pose_msg.pose.position.y = float(tvec_pub[1])
                            pose_msg.pose.position.z = float(tvec_pub[2])
                            pose_msg.pose.orientation.x = float(qx)
                            pose_msg.pose.orientation.y = float(qy)
                            pose_msg.pose.orientation.z = float(qz)
                            pose_msg.pose.orientation.w = float(qw)
                            self.pose_pub.publish(pose_msg)

                            if self.tf_broadcaster is not None:
                                t = TransformStamped()
                                t.header = pose_msg.header
                                t.header.frame_id = pose_msg.header.frame_id
                                t.child_frame_id = self.marker_frame_name
                                t.transform.translation.x = pose_msg.pose.position.x
                                t.transform.translation.y = pose_msg.pose.position.y
                                t.transform.translation.z = pose_msg.pose.position.z
                                t.transform.rotation = pose_msg.pose.orientation
                                self.tf_broadcaster.sendTransform(t)

                            cv2.circle(vis, (int(round(u_center)), int(round(v_center))),
                                       5, (0, 0, 255), -1)
                            try:
                                cv2.drawFrameAxes(
                                    vis,
                                    self.camera_matrix,
                                    self.dist_coeffs,
                                    rvec,
                                    tvec,
                                    self.marker_length * 0.5,
                                )
                            except Exception:
                                pass

                            dist = float(np.linalg.norm(tvec_optical))
                            cv2.putText(vis,
                                        f"ID {self.target_id}  dist: {dist:.3f} m",
                                        (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.9, (0, 255, 0), 2)

                else:
                    if self.depth_image is None:
                        self.get_logger().warn("pose_method=depth_center but depth image not available.")
                    else:
                        for c in sel_corners:
                            pts = c.reshape(-1, 2)
                            u_center = float(pts[:, 0].mean())
                            v_center = float(pts[:, 1].mean())

                            # Get depth at marker center
                            z = self.get_depth_at(u_center, v_center)
                            if z is None:
                                self.get_logger().warn("No valid depth at marker center.")
                                continue

                            # Back-project to 3D (camera frame, OpenCV convention)
                            fx = self.camera_matrix[0, 0]
                            fy = self.camera_matrix[1, 1]
                            cx = self.camera_matrix[0, 2]
                            cy = self.camera_matrix[1, 2]

                            X = (u_center - cx) * z / fx
                            Y = (v_center - cy) * z / fy
                            Z = z
                            self.check_left_right_consistency(u_center, float(X))

                            xyz_pub, _ = self.convert_pose_from_optical(
                                np.array([X, Y, Z], dtype=np.float64),
                                None,
                            )
                            xyz_pub = self.maybe_swap_yz_translation(xyz_pub)
                            xyz_pub = self.maybe_flip_z_translation(xyz_pub)

                            if (
                                self.publish_coordinate_convention == 'camera_link'
                                and not self._warned_depth_identity_orientation
                            ):
                                self.get_logger().warn(
                                    "pose_method=depth_center publishes identity orientation; "
                                    "only position is converted to camera_link convention."
                                )
                                self._warned_depth_identity_orientation = True

                            cv2.circle(vis, (int(round(u_center)), int(round(v_center))),
                                       5, (0, 0, 255), -1)

                            pose_msg = PoseStamped()
                            pose_msg.header.stamp = msg.header.stamp
                            pose_msg.header.frame_id = publish_frame_id
                            pose_msg.pose.position.x = float(xyz_pub[0])
                            pose_msg.pose.position.y = float(xyz_pub[1])
                            pose_msg.pose.position.z = float(xyz_pub[2])
                            pose_msg.pose.orientation.x = 0.0
                            pose_msg.pose.orientation.y = 0.0
                            pose_msg.pose.orientation.z = 0.0
                            pose_msg.pose.orientation.w = 1.0
                            self.pose_pub.publish(pose_msg)

                            if self.tf_broadcaster is not None:
                                t = TransformStamped()
                                t.header = pose_msg.header
                                t.header.frame_id = pose_msg.header.frame_id
                                t.child_frame_id = self.marker_frame_name
                                t.transform.translation.x = pose_msg.pose.position.x
                                t.transform.translation.y = pose_msg.pose.position.y
                                t.transform.translation.z = pose_msg.pose.position.z
                                t.transform.rotation = pose_msg.pose.orientation
                                self.tf_broadcaster.sendTransform(t)

                            dist = float(np.sqrt(X * X + Y * Y + Z * Z))
                            cv2.putText(vis,
                                        f"ID {self.target_id}  dist: {dist:.3f} m",
                                        (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.9, (0, 255, 0), 2)

        # FPS text
        cv2.putText(vis, f"FPS: {self._fps:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(vis, f"Out frame: {publish_frame_id}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(vis, f"Swap YZ: {'ON' if self.swap_yz_translation else 'OFF'}", (10, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(vis, f"Flip Z: {'ON' if self.flip_z_translation else 'OFF'}", (10, 140),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Publish debug image
        if self.publish_debug_image:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                dbg_msg.header = msg.header
                self.debug_img_pub.publish(dbg_msg)
            except Exception as e:
                self.get_logger().warn(f"publish debug image failed: {e}")

        # Local window
        if self.show_window:
            try:
                cv2.imshow('Aruco Debug', vis)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().warn(f"OpenCV GUI error: {e}. Disabling window.")
                self.show_window = False
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        # Returns (x, y, z, w)
        q = np.empty((4,), dtype=np.float64)
        t = np.trace(R)
        if t > 0.0:
            t = np.sqrt(t + 1.0)
            q[3] = 0.5 * t
            t = 0.5 / t
            q[0] = (R[2, 1] - R[1, 2]) * t
            q[1] = (R[0, 2] - R[2, 0]) * t
            q[2] = (R[1, 0] - R[0, 1]) * t
        else:
            i = 0
            if R[1, 1] > R[0, 0]:
                i = 1
            if R[2, 2] > R[i, i]:
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3
            t = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (R[k, j] - R[j, k]) * t
            q[j] = (R[j, i] + R[i, j]) * t
            q[k] = (R[k, i] + R[i, k]) * t
        return q[0], q[1], q[2], q[3]

    @staticmethod
    def optical_to_camera_link_rotation():
        # Convert from optical convention (x right, y down, z forward)
        # to camera_link/body convention (x forward, y left, z up).
        return np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ], dtype=np.float64)

    def get_publish_frame_id(self):
        if self.publish_coordinate_convention == 'optical':
            return self.frame_id

        if self.publish_frame_id:
            return self.publish_frame_id

        if self.frame_id.endswith('_optical_frame'):
            return self.frame_id[:-len('_optical_frame')] + '_frame'
        if self.frame_id.endswith('_optical'):
            return self.frame_id[:-len('_optical')]
        return f"{self.frame_id}_camera_link"

    def convert_pose_from_optical(self, t_xyz, rot_mtx=None):
        t_xyz = np.array(t_xyz, dtype=np.float64).reshape(3,)
        if self.publish_coordinate_convention == 'optical':
            return t_xyz, rot_mtx

        r_conv = self.optical_to_camera_link_rotation()
        t_out = r_conv @ t_xyz
        if rot_mtx is None:
            return t_out, None
        rot_out = r_conv @ rot_mtx
        return t_out, rot_out

    def maybe_swap_yz_translation(self, xyz):
        xyz = np.array(xyz, dtype=np.float64).reshape(3,)
        if not self.swap_yz_translation:
            return xyz
        return np.array([xyz[0], xyz[2], xyz[1]], dtype=np.float64)

    def maybe_flip_z_translation(self, xyz):
        xyz = np.array(xyz, dtype=np.float64).reshape(3,)
        if not self.flip_z_translation:
            return xyz
        return np.array([xyz[0], xyz[1], -xyz[2]], dtype=np.float64)

    def _debug_log_pose(self, tvec_optical, tvec_pub, publish_frame_id):
        """Throttled log: show optical-frame tvec and camera_link-frame equivalent.

        Use this to diagnose whether the mismatch is in OpenCV pose solving
        or in the URDF/TF chain. When the marker is placed right below a
        downward-looking camera, tvec_optical should be roughly (0, 0, +Z).
        """
        now = time.time()
        if now - self._last_debug_log_t < self.debug_log_period_s:
            return
        self._last_debug_log_t = now

        r_conv = self.optical_to_camera_link_rotation()
        tvec_cl = r_conv @ np.asarray(tvec_optical, dtype=np.float64).reshape(3,)
        self.get_logger().info(
            "[pose-debug] "
            f"camera_info.frame_id={self.frame_id} | "
            f"publish_frame_id={publish_frame_id} | "
            f"tvec_optical(x_right, y_down, z_forward)=({tvec_optical[0]:+.3f}, "
            f"{tvec_optical[1]:+.3f}, {tvec_optical[2]:+.3f}) m | "
            f"tvec_camera_link(x_fwd, y_left, z_up)=({tvec_cl[0]:+.3f}, "
            f"{tvec_cl[1]:+.3f}, {tvec_cl[2]:+.3f}) m | "
            f"published_tvec=({tvec_pub[0]:+.3f}, {tvec_pub[1]:+.3f}, {tvec_pub[2]:+.3f}) m"
        )

    def check_left_right_consistency(self, u_center, x_m):
        if self.camera_matrix is None or self._warned_lr_inversion:
            return
        cx = float(self.camera_matrix[0, 2])
        pixel_offset = float(u_center - cx)
        # In a normal optical frame, (u - cx) and X should have the same sign.
        if abs(pixel_offset) > 20.0 and abs(x_m) > 0.02 and pixel_offset * x_m < 0.0:
            self.get_logger().warn(
                "Left-right inconsistency detected: marker pixel is on one side of principal point, "
                "but solved X has opposite sign. This usually means mirrored image input or wrong camera frame/extrinsic."
            )
            self._warned_lr_inversion = True


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()

