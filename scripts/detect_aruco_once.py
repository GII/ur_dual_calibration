#!/usr/bin/env python3
"""
Sanity check for ArUco detection prior to running a hand-eye calibration.
 
This script does NOT compute any calibration. It only verifies that the
minimum perception chain is working before investing time in capturing
samples. It confirms that:
 
  1. The OAK-D camera publishes both image and camera_info topics.
  2. The configured ArUco marker (ID + dictionary) is detected.
  3. solvePnP returns a geometrically reasonable pose.
 
Usage:
  python3 detect_aruco_once.py
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


# Physical marker configuration
# ArUco dictionary used to print the marker.
MARKER_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL
# ID of the printed marker (must match what is physically mounted on the robot).
MARKER_ID = 100
# Side length of the BLACK square in meters. Measure with a caliper after printing.
MARKER_LENGTH = 0.0427



# Marker geometry in its own local frame
# Marker frame convention:
#   X -> right of the marker
#   Y -> up of the marker
#   Z -> out of the marker plane (towards the camera)
#
# Corner order required by SOLVEPNP_IPPE_SQUARE:
#   0: top-left  (-L/2, +L/2, 0)
#   1: top-right (+L/2, +L/2, 0)
#   2: bot-right (+L/2, -L/2, 0)
#   3: bot-left  (-L/2, -L/2, 0)

L = MARKER_LENGTH / 2.0
OBJECT_POINTS = np.array([
    [-L,  L, 0],
    [ L,  L, 0],
    [ L, -L, 0],
    [-L, -L, 0],
], dtype=np.float32)


# Camera topics (OAK-D launched with `namespace:=oak_cam`)
TOPIC_IMAGE = '/oak_cam/oak/rgb/image_raw'
TOPIC_INFO  = '/oak_cam/oak/rgb/camera_info'



def build_aruco_detector(dict_id):
    """
    Build an ArUco detector compatible with both OpenCV APIs.
 
    OpenCV >= 4.7 introduced the new `ArucoDetector` class while keeping the
    legacy `detectMarkers` function for backwards compatibility. We detect
    which one is available at runtime so the script works regardless of the
    OpenCV version installed on the host.
 
    Returns:
        (detect_fn, api_name) where detect_fn(frame) -> (corners, ids).
    """
  
    use_new_api = hasattr(cv2.aruco, 'ArucoDetector')

    if use_new_api:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        def detect(frame):
            corners, ids, _ = detector.detectMarkers(frame)
            return corners, ids
        api_name = 'nueva (ArucoDetector)'
    else:
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        params = cv2.aruco.DetectorParameters_create()
        def detect(frame):
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame, aruco_dict, parameters=params)
            return corners, ids
        api_name = 'antigua (detectMarkers)'

    return detect, api_name


class ArucoSanity(Node):
    """
    ROS 2 node that detects the configured ArUco and reports its pose.
 
    Workflow per incoming image:
      1. Wait until camera_info has provided intrinsics (K) and distortion (D).
      2. Run ArUco detection on the RGB frame.
      3. Look up the configured MARKER_ID in the detected list.
      4. Estimate marker pose with solvePnP (IPPE_SQUARE is the right solver
         for a planar 4-corner marker).
      5. Log distance and pose for visual verification.
    """
  
    def __init__(self):
        super().__init__('aruco_sanity')
        self.bridge = CvBridge()
 
        # Intrinsics are loaded once from camera_info and reused.
        self.K = None
        self.D = None
 
        self.create_subscription(CameraInfo, TOPIC_INFO, self.cb_info, 10)
        self.create_subscription(Image, TOPIC_IMAGE, self.cb_img, 10)
 
        self.detect, api_name = build_aruco_detector(MARKER_DICT)
 
        self.get_logger().info(
            f'OpenCV {cv2.__version__} | ArUco API: {api_name}')
        self.get_logger().info(
            f'Waiting for {TOPIC_INFO} and {TOPIC_IMAGE}... '
            f'Looking for ArUco ID={MARKER_ID}, L={MARKER_LENGTH*1000:.0f} mm')
 
    def cb_info(self, msg):
        """Cache intrinsics on the first camera_info message."""
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f'Intrinsics received:\n'
                f'K =\n{self.K}\n'
                f'D = {self.D}\n'
                f'Resolution: {msg.width}x{msg.height}')
 
    def cb_img(self, msg):
        """Detect the marker on each frame and log its pose."""
        # Skip frames until intrinsics are available.
        if self.K is None:
            return
 
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
 
        corners, ids = self.detect(frame)
        if ids is None:
            self.get_logger().warn(
                'No ArUco markers detected',
                throttle_duration_sec=2.0)
            return
 
        ids_flat = ids.flatten().tolist()
        if MARKER_ID not in ids_flat:
            self.get_logger().warn(
                f'Detected {ids_flat}, but not ID {MARKER_ID}',
                throttle_duration_sec=2.0)
            return
 
        idx = ids_flat.index(MARKER_ID)
        img_pts = corners[idx].reshape(4, 2).astype(np.float32)
 
        # IPPE_SQUARE is the recommended solver for planar 4-corner markers.
        ok, rvec, tvec = cv2.solvePnP(
            OBJECT_POINTS, img_pts, self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            self.get_logger().error('solvePnP failed')
            return
 
        tvec = tvec.ravel()
        rvec = rvec.ravel()
        dist = float(np.linalg.norm(tvec))
        self.get_logger().info(
            f'ArUco {MARKER_ID} OK | '
            f'tvec=[{tvec[0]:+.3f} {tvec[1]:+.3f} {tvec[2]:+.3f}] m | '
            f'rvec=[{rvec[0]:+.3f} {rvec[1]:+.3f} {rvec[2]:+.3f}] | '
            f'd={dist:.3f} m',
            throttle_duration_sec=1.0)
 
 
def main():
    rclpy.init()
    node = ArucoSanity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
