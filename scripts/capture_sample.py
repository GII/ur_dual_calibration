#!/usr/bin/env python3
"""
Captura de muestras para calibración hand-eye (eye-to-hand).

Por cada Enter:
  1. Espera 1.5 s para asegurar robot estático.
  2. Toma último frame RGB + último joint_states + TF base→tool.
  3. Detecta ArUco 100 y calcula su pose con solvePnP.
  4. Calcula error de reproyección como métrica de calidad.
  5. Guarda JSON + PNG numerados en calibration_data/samples/.

Basado en el patrón de rec.py (hilo de spin + input bloqueante).
Compatible con OpenCV < 4.7 y >= 4.7.

Uso:
  python3 capture_sample.py
"""
import cv2
import json
import numpy as np
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, JointState
from tf2_ros import Buffer, TransformListener, TransformException

# --- Configuración fija del sistema ------------------------------------
MARKER_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL
MARKER_DICT_NAME = 'DICT_ARUCO_ORIGINAL'
MARKER_ID = 100
MARKER_LENGTH = 0.0427  # m, lado NEGRO del ArUco

TOPIC_IMAGE  = '/oak_cam/oak/rgb/image_raw'
TOPIC_INFO   = '/oak_cam/oak/rgb/camera_info'
TOPIC_JOINTS = '/joint_states'

# OJO: los grupos MoveIt y los prefijos de joints están "cruzados":
#   Right_arm (perspectiva robot = tu izquierda) → joints con prefijo ur_dual_I_
#   Left_arm  (perspectiva robot = tu derecha)    → joints con prefijo ur_dual_D_
# El SoftHand está montado en el brazo con prefijo I_ (MoveIt group: Right_arm).
BASE_FRAME = 'ur_dual_I_base_link'
TOOL_FRAME = 'ur_dual_I_tool0'

# Tiempo de espera tras Enter para estabilizar (robot no móvil)
SETTLE_TIME_S = 1.5

# Ruta de salida: <repo>/calibration_data/samples/
SCRIPT_DIR  = Path(__file__).resolve().parent   # .../scripts
REPO_ROOT   = SCRIPT_DIR.parent                 # .../ur_softhand_dual
OUTPUT_DIR  = REPO_ROOT / 'calibration_data' / 'samples'

# Esquinas del marcador en su propio frame, orden SOLVEPNP_IPPE_SQUARE
L = MARKER_LENGTH / 2.0
OBJECT_POINTS = np.array([
    [-L,  L, 0],
    [ L,  L, 0],
    [ L, -L, 0],
    [-L, -L, 0],
], dtype=np.float32)


def build_aruco_detector(dict_id):
    """API compatible OpenCV < 4.7 (Dictionary_get) y >= 4.7 (ArucoDetector)."""
    if hasattr(cv2.aruco, 'ArucoDetector'):
        d = cv2.aruco.getPredefinedDictionary(dict_id)
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        def detect(frame):
            c, i, _ = det.detectMarkers(frame); return c, i
    else:
        d = cv2.aruco.Dictionary_get(dict_id)
        p = cv2.aruco.DetectorParameters_create()
        def detect(frame):
            c, i, _ = cv2.aruco.detectMarkers(frame, d, parameters=p); return c, i
    return detect


class SampleCapturer(Node):
    def __init__(self):
        super().__init__('sample_capturer')
        self.bridge = CvBridge()

        # Estado compartido (actualizado por callbacks en hilo de spin)
        self.K = None
        self.D = None
        self.width = None
        self.height = None
        self.camera_frame_id = None
        self.latest_image = None
        self.latest_joints = None

        self.create_subscription(CameraInfo, TOPIC_INFO,   self.cb_info,   10)
        self.create_subscription(Image,      TOPIC_IMAGE,  self.cb_img,    10)
        self.create_subscription(JointState, TOPIC_JOINTS, self.cb_joints, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detect      = build_aruco_detector(MARKER_DICT)

    # --- Callbacks ---
    def cb_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.width, self.height = msg.width, msg.height
            self.camera_frame_id = msg.header.frame_id
            self.get_logger().info(
                f'Intrínsecos OK ({self.width}x{self.height}, frame={self.camera_frame_id})')

    def cb_img(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def cb_joints(self, msg):
        self.latest_joints = {n: float(p) for n, p in zip(msg.name, msg.position)}

    def ready(self):
        return (self.K is not None
                and self.latest_image is not None
                and self.latest_joints is not None)

    # --- Operaciones de captura ---
    def lookup_flange(self):
        """TF de TOOL_FRAME en BASE_FRAME (pose del flange visto desde la base)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, TOOL_FRAME, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f'TF lookup falló: {e}')
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return {
            'translation':       [t.x, t.y, t.z],
            'quaternion_xyzw':   [r.x, r.y, r.z, r.w],
        }

    def detect_marker(self, frame):
        """Devuelve (dict | None, error_msg | None)."""
        corners, ids = self.detect(frame)
        if ids is None:
            return None, 'Ningún ArUco detectado'
        ids_flat = ids.flatten().tolist()
        if MARKER_ID not in ids_flat:
            return None, f'Detectados {ids_flat} pero no ID {MARKER_ID}'
        idx = ids_flat.index(MARKER_ID)
        img_pts = corners[idx].reshape(4, 2).astype(np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            OBJECT_POINTS, img_pts, self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None, 'solvePnP falló'

        # Error de reproyección: re-proyectamos las 4 esquinas y medimos px de diferencia
        projected, _ = cv2.projectPoints(OBJECT_POINTS, rvec, tvec, self.K, self.D)
        err_px = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - img_pts, axis=1)))

        return {
            'rvec':                   rvec.ravel().tolist(),
            'tvec':                   tvec.ravel().tolist(),
            'distance_m':             float(np.linalg.norm(tvec)),
            'reprojection_error_px':  err_px,
            'detected_corners_px':    img_pts.tolist(),
        }, None

    def capture_sample(self, sample_id):
        if not self.ready():
            self.get_logger().warn('Datos aún no disponibles')
            return False

        print(f'  Esperando {SETTLE_TIME_S}s para asegurar robot estático...')
        time.sleep(SETTLE_TIME_S)

        image = self.latest_image.copy()
        joints = dict(self.latest_joints)

        marker, err = self.detect_marker(image)
        if marker is None:
            self.get_logger().error(f'✗ Detección falló: {err}')
            return False

        flange = self.lookup_flange()
        if flange is None:
            return False

        # Feedback visible antes de guardar
        print('  --- Resultado ---')
        print(f'  ArUco {MARKER_ID} detectado | '
              f'distancia={marker["distance_m"]:.3f} m | '
              f'reproj={marker["reprojection_error_px"]:.2f} px')
        print(f'  Flange en base: '
              f'[{flange["translation"][0]:+.3f}, '
              f'{flange["translation"][1]:+.3f}, '
              f'{flange["translation"][2]:+.3f}] m')

        if marker['reprojection_error_px'] > 3.0:
            print('  ⚠ Reproyección > 3 px — considera descartar y recapturar.')

        # Guardado
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        json_path = OUTPUT_DIR / f'sample_{sample_id:03d}.json'
        png_path  = OUTPUT_DIR / f'sample_{sample_id:03d}.png'

        data = {
            'sample_id':       sample_id,
            'timestamp_iso':   datetime.now().isoformat(),
            'opencv_version':  cv2.__version__,
            'camera': {
                'topic':     TOPIC_IMAGE,
                'frame_id':  self.camera_frame_id,
                'width':     self.width,
                'height':    self.height,
                'K':         self.K.tolist(),
                'D':         self.D.tolist(),
            },
            'marker': {
                'dict':      MARKER_DICT_NAME,
                'id':        MARKER_ID,
                'length_m':  MARKER_LENGTH,
                **marker,
            },
            'flange': {
                'base_frame': BASE_FRAME,
                'tool_frame': TOOL_FRAME,
                **flange,
            },
            'joints':      joints,
            'image_file':  png_path.name,
        }

        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)
        cv2.imwrite(str(png_path), image)

        print(f'  ✓ Guardado: {json_path.name} + {png_path.name}\n')
        return True


def next_sample_id():
    """Determina el siguiente sample_XXX disponible sin sobrescribir."""
    if not OUTPUT_DIR.exists():
        return 1
    existing = sorted(OUTPUT_DIR.glob('sample_*.json'))
    if not existing:
        return 1
    last_id = int(existing[-1].stem.split('_')[1])
    return last_id + 1


def main():
    rclpy.init()
    node = SampleCapturer()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('Esperando cámara (image + camera_info) y /joint_states...')
    while not node.ready():
        time.sleep(0.1)

    print(f'\nListo.  Output → {OUTPUT_DIR}')
    print('Mueve el robot a cada pose planeada y presiona Enter.\n')

    sample_id = next_sample_id()
    if sample_id > 1:
        print(f'→ Continuando desde sample_{sample_id:03d} '
              f'(ya existen {sample_id-1} muestras previas)\n')

    try:
        while True:
            input(f'>>> Pose {sample_id:03d}: Enter para capturar '
                  f'(Ctrl+C para salir)... ')
            if node.capture_sample(sample_id):
                sample_id += 1
    except KeyboardInterrupt:
        print(f'\nSesión terminada. Total capturado: {sample_id-1} muestras.')

    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
