#!/usr/bin/env python3
"""
Sanity check de deteccion de ArUco para la calibracion hand-eye de la camara.

---Objetivo del script:---
Validar que la cadena minima de percepcion funciona correctamente antes de
invertir tiempo en capturar muestras para calibracion. En particular, este
nodo comprueba que:

  1. La camara OAK-D publica imagenes y camera_info.
  2. El detector encuentra el ArUco ID 100 (DICT_ARUCO_ORIGINAL).
  3. solvePnP devuelve una pose (tvec, rvec) geometricamente razonable.

--------------------
NO hace calibración.
--------------------

Uso:
  python3 detect_aruco_once.py

"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


# ---Configuración del marcador físico ---
# Diccionario ArUco original (Es el que se imprime para realizar la calibración hand-eye).
MARKER_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL
# ID del marcador impreso (Debe coincidir con el impreso).
MARKER_ID = 100
# Lado negro en metros (75 mm, debe ser medido despues de impreso, colocar valor exacto)
MARKER_LENGTH = 0.0427 


# ---Geometría 3D del marcador en su frame local---
# Aca se define el marcador centrado en el origen de su propio frame.
#
# Convención del frame del marcador:
#   X -> hacia la derecha del marcador
#   Y -> hacia arriba del marcador
#   Z -> saliendo del plano del marcador
#
# Esquinas del marcador en su propio frame, orden SOLVEPNP_IPPE_SQUARE:
#   0: (-L/2, +L/2, 0)   Esquina superior izquierda
#   1: (+L/2, +L/2, 0)   Esquina superior derecha
#   2: (+L/2, -L/2, 0)   Esquina inferior derecha
#   3: (-L/2, -L/2, 0)   Esquina inferior izquierda
#
L = MARKER_LENGTH / 2.0
OBJECT_POINTS = np.array([
    [-L,  L, 0],
    [ L,  L, 0],
    [ L, -L, 0],
    [-L, -L, 0],
], dtype=np.float32)

# --- Topics de la OAK-D ---
# Imagen RGB de la OAK-D.
TOPIC_IMAGE = '/oak_cam/oak/rgb/image_raw'
# Intrínsecos y distorsión de la cámara RGB de la OAK-D.
TOPIC_INFO  = '/oak_cam/oak/rgb/camera_info'


# --- Wrapper de API ArUco ---
def build_aruco_detector(dict_id):
    """
    Construye un detector ArUco compatible con diferentes versiones de OpenCV.

    Parámetros:
    dict_id : int
        Identificador del diccionario ArUco de OpenCV.

    Retorna:
    detect : callable
        Función con firma detect(frame) -> (corners, ids), donde:
        - corners: lista de esquinas detectadas
        - ids: IDs de los marcadores detectados
    api_name : str
        Nombre descriptivo de la API usada, útil para logging.
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
    Nodo de ROS 2 para validación básica de detección ArUco y una estimación de pose.

    Como funciona:
    1. Espera recibir camera_info para cargar intrínsecos K y distorsión D.
    2. Espera recibir imágenes RGB.
    3. Detecta marcadores ArUco en cada imagen.
    4. Busca específicamente el ID configurado en MARKER_ID.
    5. Ejecuta solvePnP para estimar la pose marcador->cámara.
    6. Reporta por consola la pose estimada como sanity check.

    Estado interno:
    self.K : np.ndarray or None
        Matriz intrínseca 3x3 de la cámara.
    self.D : np.ndarray or None
        Vector de coeficientes de distorsión.
    """
    def __init__(self):
        super().__init__('aruco_sanity')
        # Conversor ROS Image a OpenCV image.
        self.bridge = CvBridge()

        # Intrínsecos y distorsión se cargan una sola vez desde camera_info.
        self.K = None
        self.D = None

        # Suscripciones a intrínsecos y flujo de imagen.
        self.create_subscription(CameraInfo, TOPIC_INFO, self.cb_info, 10)
        self.create_subscription(Image, TOPIC_IMAGE, self.cb_img, 10)

        # Construcción del detector ArUco
        self.detect, api_name = build_aruco_detector(MARKER_DICT)

        self.get_logger().info(
            f'OpenCV {cv2.__version__} | API ArUco: {api_name}')
        self.get_logger().info(
            f'Esperando {TOPIC_INFO} y {TOPIC_IMAGE}... '
            f'Buscando ArUco ID={MARKER_ID}, L={MARKER_LENGTH*1000:.0f} mm')

    def cb_info(self, msg):
        """
        Callback de camera_info.

        Guarda la matriz intrínseca K y los coeficientes de distorsión D
        solo la primera vez que llegan.
        """
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f'Intrínsecos recibidos:\n'
                f'K =\n{self.K}\n'
                f'D = {self.D}\n'
                f'Resolución: {msg.width}x{msg.height}')

    def cb_img(self, msg):
        """
        Callback de imagen RGB.

        Procesa cada frame recibido para:
        - detectar marcadores,
        - localizar el ID de interés,
        - estimar la pose del marcador respecto a la cámara usando solvePnP.
        """
        if self.K is None:
            return  # aún sin intrínsecos
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        corners, ids = self.detect(frame)
        if ids is None:
            self.get_logger().warn(
                'Ningún ArUco detectado en la imagen',
                throttle_duration_sec=2.0)
            return

        ids_flat = ids.flatten().tolist()
        if MARKER_ID not in ids_flat:
            self.get_logger().warn(
                f'Detectados {ids_flat}, pero no el ID {MARKER_ID}',
                throttle_duration_sec=2.0)
            return

        idx = ids_flat.index(MARKER_ID)
        img_pts = corners[idx].reshape(4, 2).astype(np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            OBJECT_POINTS, img_pts, self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            self.get_logger().error('solvePnP falló')
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
