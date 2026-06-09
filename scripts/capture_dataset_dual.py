#!/usr/bin/env python3
"""
Captura automática de dataset YOLO desde OAK-D + Intel RealSense L515.

Cada INTERVAL segundos guarda un frame de cada cámara en carpetas separadas.
Emite un beep en terminal para que sepas cuándo capturó sin mirar la pantalla.

Uso:
  cd ~/ws_daniel
  python3 src/ur_softhand_dual/scripts/capture_dataset_dual.py

  # Con intervalo personalizado (por defecto 5 s):
  python3 src/ur_softhand_dual/scripts/capture_dataset_dual.py --interval 3

  # Pausar/reanudar: pulsa Enter en cualquier momento.
  # Salir: Ctrl+C.

Las imágenes se guardan en:
  <repo>/dataset_yolo/oak/img_XXXX_HHMMSS.png
  <repo>/dataset_yolo/l515/img_XXXX_HHMMSS.png

Ambas carpetas comparten el mismo img_id para que las imágenes
queden sincronizadas (fácil de asociar oak/img_0042 ↔ l515/img_0042).

Resolución de captura:
  - OAK-D: resolución nativa del topic (típicamente 1920×1080).
  - L515:  resolución nativa del topic (típicamente 1280×720).
  Se guardan en resolución COMPLETA. El resize a 640×640 lo hace
  Roboflow/YOLO durante el preprocesamiento de entrenamiento.
"""

import argparse
import cv2
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ── Topics ──────────────────────────────────────────────────────
TOPIC_OAK  = '/oak_cam/oak/rgb/image_raw'
TOPIC_L515 = '/camera/camera/color/image_raw'

# ── QoS profiles ───────────────────────────────────────────────
# OAK-D (depthai-ros) publica BEST_EFFORT → subscriber debe ser BEST_EFFORT
QOS_OAK = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# L515 (realsense-ros) publica RELIABLE → subscriber RELIABLE
QOS_L515 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── Output dirs ─────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT  = SCRIPT_DIR.parent
OAK_DIR    = REPO_ROOT / 'dataset_yolo' / 'oak'
L515_DIR   = REPO_ROOT / 'dataset_yolo' / 'l515'


class DualCapturer(Node):
    def __init__(self):
        super().__init__('dataset_capturer_dual')
        self.bridge = CvBridge()

        self.oak_img  = None
        self.l515_img = None
        self.oak_res  = None   # para reportar resolución
        self.l515_res = None

        # ── Subscripciones con QoS correcto para cada cámara ───
        self.create_subscription(
            Image, TOPIC_OAK, self._cb_oak, QOS_OAK)
        self.create_subscription(
            Image, TOPIC_L515, self._cb_l515, QOS_L515)

    # ── callbacks ───────────────────────────────────────────────
    def _cb_oak(self, msg):
        self.oak_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.oak_res is None:
            h, w = self.oak_img.shape[:2]
            self.oak_res = (w, h)

    def _cb_l515(self, msg):
        self.l515_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.l515_res is None:
            h, w = self.l515_img.shape[:2]
            self.l515_res = (w, h)

    # ── status ──────────────────────────────────────────────────
    def oak_ready(self):
        return self.oak_img is not None

    def l515_ready(self):
        return self.l515_img is not None

    # ── save ────────────────────────────────────────────────────
    def save(self, img_id: int) -> dict:
        """Guarda un frame de cada cámara disponible. Retorna dict con paths."""
        ts = datetime.now().strftime('%H%M%S')
        saved = {}

        if self.oak_img is not None:
            OAK_DIR.mkdir(parents=True, exist_ok=True)
            p = OAK_DIR / f'img_{img_id:04d}_{ts}.png'
            cv2.imwrite(str(p), self.oak_img.copy())
            saved['oak'] = p.name

        if self.l515_img is not None:
            L515_DIR.mkdir(parents=True, exist_ok=True)
            p = L515_DIR / f'img_{img_id:04d}_{ts}.png'
            cv2.imwrite(str(p), self.l515_img.copy())
            saved['l515'] = p.name

        return saved


def next_img_id():
    """Busca el mayor img_id existente en ambas carpetas."""
    max_id = 0
    for d in (OAK_DIR, L515_DIR):
        if not d.exists():
            continue
        for f in d.glob('img_*.png'):
            try:
                n = int(f.stem.split('_')[1])
                max_id = max(max_id, n)
            except (IndexError, ValueError):
                pass
    return max_id + 1


def beep():
    """Beep audible en terminal (carácter BEL)."""
    sys.stdout.write('\a')
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description='Captura dual OAK-D + L515')
    parser.add_argument('--interval', type=float, default=5.0,
                        help='Segundos entre capturas (default: 5)')
    args = parser.parse_args()

    rclpy.init()
    node = DualCapturer()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── Esperar al menos una cámara ─────────────────────────────
    print('Esperando cámaras...')
    timeout = 15.0
    t0 = time.time()
    while not node.oak_ready() and not node.l515_ready():
        if time.time() - t0 > timeout:
            break
        time.sleep(0.2)

    # Esperar un poco más por la segunda
    if node.oak_ready() != node.l515_ready():
        print('  Una cámara lista, esperando 8 s por la otra...')
        t1 = time.time()
        while time.time() - t1 < 8.0:
            if node.oak_ready() and node.l515_ready():
                break
            time.sleep(0.3)

    # ── Reporte de cámaras ──────────────────────────────────────
    cams = []
    if node.oak_ready():
        cams.append(f'OAK-D  ({node.oak_res[0]}x{node.oak_res[1]})')
    else:
        print(f'  ⚠  OAK-D no respondió en {TOPIC_OAK}')
        print(f'     Verificá QoS con: ros2 topic info {TOPIC_OAK} --verbose')
    if node.l515_ready():
        cams.append(f'L515   ({node.l515_res[0]}x{node.l515_res[1]})')
    else:
        print(f'  ⚠  L515 no respondió en {TOPIC_L515}')

    if not cams:
        print('ERROR: ninguna cámara respondió. Verificá los topics con:')
        print('  ros2 topic list | grep -E "oak|camera"')
        rclpy.shutdown()
        return

    print(f'\n  Cámaras activas:')
    for c in cams:
        print(f'    ✓ {c}')

    # ── Info de arranque ────────────────────────────────────────
    img_id = next_img_id()
    print(f'\n  OAK  -> {OAK_DIR}')
    print(f'  L515 -> {L515_DIR}')
    print(f'  Intervalo: {args.interval} s')
    if img_id > 1:
        print(f'  Continuando desde img_{img_id:04d} '
              f'(ya existen {img_id - 1} pares).')
    print(f'\n  Captura AUTOMATICA activa.')
    print(f'  Enter -> pausar/reanudar  |  Ctrl+C -> salir\n')

    # ── Estado de pausa (controlado desde hilo de input) ────────
    paused = threading.Event()       # set() = pausado

    def input_listener():
        while True:
            try:
                input()
            except EOFError:
                break
            if paused.is_set():
                paused.clear()
                print('  >>  Reanudado.\n')
            else:
                paused.set()
                print('  ||  Pausado. Enter para reanudar.\n')

    t = threading.Thread(target=input_listener, daemon=True)
    t.start()

    # ── Bucle principal ─────────────────────────────────────────
    try:
        while True:
            if paused.is_set():
                time.sleep(0.2)
                continue

            saved = node.save(img_id)
            if saved:
                parts = [f'{k}: {v}' for k, v in saved.items()]
                print(f'  [{img_id:04d}]  {" | ".join(parts)}')
                beep()
                img_id += 1

            time.sleep(args.interval)

    except KeyboardInterrupt:
        total = img_id - 1
        print(f'\n  Sesion terminada. Total: {total} capturas.')

    rclpy.shutdown()
    spin_thread.join(timeout=2)


if __name__ == '__main__':
    main()
