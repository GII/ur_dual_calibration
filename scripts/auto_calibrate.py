#!/usr/bin/env python3
"""
auto_calibrate.py — Calibración hand-eye automatizada.

Lee calibration_poses.yaml y ejecuta cada pose vía el action server del
controller. Por cada pose: mueve, espera estabilización, detecta ArUco,
guarda muestra. Al final corre run_handeye, validate_handeye y
export_handeye_yaml.

Modos:
  --mode auto         : sin intervención humana, intenta todas las poses
  --mode semi         : confirmación entre poses (default)
  --max-reproj-px N   : umbral de aceptación (default 3.0)
  --velocity-frac F   : escala de velocidad UR5 (default 0.30)
  --skip-postproc     : no correr run_handeye/validate/export al final
  --dry-run           : listar poses sin mover ni capturar

Pre-requisitos:
  - calibration_data/samples/ vacía (archive_calibration.py si hace falta)
  - Driver OAK-D corriendo (namespace oak_cam)
  - Brazo en modo remote control, controller activo
  - calibration_poses.yaml generado con extract_poses_from_samples.py

Uso típico:
  cd ~/ws_daniel
  python3 src/ur_softhand_dual/scripts/auto_calibrate.py --mode semi
"""
import argparse
import cv2
import json
import numpy as np
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from cv_bridge import CvBridge

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import Image, CameraInfo, JointState
from tf2_ros import Buffer, TransformListener, TransformException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# --- Configuración fija ----------------------------------------------
MARKER_DICT_ID   = cv2.aruco.DICT_ARUCO_ORIGINAL
MARKER_DICT_NAME = 'DICT_ARUCO_ORIGINAL'
MARKER_ID        = 100
MARKER_LENGTH    = 0.0427   # m — lado negro del ArUco actual

TOPIC_IMAGE  = '/oak_cam/oak/rgb/image_raw'
TOPIC_INFO   = '/oak_cam/oak/rgb/camera_info'
TOPIC_JOINTS = '/joint_states'

BASE_FRAME = 'ur_dual_I_base_link'
TOOL_FRAME = 'ur_dual_I_tool0'

SETTLE_TIME_S         = 3.0
MIN_DURATION_S        = 3.0     # duración mínima de cualquier movimiento
NOMINAL_MAX_VEL_RAD_S = 1.5     # velocidad de joint a fracción = 1.0
TRAJECTORY_TIMEOUT_S  = 30.0
MAX_CONSECUTIVE_FAILS = 3       # aborta si fallan estos seguidos

SCRIPT_DIR  = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent
WS_ROOT     = REPO_ROOT.parent.parent              # ~/ws_daniel
POSES_YAML  = REPO_ROOT / 'src' / 'ur_dual_calibration' / 'config' / 'calibration_poses.yaml'
SAMPLES_DIR = REPO_ROOT / 'calibration_data' / 'samples'
LOG_PATH    = REPO_ROOT / 'calibration_data' / 'auto_calibrate.log'

# Esquinas del marcador en su propio frame (orden SOLVEPNP_IPPE_SQUARE)
L = MARKER_LENGTH / 2.0
OBJECT_POINTS = np.array([
    [-L,  L, 0],
    [ L,  L, 0],
    [ L, -L, 0],
    [-L, -L, 0],
], dtype=np.float32)


def build_aruco_detector(dict_id):
    if hasattr(cv2.aruco, 'ArucoDetector'):
        d = cv2.aruco.getPredefinedDictionary(dict_id)
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        def detect(frame):
            c, i, _ = det.detectMarkers(frame)
            return c, i
    else:
        d = cv2.aruco.Dictionary_get(dict_id)
        p = cv2.aruco.DetectorParameters_create()
        def detect(frame):
            c, i, _ = cv2.aruco.detectMarkers(frame, d, parameters=p)
            return c, i
    return detect


class AutoCalibrator(Node):
    def __init__(self, action_topic, joint_order):
        super().__init__('auto_calibrator')
        self.bridge = CvBridge()

        # Estado del nodo
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

        self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)
        self.detect = build_aruco_detector(MARKER_DICT_ID)
        self.joint_order = joint_order

    # --- Callbacks ---
    def cb_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.width, self.height = msg.width, msg.height
            self.camera_frame_id = msg.header.frame_id

    def cb_img(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def cb_joints(self, msg):
        self.latest_joints = {n: float(p) for n, p in zip(msg.name, msg.position)}

    def ready(self):
        return (self.K is not None
                and self.latest_image is not None
                and self.latest_joints is not None)

    # --- Movimiento ---
    def get_current_joints(self):
        if self.latest_joints is None:
            return None
        try:
            return [self.latest_joints[j] for j in self.joint_order]
        except KeyError:
            return None

    def compute_duration(self, target_joints, max_vel):
        current = self.get_current_joints()
        if current is None:
            return MIN_DURATION_S
        max_delta = max(abs(t - c) for t, c in zip(target_joints, current))
        return max(max_delta / max_vel, MIN_DURATION_S)

    def send_pose_goal(self, joint_values, duration_s, timeout_s=TRAJECTORY_TIMEOUT_S):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return False, 'Action server no disponible'

        traj = JointTrajectory()
        traj.joint_names = self.joint_order

        pt = JointTrajectoryPoint()
        pt.positions = list(joint_values)
        sec = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = self.action_client.send_goal_async(goal)
        deadline = time.time() + 10
        while not send_future.done():
            if time.time() > deadline:
                return False, 'Timeout esperando aceptación del goal'
            time.sleep(0.05)

        gh = send_future.result()
        if not gh.accepted:
            return False, 'Goal rechazado por el controller'

        result_future = gh.get_result_async()
        deadline = time.time() + timeout_s
        while not result_future.done():
            if time.time() > deadline:
                gh.cancel_goal_async()
                return False, f'Timeout {timeout_s}s en ejecución'
            time.sleep(0.05)

        result = result_future.result().result
        if result.error_code != 0:
            return False, f'error_code={result.error_code}: {result.error_string}'
        return True, 'OK'

    # --- Captura ---
    def lookup_flange(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, TOOL_FRAME, rclpy.time.Time())
        except TransformException as e:
            return None, f'TF: {e}'
        t, r = tf.transform.translation, tf.transform.rotation
        return {
            'translation':     [t.x, t.y, t.z],
            'quaternion_xyzw': [r.x, r.y, r.z, r.w],
        }, None

    def detect_marker_pose(self, frame):
        corners, ids = self.detect(frame)
        if ids is None:
            return None, 'Ningún ArUco detectado'
        ids_flat = ids.flatten().tolist()
        if MARKER_ID not in ids_flat:
            return None, f'Detectados {ids_flat}, no ID {MARKER_ID}'
        idx = ids_flat.index(MARKER_ID)
        img_pts = corners[idx].reshape(4, 2).astype(np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            OBJECT_POINTS, img_pts, self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None, 'solvePnP falló'

        proj, _ = cv2.projectPoints(OBJECT_POINTS, rvec, tvec, self.K, self.D)
        err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)))

        return {
            'rvec':                  rvec.ravel().tolist(),
            'tvec':                  tvec.ravel().tolist(),
            'distance_m':            float(np.linalg.norm(tvec)),
            'reprojection_error_px': err,
            'detected_corners_px':   img_pts.tolist(),
        }, None

    def capture_at_pose(self, sample_id, pose_id):
        time.sleep(SETTLE_TIME_S)
        if not self.ready():
            return False, 'Topics aún no disponibles'

        image = self.latest_image.copy()
        joints = dict(self.latest_joints)

        marker, err = self.detect_marker_pose(image)
        if marker is None:
            return False, err

        flange, ferr = self.lookup_flange()
        if flange is None:
            return False, ferr

        SAMPLES_DIR.mkdir(parents=True, exist_ok=True)
        json_path = SAMPLES_DIR / f'sample_{sample_id:03d}.json'
        png_path  = SAMPLES_DIR / f'sample_{sample_id:03d}.png'

        data = {
            'sample_id':      sample_id,
            'pose_id':        pose_id,
            'timestamp_iso':  datetime.now().isoformat(),
            'opencv_version': cv2.__version__,
            'capture_mode':   'auto',
            'camera': {
                'topic':    TOPIC_IMAGE,
                'frame_id': self.camera_frame_id,
                'width':    self.width,
                'height':   self.height,
                'K':        self.K.tolist(),
                'D':        self.D.tolist(),
            },
            'marker': {
                'dict':     MARKER_DICT_NAME,
                'id':       MARKER_ID,
                'length_m': MARKER_LENGTH,
                **marker,
            },
            'flange': {
                'base_frame': BASE_FRAME,
                'tool_frame': TOOL_FRAME,
                **flange,
            },
            'joints':     joints,
            'image_file': png_path.name,
        }
        json.dump(data, open(json_path, 'w'), indent=2)
        cv2.imwrite(str(png_path), image)
        return True, data


def load_poses_yaml(path):
    if not path.exists():
        raise SystemExit(f'No existe {path}.\n'
                         f'Genera primero con extract_poses_from_samples.py')
    with open(path) as f:
        data = yaml.safe_load(f)
    return data['metadata'], data['poses']


def log_line(msg):
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(LOG_PATH, 'a') as f:
        f.write(f'[{datetime.now().isoformat()}] {msg}\n')


def prompt_continue_semi():
    """En modo semi, pregunta tras cada pose. Devuelve 'continue' | 'quit'."""
    while True:
        resp = input('  Enter=siguiente / q=terminar : ').strip().lower()
        if resp in ('', 'y', 's'):
            return 'continue'
        if resp == 'q':
            return 'quit'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mode', choices=['auto', 'semi'], default='semi')
    p.add_argument('--max-reproj-px', type=float, default=1.5)
    p.add_argument('--velocity-frac', type=float, default=0.30,
                   help='Velocidad efectiva = frac × 1.5 rad/s por joint')
    p.add_argument('--skip-postproc', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()

    metadata, poses = load_poses_yaml(POSES_YAML)
    joint_order  = metadata['joint_order']
    action_topic = metadata['action_topic']

    max_vel = max(args.velocity_frac * NOMINAL_MAX_VEL_RAD_S, 0.15)

    print('═══ AUTO CALIBRATION ═══')
    print(f'  Modo:           {args.mode}')
    print(f'  Max reproj px:  {args.max_reproj_px}')
    print(f'  Velocity:       {args.velocity_frac:.0%} → {max_vel:.2f} rad/s por joint')
    print(f'  Poses:          {len(poses)}')
    print(f'  Action topic:   {action_topic}')

    if args.dry_run:
        print('\n[DRY RUN] No se ejecuta movimiento ni captura.')
        for pose in poses:
            print(f'  {pose["id"]}: src={pose["source_sample"]} '
                  f'reproj_orig={pose["reproj_error_px"]}px')
        return

    # Aviso de samples preexistentes
    existing = list(SAMPLES_DIR.glob('sample_*.json')) if SAMPLES_DIR.exists() else []
    if existing:
        print(f'\n⚠ Hay {len(existing)} sample_XXX.json preexistentes.')
        print('  Se MEZCLARÁN con la nueva sesión si continúas.')
        print('  (Archiva con archive_calibration.py antes para empezar limpio.)')
        if input('  Continuar de todos modos? (s/N) ').strip().lower() != 's':
            print('Abortado.')
            return

    rclpy.init()
    node = AutoCalibrator(action_topic, joint_order)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('\nEsperando topics (cámara + joint_states)...')
    deadline = time.time() + 10
    while not node.ready():
        if time.time() > deadline:
            print('✗ Timeout. ¿Driver OAK-D y brazos arrancados?')
            rclpy.shutdown()
            return
        time.sleep(0.2)
    print('✓ Topics OK')

    log_line(f'=== Sesión modo={args.mode} '
             f'velocity_frac={args.velocity_frac} ===')

    n_ok, n_skip, n_fail, consec_fails = 0, 0, 0, 0
    sample_id = 1
    aborted = False

    try:
        for pose in poses:
            pid = pose['id']
            joints = pose['joints']
            print(f'\n──── {pid} → sample_{sample_id:03d} ────')

            duration = node.compute_duration(joints, max_vel)
            print(f'  Moviendo... (~{duration:.1f}s)')

            ok, msg = node.send_pose_goal(joints, duration)
            if not ok:
                print(f'  ✗ Movimiento falló: {msg}')
                log_line(f'{pid} MOVE_FAIL: {msg}')
                n_fail += 1
                consec_fails += 1
                if consec_fails >= MAX_CONSECUTIVE_FAILS:
                    print(f'\n✗ {MAX_CONSECUTIVE_FAILS} fallos consecutivos. Abortando.')
                    aborted = True
                    break
                if args.mode == 'semi' and prompt_continue_semi() == 'quit':
                    break
                continue

            ok, data = node.capture_at_pose(sample_id, pid)
            if not ok:
                print(f'  ✗ Captura: {data}')
                log_line(f'{pid} CAPTURE_FAIL: {data}')
                n_fail += 1
                consec_fails += 1
                if args.mode == 'semi' and prompt_continue_semi() == 'quit':
                    break
                continue

            reproj = data['marker']['reprojection_error_px']
            dist   = data['marker']['distance_m']
            print(f'  ✓ ArUco {MARKER_ID}: d={dist:.3f}m  reproj={reproj:.2f}px')

            if reproj > args.max_reproj_px:
                print(f'  ⚠ Rechazada (reproj > {args.max_reproj_px})')
                (SAMPLES_DIR / f'sample_{sample_id:03d}.json').unlink(missing_ok=True)
                (SAMPLES_DIR / f'sample_{sample_id:03d}.png').unlink(missing_ok=True)
                log_line(f'{pid} REJECT reproj={reproj:.2f}')
                n_skip += 1
                consec_fails = 0
                if args.mode == 'semi' and prompt_continue_semi() == 'quit':
                    break
                continue

            log_line(f'{pid} OK reproj={reproj:.2f} sample={sample_id:03d}')
            n_ok += 1
            consec_fails = 0
            sample_id += 1

            if args.mode == 'semi' and prompt_continue_semi() == 'quit':
                break

    except KeyboardInterrupt:
        print('\n⚠ Interrumpido por usuario.')
        aborted = True

    print(f'\n═══ RESUMEN ═══')
    print(f'  Aceptadas:  {n_ok}')
    print(f'  Rechazadas: {n_skip}  (reproj > {args.max_reproj_px})')
    print(f'  Fallidas:   {n_fail}  (mov/captura)')
    log_line(f'SUMMARY ok={n_ok} skip={n_skip} fail={n_fail} aborted={aborted}')

    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    if aborted or n_ok < 10:
        print(f'\n✗ Insuficientes muestras válidas ({n_ok}) o sesión abortada.')
        print('  Post-procesado NO se ejecuta.')
        return

    if args.skip_postproc:
        print('\n--skip-postproc → no se ejecuta post-procesado.')
        return

    print('\n═══ POST-PROCESADO ═══')
    for script in ['run_handeye.py', 'validate_handeye.py', 'export_handeye_yaml.py']:
        print(f'\n→ {script}')
        ret = subprocess.run(
            ['python3', str(SCRIPT_DIR / script)],
            cwd=str(WS_ROOT))
        if ret.returncode != 0:
            print(f'\n✗ {script} terminó con código {ret.returncode}. Revisa antes de continuar.')
            return

    print('\n✓ Calibración automática completada.')
    print('\nSiguiente paso (manual):')
    print('  cd ~/ws_daniel')
    print('  colcon build --packages-select ur_dual_calibration --symlink-install')
    print('  source install/setup.bash')
    print('  ros2 launch ur_dual_calibration publish_handeye.launch.py')


if __name__ == '__main__':
    main()

