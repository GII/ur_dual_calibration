#!/usr/bin/env python3
"""
Ejecuta calibrateHandEye sobre los samples capturados.

Configuración eye-to-hand: cámara fija (OAK-D en trípode), marcador en el
robot. Se usa el truco de invertir gripper→base para que OpenCV devuelva
T_camera→base en vez de T_camera→gripper.

Compara los 4 métodos disponibles. Si coinciden en mm, la calibración es
robusta. Si divergen mucho, indica problemas de diversidad de poses.
"""
import cv2
import json
import numpy as np
from pathlib import Path

SCRIPT_DIR  = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent
SAMPLES_DIR = REPO_ROOT / 'calibration_data' / 'samples'
RESULT_DIR  = REPO_ROOT / 'calibration_data' / 'results'

BASE_FRAME   = 'ur_dual_I_base_link'
CAMERA_FRAME = 'oak_rgb_camera_optical_frame'


def quat_to_R(q_xyzw):
    """Cuaternión (x,y,z,w) → matriz rotación 3x3."""
    x, y, z, w = q_xyzw
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def invert_rt(R, t):
    """Inversa de transformación rígida."""
    Ri = R.T
    return Ri, -Ri @ t


def R_to_rpy_deg(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0
    return np.degrees([rx, ry, rz])


def R_to_quat(R):
    """Matriz rotación 3x3 → cuaternión xyzw."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2 * np.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]


def load_samples():
    files = sorted(SAMPLES_DIR.glob('sample_*.json'))
    if not files:
        raise RuntimeError(f'No hay muestras en {SAMPLES_DIR}')

    R_base2grip, t_base2grip = [], []   # INVERTIDO (truco eye-to-hand)
    R_targ2cam, t_targ2cam = [], []

    for f in files:
        d = json.load(open(f))
        # Flange en base (desde TF en la captura)
        R_g2b = quat_to_R(d['flange']['quaternion_xyzw'])
        t_g2b = np.array(d['flange']['translation'], dtype=np.float64).reshape(3, 1)
        # Invertir para el truco eye-to-hand
        R_b2g, t_b2g = invert_rt(R_g2b, t_g2b)
        R_base2grip.append(R_b2g)
        t_base2grip.append(t_b2g)
        # Marcador en cámara (solvePnP directo desde la captura)
        rvec = np.array(d['marker']['rvec'], dtype=np.float64)
        tvec = np.array(d['marker']['tvec'], dtype=np.float64).reshape(3, 1)
        R_t2c, _ = cv2.Rodrigues(rvec)
        R_targ2cam.append(R_t2c)
        t_targ2cam.append(tvec)

    return files, R_base2grip, t_base2grip, R_targ2cam, t_targ2cam


METHODS = [
    ('TSAI',       cv2.CALIB_HAND_EYE_TSAI),
    ('PARK',       cv2.CALIB_HAND_EYE_PARK),
    ('HORAUD',     cv2.CALIB_HAND_EYE_HORAUD),
    ('DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS),
]


def main():
    files, R_b2g, t_b2g, R_t2c, t_t2c = load_samples()
    n = len(files)
    print(f'Muestras cargadas: {n}\n')

    results = {}
    print(f'{"Método":<12} {"tx (m)":>9} {"ty (m)":>9} {"tz (m)":>9}   '
          f'{"roll (°)":>9} {"pitch (°)":>9} {"yaw (°)":>9}')
    print('-' * 78)

    for name, method in METHODS:
        R_c2b, t_c2b = cv2.calibrateHandEye(
            R_b2g, t_b2g, R_t2c, t_t2c, method=method)
        t_flat = t_c2b.ravel()
        rpy = R_to_rpy_deg(R_c2b)
        print(f'{name:<12} {t_flat[0]:>9.4f} {t_flat[1]:>9.4f} {t_flat[2]:>9.4f}   '
              f'{rpy[0]:>9.2f} {rpy[1]:>9.2f} {rpy[2]:>9.2f}')
        results[name] = {
            'R_cam2base': R_c2b.tolist(),
            't_cam2base': t_flat.tolist(),
            'rpy_deg':    rpy.tolist(),
            'quaternion_xyzw': R_to_quat(R_c2b),
        }

    # Consistencia traslacional entre métodos
    ts = np.array([results[m[0]]['t_cam2base'] for m in METHODS])
    pairwise = np.linalg.norm(ts[:, None, :] - ts[None, :, :], axis=2)
    max_diff_mm = float(np.max(pairwise) * 1000)
    print(f'\nMax diferencia traslacional entre métodos: {max_diff_mm:.2f} mm')
    print('  < 10 mm  → muy robusta')
    print('  < 30 mm  → aceptable')
    print('  > 50 mm  → revisar diversidad de poses')

    # Guardar
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    out = RESULT_DIR / 'handeye_results.json'
    with open(out, 'w') as f:
        json.dump({
            'n_samples':            n,
            'base_frame':           BASE_FRAME,
            'camera_frame':         CAMERA_FRAME,
            'convention':           'T_camera_in_base (eye-to-hand, OpenCV con inversión gripper2base)',
            'max_method_diff_mm':   max_diff_mm,
            'methods':              results,
        }, f, indent=2)

    print(f'\n✓ Guardado: {out}')


if __name__ == '__main__':
    main()
