#!/usr/bin/env python3
"""
Validación de la calibración hand-eye por auto-consistencia.

Para cada muestra, estima T_marker_in_flange. Si la calibración es correcta,
debe ser casi idéntica en las 20 muestras (porque el marcador no se movió
respecto al flange). La desviación estándar de las traslaciones es la
métrica primaria de validación (Strobl & Hirzinger, 2006).

Además: reproyecta el marcador sobre cada imagen capturada y guarda las
imágenes con overlays para inspección visual.
"""
import cv2
import json
import numpy as np
from pathlib import Path

SCRIPT_DIR  = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent
SAMPLES_DIR = REPO_ROOT / 'calibration_data' / 'samples'
RESULT_DIR  = REPO_ROOT / 'calibration_data' / 'results'
OVERLAY_DIR = REPO_ROOT / 'calibration_data' / 'overlays'

METHOD_TO_USE = 'PARK'   # elegido por consistencia y robustez (Park & Martin 1994)
MARKER_LENGTH = 0.0427

L = MARKER_LENGTH / 2.0
OBJECT_POINTS = np.array([
    [-L,  L, 0],
    [ L,  L, 0],
    [ L, -L, 0],
    [-L, -L, 0],
], dtype=np.float32)


def quat_to_R(q):
    x, y, z, w = q
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).ravel()
    return T


def rot_angle_deg(R):
    """Ángulo de rotación (norma del axis-angle) en grados."""
    v = (np.trace(R) - 1.0) / 2.0
    v = np.clip(v, -1.0, 1.0)
    return float(np.degrees(np.arccos(v)))


def main():
    # Cargar resultado
    results = json.load(open(RESULT_DIR / 'handeye_results.json'))
    R_c2b = np.array(results['methods'][METHOD_TO_USE]['R_cam2base'])
    t_c2b = np.array(results['methods'][METHOD_TO_USE]['t_cam2base'])
    T_cam_in_base = make_T(R_c2b, t_c2b)

    # Cargar todas las muestras
    files = sorted(SAMPLES_DIR.glob('sample_*.json'))
    T_marker_in_flange = []

    for f in files:
        d = json.load(open(f))

        # T_flange_in_base
        R_fb = quat_to_R(d['flange']['quaternion_xyzw'])
        t_fb = np.array(d['flange']['translation'])
        T_flange_in_base = make_T(R_fb, t_fb)

        # T_marker_in_camera
        R_mc, _ = cv2.Rodrigues(np.array(d['marker']['rvec']))
        t_mc = np.array(d['marker']['tvec'])
        T_marker_in_cam = make_T(R_mc, t_mc)

        # T_marker_in_flange = inv(T_flange_in_base) @ T_cam_in_base @ T_marker_in_cam
        T_mf = np.linalg.inv(T_flange_in_base) @ T_cam_in_base @ T_marker_in_cam
        T_marker_in_flange.append(T_mf)

    # Estadísticos sobre traslaciones y rotaciones
    ts = np.array([T[:3, 3] for T in T_marker_in_flange])
    mean_t = ts.mean(axis=0)
    std_t  = ts.std(axis=0) * 1000   # a mm
    max_dev_mm = float(np.linalg.norm(ts - mean_t, axis=1).max() * 1000)

    # Rotación: ángulo respecto al promedio de las rotaciones
    R_mean = T_marker_in_flange[0][:3, :3].copy()  # rotación "ancla"
    rot_devs = [rot_angle_deg(R_mean.T @ T[:3, :3]) for T in T_marker_in_flange]

    print('─' * 70)
    print(f'VALIDACIÓN DE CALIBRACIÓN — método: {METHOD_TO_USE}')
    print('─' * 70)
    print(f'Muestras: {len(files)}')
    print(f'\nT_marker_in_flange — translación media [m]: '
          f'[{mean_t[0]:+.4f}, {mean_t[1]:+.4f}, {mean_t[2]:+.4f}]')
    print(f'Desviación estándar por eje [mm]: '
          f'σx={std_t[0]:.2f}  σy={std_t[1]:.2f}  σz={std_t[2]:.2f}')
    print(f'Desviación 3D máxima respecto a la media: {max_dev_mm:.2f} mm')
    print(f'Desviación rotacional máx respecto a muestra 1: '
          f'{max(rot_devs):.2f}°')
    print('\nInterpretación:')
    print('  σ 3D máx < 5 mm  y  rot < 1°  → calibración EXCELENTE')
    print('  σ 3D máx < 15 mm y  rot < 3°  → calibración ACEPTABLE')
    print('  σ 3D máx > 30 mm  o  rot > 5° → revisar (malas poses o bug)')

    # Overlays visuales: reproyectar marcador en cada imagen
    OVERLAY_DIR.mkdir(parents=True, exist_ok=True)
    print(f'\nGenerando overlays en {OVERLAY_DIR}...')

    for i, f in enumerate(files):
        d = json.load(open(f))
        png_path = SAMPLES_DIR / d['image_file']
        img = cv2.imread(str(png_path))
        if img is None:
            continue

        K = np.array(d['camera']['K'])
        D = np.array(d['camera']['D'])

        # Reproyectar las 4 esquinas usando la pose observada (sanity)
        rvec = np.array(d['marker']['rvec'])
        tvec = np.array(d['marker']['tvec'])
        proj, _ = cv2.projectPoints(OBJECT_POINTS, rvec, tvec, K, D)
        proj = proj.reshape(-1, 2).astype(int)

        # Detectadas (verde) y reproyectadas (rojo); deben coincidir si la
        # detección es buena (no es test de hand-eye, es test de solvePnP).
        detected = np.array(d['marker']['detected_corners_px'], dtype=int)
        for p in detected:
            cv2.circle(img, tuple(p), 6, (0, 255, 0), 2)
        for p in proj:
            cv2.drawMarker(img, tuple(p), (0, 0, 255),
                           markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)

        # Dibujar axes del marcador
        cv2.drawFrameAxes(img, K, D, rvec, tvec, L)

        # Header
        txt = (f'#{d["sample_id"]:03d}  '
               f'd={d["marker"]["distance_m"]:.2f}m  '
               f'reproj={d["marker"]["reprojection_error_px"]:.2f}px')
        cv2.putText(img, txt, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)

        cv2.imwrite(str(OVERLAY_DIR / f'overlay_{d["sample_id"]:03d}.png'), img)

    print(f'✓ {len(files)} overlays generados')

    # Guardar métricas
    with open(RESULT_DIR / 'validation_metrics.json', 'w') as f:
        json.dump({
            'method_validated':        METHOD_TO_USE,
            'n_samples':               len(files),
            'marker_in_flange_mean_m': mean_t.tolist(),
            'std_per_axis_mm':         std_t.tolist(),
            'max_translational_dev_mm': max_dev_mm,
            'max_rotational_dev_deg':   float(max(rot_devs)),
        }, f, indent=2)

    print(f'✓ Guardado: {RESULT_DIR / "validation_metrics.json"}')


if __name__ == '__main__':
    main()
