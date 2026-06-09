#!/usr/bin/env python3
"""
extract_poses_from_samples.py

Genera calibration_poses.yaml a partir de los sample_XXX.json de la sesión
actual de calibración. Se corre UNA vez tras una recalibración manual
exitosa. A partir de entonces, auto_calibrate.py puede replicar esa
secuencia de poses automáticamente.

Las poses solo se invalidan si cambia la geometría del montaje del marcador
(p.ej. desmontas la pieza 3D y la pones en otro sitio). El movimiento del
trípode NO las invalida, solo cambia qué poses ven el marcador.

Uso:
  python3 extract_poses_from_samples.py
"""
import json
from pathlib import Path

SCRIPT_DIR  = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent
SAMPLES_DIR = REPO_ROOT / 'calibration_data' / 'samples'
OUT_PATH    = REPO_ROOT / 'src' / 'ur_dual_calibration' / 'config' / 'calibration_poses.yaml'

# Orden estándar UR5 que el controller espera en JointTrajectory.joint_names
JOINT_ORDER = [
    'ur_dual_I_shoulder_pan_joint',
    'ur_dual_I_shoulder_lift_joint',
    'ur_dual_I_elbow_joint',
    'ur_dual_I_wrist_1_joint',
    'ur_dual_I_wrist_2_joint',
    'ur_dual_I_wrist_3_joint',
]


def main():
    files = sorted(SAMPLES_DIR.glob('sample_*.json'))
    if not files:
        raise SystemExit(f'No hay muestras en {SAMPLES_DIR}. '
                         f'Calibra manualmente primero.')

    poses = []
    for f in files:
        d = json.load(open(f))
        joints = d['joints']
        try:
            ordered = [float(joints[j]) for j in JOINT_ORDER]
        except KeyError as e:
            raise SystemExit(f'{f.name} no contiene el joint {e}. '
                             f'¿Calibración de otro brazo?')
        poses.append({
            'id':              f'pose_{d["sample_id"]:03d}',
            'source_sample':   f.name,
            'reproj_error_px': round(float(d['marker']['reprojection_error_px']), 3),
            'distance_m':      round(float(d['marker']['distance_m']), 3),
            'joints':          [round(v, 6) for v in ordered],
        })

    # YAML escrito a mano para control total del formato
    out = []
    out.append('# Poses pregrabadas para calibración hand-eye automática')
    out.append('# Generado por extract_poses_from_samples.py')
    out.append(f'# Total: {len(poses)} poses extraídas de calibration_data/samples/')
    out.append('#')
    out.append('# IMPORTANTE: estas poses dependen del montaje del ArUco respecto al flange.')
    out.append('# Si cambia la pieza 3D del marcador o su orientación, hay que regenerarlas.')
    out.append('')
    out.append('metadata:')
    out.append('  arm_prefix:    "ur_dual_I_"')
    out.append('  controller:    "scaled_joint_trajectory_controller_I"')
    out.append('  action_topic:  "/scaled_joint_trajectory_controller_I/follow_joint_trajectory"')
    out.append('  joint_order:')
    for j in JOINT_ORDER:
        out.append(f'    - "{j}"')
    out.append('')
    out.append('poses:')
    for p in poses:
        out.append(f'  - id: "{p["id"]}"')
        out.append(f'    source_sample:   "{p["source_sample"]}"')
        out.append(f'    reproj_error_px: {p["reproj_error_px"]}')
        out.append(f'    distance_m:      {p["distance_m"]}')
        joints_str = ', '.join(f'{v:.6f}' for v in p['joints'])
        out.append(f'    joints: [{joints_str}]')

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text('\n'.join(out) + '\n')
    print(f'✓ Generado {OUT_PATH}')
    print(f'  {len(poses)} poses extraídas')


if __name__ == '__main__':
    main()
