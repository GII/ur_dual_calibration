#!/usr/bin/env python3
"""
Archiva la calibración actual antes de iniciar una nueva.

Mueve samples/, results/ y overlays/ a calibration_data/archive/calibration_YYYY-MM-DD_HH-MM/
sin tocar el handeye.yaml en uso (ese se actualiza al final de la nueva sesión).

Uso:
  python3 archive_calibration.py
"""
import shutil
import sys
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT  = SCRIPT_DIR.parent
CAL_DIR    = REPO_ROOT / 'calibration_data'

ARCHIVE_ROOT = CAL_DIR / 'archive'
TO_ARCHIVE = ['samples', 'results', 'overlays']


def main():
    if not CAL_DIR.exists():
        sys.exit(f'No existe {CAL_DIR}. Nada que archivar.')

    has_data = any((CAL_DIR / d).exists() and any((CAL_DIR / d).iterdir())
                   for d in TO_ARCHIVE)
    if not has_data:
        print('No hay datos previos que archivar. Carpetas vacías o inexistentes.')
        return

    stamp = datetime.now().strftime('%Y-%m-%d_%H-%M')
    dest = ARCHIVE_ROOT / f'calibration_{stamp}'
    dest.mkdir(parents=True, exist_ok=False)

    print(f'Archivando en: {dest}\n')
    for sub in TO_ARCHIVE:
        src = CAL_DIR / sub
        if not src.exists():
            continue
        target = dest / sub
        shutil.move(str(src), str(target))
        n = sum(1 for _ in target.rglob('*') if _.is_file())
        print(f'  ✓ {sub}/ → {target.name}/{sub}/  ({n} archivos)')
        # Recrear vacía para la nueva sesión
        src.mkdir(parents=True, exist_ok=True)

    print(f'\nListo. Carpetas {", ".join(TO_ARCHIVE)} vacías para la nueva sesión.')


if __name__ == '__main__':
    main()
