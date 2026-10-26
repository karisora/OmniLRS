#!/usr/bin/env python3
"""Remove Lumos and recreate it at the terrain-aware pose saved by the launch."""

import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('build/gazebo_site20'))
    args = parser.parse_args()
    pose = json.loads((args.output/'spawn.json').read_text())
    result = subprocess.run([
        'gz', 'service', '-s', '/world/site20/remove', '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean', '--timeout', '10000',
        '--req', 'name: "lumos", type: MODEL'], capture_output=True, text=True, check=True)
    if 'data: true' not in result.stdout:
        raise SystemExit(f'Could not remove lumos; verify that the Site20 simulation is running. {result.stdout} {result.stderr}')
    subprocess.run([
        'ros2', 'run', 'ros_gz_sim', 'create', '-world', 'site20', '-file', pose['urdf'],
        '-name', 'lumos', '-allow_renaming', 'false', '-x', str(pose['x']),
        '-y', str(pose['y']), '-z', str(pose['z']), '-Y', str(pose['yaw'])], check=True)


if __name__ == '__main__':
    main()
