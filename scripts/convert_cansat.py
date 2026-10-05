"""Convert the static Fusion CAD FBX to a drivable USD without starting Kit."""

import argparse
import importlib.util
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def ensure_usd():
    """Use Isaac Sim's bundled USD Python bindings outside SimulationApp."""
    if importlib.util.find_spec("pxr") is not None:
        return
    cache = Path(sys.prefix) / "lib/python3.11/site-packages/isaacsim/extscache"
    candidates = sorted(cache.glob("omni.usd.libs-*/pxr"))
    if not candidates:
        raise RuntimeError("Run with Isaac Sim's Python environment (pixi run -e default python).")
    library = candidates[-1].parent
    os.environ["PYTHONPATH"] = os.pathsep.join([str(library), str(ROOT), os.environ.get("PYTHONPATH", "")])
    os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
        [str(Path(sys.prefix) / "lib"), str(library / "bin"), os.environ.get("LD_LIBRARY_PATH", "")]
    )
    os.execv(sys.executable, [sys.executable, *sys.argv])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "assets/cansat.fbx")
    parser.add_argument("--settings", type=Path, default=ROOT / "cfg/robot/cansat_physics.json")
    parser.add_argument("--output", type=Path, default=ROOT / "assets/USD_Assets/robots/cansat/cansat.usda")
    args = parser.parse_args()
    ensure_usd()
    sys.path.insert(0, str(ROOT))
    from src.robots.cansat_asset import build_asset

    build_asset(args.source, args.settings, args.output)


if __name__ == "__main__":
    main()
