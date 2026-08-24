#!/usr/bin/env python3
"""Interactive viewer for the mesh-accurate hexapod model.

macOS needs mjpython for the MuJoCo viewer -- use ./view_mesh.sh, which
resolves the repo venv's mjpython and builds the model first if needed.

Usage: view_mesh.py [plant|stance]   (default: plant, the standing pose)

The position servos hold the chosen keyframe's targets, so the robot
actively stands (plant) or belly-rests (CAD display stance).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

HERE = Path(__file__).resolve().parent
XML = HERE / "hexapod_mesh.xml"


def main():
    key = sys.argv[1] if len(sys.argv) > 1 else "plant"
    if not XML.is_file():
        sys.exit(f"{XML.name} missing -- run build_mesh_model.py first")
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    k = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key)
    if k < 0:
        sys.exit(f"unknown keyframe {key!r} (have: plant, stance)")
    mujoco.mj_resetDataKeyframe(model, data, k)
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            t0 = time.time()
            mujoco.mj_step(model, data)
            v.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - t0)))


if __name__ == "__main__":
    main()
