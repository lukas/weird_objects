#!/usr/bin/env python3
"""Emit + publish the ANIMATED hexapod BuildViz build (motion API demo).

This is the motion sibling of ``full_robot_viz_build.py``.  It is purely
ADDITIVE: it never touches the static ``full_robot_viz/scene.json`` or any part
geometry.  It re-runs the verified standing-pose build and then writes a SECOND
scene, ``full_robot_viz/scene_motion.json``, that carries BuildViz's optional
``joints[]`` + ``poses[]`` motion blocks on top of the identical meshes +
home-pose transforms.

What the motion blocks unlock in BuildViz (see /Users/lbiewald/buildviz
BUILDVIZ_INTEGRATION.md "Motion / kinematics" + src/MotionPanel.tsx):
  * per-joint sliders for all 18 leg DOFs (6 legs x yaw/hip/knee),
  * an "Animate sweep" button that drives every DOF min->max->min (the legs
    visibly flex/extend in a continuous loop), driven by the REAL joint axes,
  * clickable named poses (a tripod-gait march + stance variants),
  * swept-pose overlap validation (``buildviz sweep``).

The yaw/hip/knee axes + pivots come straight from the authoritative leg-0
kinematic chain the static build / verifier use, mapped per-leg, so a slider
rotates about the true joint axis through the true pivot.

Run (from repo root):
    ./run.sh hexapod_walker/prototype_sts3215/tools/full_robot_viz_motion_build.py
    # then open the published build, or locally:
    #   cd hexapod_walker/prototype_sts3215 && npx buildviz full_robot_viz \
    #       --scene scene_motion.json --port 5174

Flags:
    --single      build only leg 0 (fast iteration).
    --no-publish  write scene_motion.json but skip the BuildViz hub push.
    --keep N      retained-version cap for the hub push (default 20).
"""

from __future__ import annotations

import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

import full_robot_viz_build as F  # noqa: E402
import buildviz_checks as BC  # noqa: E402

# Published under a DISTINCT build id so the animated demo never disturbs the
# static ``prototype_sts3215`` build's version history (the static build keeps
# being bumped by ``make verify-buildviz``).
MOTION_BUILD_ID = "prototype_sts3215_motion"


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    single = "--single" in argv
    publish = "--no-publish" not in argv
    keep = 20
    if "--keep" in argv:
        keep = int(argv[argv.index("--keep") + 1])

    # Generate the static scene + the additive motion sibling.
    F.main(single_leg=single, motion=True)

    motion_scene = F.OUT_DIR / "scene_motion.json"
    if not motion_scene.is_file():
        print(f"ERROR: {motion_scene} was not written")
        return 1

    if not publish:
        print(f"[motion] wrote {motion_scene} (publish skipped)")
        return 0

    print(f"[motion] publishing {motion_scene} as build "
          f"'{MOTION_BUILD_ID}' (buildviz push --bump)")
    code = BC.publish_to_buildviz(
        str(motion_scene), build_id=MOTION_BUILD_ID, keep=keep)
    if code != 0:
        print(f"[motion] publish returned {code} (scene_motion.json still "
              "written; see message above)")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
