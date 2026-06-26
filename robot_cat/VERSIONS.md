# Robot Cat — BuildViz Versioning

The robot cat keeps its revision history the way **BuildViz is actually designed
to**: **one build, many named versions** — *not* a separate hub build id per
revision. (Run `node /Users/lbiewald/buildviz/bin/buildviz.mjs --help` and read
the "Data model — PROJECT / BUILD / VERSION" and "Versions (named branches)"
sections.)

## The model

BuildViz's data model is **PROJECT → BUILD → NAMED VERSION**. For a single
evolving design you use **one build id** with several **named versions**; each
version lives on disk at `<build-dir>/versions/<name>/scene.json`, and the
**default** version is mirrored at the build root `scene.json`. `meta.json`
records the default + the version list.

The robot cat is therefore a **single hub build, `robot-cat`**, registered at
`robot_cat/cat_viz/`, with these versions:

| Version | Meaning | Served from | Viewer URL |
| --- | --- | --- | --- |
| **`main`** *(default)* | The **live working tip** — whatever `build_cat.py` last wrote. Overwritten in place as the design evolves. | `cat_viz/scene.json` (build root) | `…/?build=robot-cat` |
| **`v10`** | Frozen: **BABY KITTEN proportions** — pushes v8/v9 to peak neoteny: bigger domed head (head ≈ ½ body), bigger/rounder low-set eyes, a shorter round pot-belly body on shorter stubbier legs with oversized paws, softer round ears, and a shorter softly-curled tail. Also a **tail bugfix**: the old sharp-hook loft self-intersected at the tip (looked detached); now a Catmull-Rom-smoothed dense continuous curl rooted in the rump (base gap 0.04 mm). Keeps all v9 manufacturability (split shell, M2 bosses, mounts, 9-DOF). ~591 g, CoM (23, 0, 73). | `cat_viz/versions/v10/` | `…/?build=robot-cat&version=v10` |
| **`v9`** | Frozen: **manufacturable** — same cute kitten, but the body is now a real screw-together print: sagittal `shell_left`+`shell_right` halves (0.2 mm seam), removable `belly_hatch`, separate head, M2 self-tap seam/hatch bosses + alignment lip, integral servo/board/battery mounts; every part tagged MOVING vs STATIC (9-DOF kinematics) with an exploded render. | `cat_viz/versions/v9/` | `…/?build=robot-cat&version=v9` |
| **`v8`** | Frozen: **cuteness redesign** — a compact ~Nybble-class neotenic kitten (~250 mm, ~570 g) on ~10 g **micro servos** + a small ESP32 board; big round head held up with large forward eyes, chubby rounded body, short stubby legs with hidden knee actuators, collar + bell, electronics hidden in the belly. | `cat_viz/versions/v8/` | `…/?build=robot-cat&version=v8` |
| **`v7`** | Frozen: aesthetic overhaul of the full-size cat — sculpted continuous body, teardrop haunch/shoulder muscles, tapered limbs, clean knee/elbow housings (still 60 g STS3215). | `cat_viz/versions/v7/` | `…/?build=robot-cat&version=v7` |
| **`v6`** | Frozen: full-size adult cat (~490 mm, ~1.6 kg); drive returns to FEETECH STS3215; hip/shoulder servos inboard, hidden under printed body shrouds (spheres). | `cat_viz/versions/v6/` | `…/?build=robot-cat&version=v6` |
| **`v5`** | Frozen: 9-servo STS3032 walker, 2 DOF/leg, ~0.72× small cat (the design when versioning began). | `cat_viz/versions/v5/` | `…/?build=robot-cat&version=v5` |

The hub runs at `http://127.0.0.1:5183` (registry `~/.buildviz/registry.json`).
On serve it rewrites each version's relative `stl/…` mesh URL to
`/builds/robot-cat/versions/<name>/stl/…`, so every version serves its own
assets. The version menu and the `diff`/`compare` view are driven by this one
build's version list — no duplicate hub registrations.

> **Why `main` currently matches `v10`.** `main` is the live tip; right after a
> revision is frozen, the tip equals the most recent frozen version. They diverge
> again the next time `build_cat.py` runs with new geometry. This mirrors how
> `buildviz push --set-default` keeps the default mirrored at the build root.

## On-disk layout

```
robot_cat/cat_viz/                 # hub build id: robot-cat
├── meta.json                      # BuildViz project/build/version record
├── scene.json  design_spec.yaml  stl/      # the DEFAULT version "main" (live tip)
└── versions/
    ├── v5/  { scene.json, design_spec.yaml, stl/ }   # frozen
    ├── v6/  { scene.json, design_spec.yaml, stl/ }   # frozen
    ├── v7/  { scene.json, design_spec.yaml, stl/ }   # frozen
    ├── v8/  { scene.json, design_spec.yaml, stl/ }   # frozen
    ├── v9/  { scene.json, design_spec.yaml, stl/ }   # frozen
    └── v10/ { scene.json, design_spec.yaml, stl/ }   # frozen
```

`meta.json` follows the exact schema `buildviz push` / `buildviz migrate` write
(`{ schema, buildId, defaultVersion, versions:[{name,pushedAt}], … }`).

## Inspecting / diffing versions

These are read commands; address the build **by directory path** (see the note
below on why a bare id doesn't resolve for a `register`-based build):

```sh
BV=/Users/lbiewald/buildviz/bin/buildviz.mjs
node $BV versions robot_cat/cat_viz --json          # lists main, v5, v6, v7, v8, v9, v10
node $BV validate "robot_cat/cat_viz@v10" --json     # ok: true
node $BV diff robot_cat/cat_viz v9 v10 --json        # what changed v9 → v10
```

The **hub HTTP** addresses the same build by its id (`robot-cat`); only the local
CLI read commands need the path.

## Cutting a new version going forward

Versioning is built into `build_cat.py` using BuildViz's native layout — no hub
build ids are minted.

```sh
# 1. Edit the design (bump REV in build_cat.py if it's a new revision), then
#    rebuild the LIVE tip (the "main" version at the build root):
./run.sh robot_cat/build_cat.py

# 2. When the revision is ready, FREEZE it as an immutable named version
#    cat_viz/versions/v<REV>/ and record it in meta.json:
./run.sh robot_cat/build_cat.py --freeze            # default name = v<REV>
#    (or: --freeze v7   to pick a name;   --force   to replace an existing one)
```

`--freeze` copies the live `cat_viz/` (`scene.json` + `design_spec.yaml` + `stl/`)
into `cat_viz/versions/<name>/` and updates `meta.json`. It is the **local,
binary-carrying equivalent of `buildviz push --version <name> --set-default`**.
Frozen versions are immutable (it refuses to clobber one without `--force`).

The hub serves the new version **live** (it enumerates the on-disk `versions/`
dir per request); if it doesn't appear yet, run
`node /Users/lbiewald/buildviz/bin/buildviz.mjs hub restart`.

### One-time hub registration

The single build is registered once; you only need to repeat this if the registry
is reset:

```sh
node /Users/lbiewald/buildviz/bin/buildviz.mjs register \
  /Users/lbiewald/weird_objects/robot_cat/cat_viz \
  --build-id robot-cat --name "Robot Cat (cat-like quadruped)"
```

## Why this replaced the old scheme

The earlier home-grown scheme registered a **separate hub build id**
(`robot-cat-v5`, `robot-cat-v6`) per revision and copied `cat_viz/` into
`robot_cat/versions/v<N>_cat_viz/` outside the build dir. That invented a
convention BuildViz doesn't use and cluttered the hub with parallel build ids.
The cleanup collapsed those into the **one** `robot-cat` build with native
versions; the redundant `robot-cat-v5` / `robot-cat-v6` registrations were pruned
from the hub (by removing their source dirs and restarting the hub — BuildViz's
prune-on-restart is its unregister mechanism).

BuildViz lacks a first-class CLI command to *create* this on-disk version layout
for a `register`-based local-asset build (its `push` writes only the hub cache and
can't carry STL bytes). That gap — and a small proposed `buildviz freeze` command
— is written up in
[`../../buildviz/BUILDVIZ_ONDISK_VERSIONING_NOTE.md`](../../buildviz/BUILDVIZ_ONDISK_VERSIONING_NOTE.md).
`build_cat.py --freeze` is the ~40-line local stand-in for it.
