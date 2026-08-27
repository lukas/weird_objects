# Agent conventions — weird_objects

## Python commands: use uv

For all local project Python commands, use `uv` instead of bare
`python`, `python3`, or direct `.venv/bin/python` paths.

- Prefer `uv run python ...` for scripts and `uv run python -m ...`
  for modules.
- Prefer `uv run pytest ...` or `uv run python -m pytest ...` for
  tests.
- Use `uv pip ...` / `uv venv ...` for dependency and environment
  work.
- Do not rewrite historical logs, generated run records, vendored code,
  or shebangs just to say `uv`. Native MuJoCo GUI/viewer launches on
  macOS are the named exception: use `uv run mjpython ...` or the
  repo's Makefile wrapper, because Cocoa needs `mjpython`.
- The Uno Q also uses uv. Its web service should launch as
  `/home/arduino/.local/bin/uv run python ...`.

## BuildViz: two-port convention (5183 central, 5173 dev)

BuildViz uses exactly **two** fixed ports. Never start a server on any other
(random) port.

- **`5183` = the shared central hub — the ONE instance everyone uses.** It
  serves *every* project's builds at once; select one with `?build=<id>`. All
  project builds register into and are viewed from `5183`.
- **`5173` = reserved for BuildViz's own dev/testing** (`npm run dev`). It is a
  local dev server, never the hub. **Leave it alone** — do not view project
  builds on it, do not register into it, do not kill it. (On this machine it is
  kept alive by the `com.lbiewald.buildviz` LaunchAgent — don't touch that.)

Rules:

- **View** any build at `http://127.0.0.1:5183/?build=<id>`
  (e.g. `http://127.0.0.1:5183/?build=hexapod-prototype`).
- **Start / ensure** the central hub with the one canonical command
  (idempotent):

  ```sh
  npx buildviz hub --detach          # single hub on :5183; no-op if already up
  npx buildviz hub status            # is it up? url / pid / build count
  ```

- **Do NOT** start a new dev server on a random port to view a build — no
  `npm run dev -- --port 5199`, no `npx buildviz --port 5174`, no auto-picked
  Vite port. Those are the port sprawl this rule prevents. Do **not** touch
  `5173`. `register`/`push` auto-start the `5183` hub, so the hub is the only
  thing you ever launch.
- **Expose** a project's build by REGISTERING (or pushing) it into the hub:

  ```sh
  npx buildviz register <build-dir> --project <project> --build <build>
  # or send a scene.json layout straight to the hub:
  npx buildviz push --project <project> --build <build> --version main --scene scene.json
  ```

  The per-project `make view-buildviz` targets already do this and open the hub
  URL — prefer them.

- **Verify** the hub before trusting a server: read `~/.buildviz/server.json`
 and confirm `GET http://127.0.0.1:5183/__buildviz/status` returns
 `{ "service": "buildviz-hub" }`. A plain dev server also answers
 `/builds/index.json`, so that alone never proves it is the hub.
- **Mirror to the CLOUD hub** (standing convention, Lukas, Aug 2026): after
 publishing a new default version to the local hub, ALSO push it to the
 CoreWeave-hosted hub at
 `https://buildviz.cwd1f0-new-cluster.coreweave.app` so it is viewable off
 this machine. For the hexapod this is automatic — the sts3215
 `make verify-buildviz` target ends with a failure-tolerant cloud mirror
 step (standalone: `make -C hexapod_walker/prototype_sts3215 push-cloud`,
 script: `tools/push_cloud_buildviz.py`). Auth: the remote hub requires
 `X-API-Key` = `BUILDVIZ_API_KEY` (same key as the local hub; canonical
 source is the CoreWeave k8s secret `buildviz-api-key`:
 `KUBECONFIG=~/.kube/coreweave.yaml kubectl get secret buildviz-api-key -o
 jsonpath='{.data.key}' | base64 -d`). A dead network must never fail the
 local publish — mirror later with `push-cloud`.

Reference: `~/buildviz/README.md` ("How to run BuildViz") and
`~/buildviz/BUILDVIZ_LLM_INTERFACE.md`.

### Current build ids in the hub

`hexapod-prototype` (prototype_v1, animated gait), `prototype_sts3215`
(full robot; motion baked into its single scene.json — the separate
`prototype_sts3215_motion` build id was retired),
`prototype_ak40` (AK40 QDD hexapod, Design B, joints + stance poses; FLAT
build id — register with `--build-id prototype_ak40`, never `--project` +
`--build`, which creates a two-segment id the `?build=` viewer param can't
resolve; regenerate via `make -C hexapod_walker/prototype_ak40 view-buildviz`),
`prototype_v1/chassis`, `prototype_v1/leg`, `prototype_v1/leg/coxa`,
`rideable_v1`, `robot-cat`, plus older collision/demo builds. List them live
with `npx buildviz hub status` or open `http://127.0.0.1:5183/`.

## Out of scope for agents (unless the user explicitly asks)

- Do not touch the physical robot, SSH into it, or flash firmware.
- Do not modify firmware `.ino` files or CAD geometry as a side effect of an
  unrelated task.

## Hexapod STS3215 (`prototype_sts3215`) — hardware

**2026-08-06 incident:** agents drove stand/plant with wrong logical zeros
(straight-out legs already read knee ≈ −80°). That caused tip/brownout,
~7 A stilt holds, and a cooked knee servo. Hardware is FULLY RESOLVED
(servo replaced 2026-08-09, bus verified 18/18 healthy — do NOT
resurface it as an open issue); the process lessons are what remain.
Hard rules:

1. **No motion** unless the user explicitly asks in the current turn.
2. **HTTP over SSH** for control (`:8080` `/api/*`, `/cmd`). SSH = deploy
   only when asked.
   For robot-control/web edits, use the documented fast loop:
   `make -C hexapod_walker/prototype_sts3215 robot-check`,
   `robot-unit-check`, `robot-status`, and `robot-deploy`
   (`linux_control/dev_loop.sh`). These helpers do not move the robot;
   `robot-deploy` only restarts the web service. If `hexapod.local` is
   flaky, use `make ... robot-resolve` and pass the temporary IP via
   `HEXAPOD_HOST`/`HEXAPOD_SSH` instead of hard-coding it.
3. **Set-zero-here before absolute poses.** If encoders disagree with the
   photo, remap zero — do not command software 0°/stand/plant.
4. **Basic controls first — not standing.** Dead IDs, zeros, single-joint
   air moves, predict↔encoder. Do not push stand/plant/balance until those
   are solid and the operator asks.
5. **No unsupervised stand-up / plant blends.** Hip0+knee80 is stilts, not
   a low plant.
6. **Stop after tip, brownout, hot motor, or missing servo ID.** Do not retry
   blends. Limp and wait for the operator.

Details: `.cursor/rules/hexapod-sts-hardware-safety.mdc`,
`hexapod_walker/prototype_sts3215/rl_move/API.md`.

### Local Mac web hub on `:8898`

Canonical command:

```sh
cd ~/weird_objects/hexapod_walker/prototype_sts3215
make web-8898-start       # http://localhost:8898/rl
make web-8898-status
make web-8898-restart
make web-8898-stop
```

This starts a Mac-side `launchctl` job via `uv run python -m
rl_move.sim.web_server`; it is not the Uno Q's `hexapod-web.service`
(`:8080`). The launcher is
`hexapod_walker/prototype_sts3215/sim_viewer/hexapod_web_8898.sh`.
It resolves the robot's current IP unless `HEXAPOD_HOST` is set.
