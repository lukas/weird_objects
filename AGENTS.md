# Agent conventions — weird_objects

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

Reference: `~/buildviz/README.md` ("How to run BuildViz") and
`~/buildviz/BUILDVIZ_LLM_INTERFACE.md`.

### Current build ids in the hub

`hexapod-prototype` (prototype_v1, animated gait), `prototype_sts3215`
(full robot; motion baked into its single scene.json — the separate
`prototype_sts3215_motion` build id was retired), `prototype_v1/chassis`,
`prototype_v1/leg`, `prototype_v1/leg/coxa`, `rideable_v1`, `robot-cat`, plus
older collision/demo builds. List them live with `npx buildviz hub status` or
open `http://127.0.0.1:5183/`.

## Out of scope for agents (unless the user explicitly asks)

- Do not touch the physical robot, SSH into it, or flash firmware.
- Do not modify firmware `.ino` files or CAD geometry as a side effect of an
  unrelated task.
