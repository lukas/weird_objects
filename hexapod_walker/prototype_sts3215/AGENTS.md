# Agent conventions — prototype_sts3215

## Python commands: use uv

Use `uv` for local Python work in this project. Do not run bare
`python3`, bare `python`, or direct `.venv/bin/python` paths for normal
development commands.

- Scripts: `uv run python path/to/script.py`
- Modules: `uv run python -m package.module`
- Tests: `uv run pytest ...` or `uv run python -m pytest ...`
- Dependencies and venvs: `uv pip ...` and `uv venv ...`

Exceptions are narrow: historical logs/generated run records, vendored
code, and shebangs. Native MuJoCo GUI/viewer launches on macOS are the
special live exception: use `uv run mjpython ...` or the Makefile
viewer wrapper, because Cocoa needs `mjpython`.
The Uno Q is not an exception: its service/deploy path should use
`/home/arduino/.local/bin/uv run python ...`.

## Local web hub on :8898

The canonical Mac-side browser/control surface is:

```sh
cd ~/weird_objects/hexapod_walker/prototype_sts3215
make web-8898-start       # serves http://localhost:8898/rl
make web-8898-status
make web-8898-restart
make web-8898-stop
```

This is a local Mac `launchctl` job that runs
`uv run python -m rl_move.sim.web_server`; it is not an on-robot service.
The script is `sim_viewer/hexapod_web_8898.sh`. It resolves the current
robot IP for the `:8080` target unless `HEXAPOD_HOST` is set. Use this
instead of direct `.venv/bin/python`, ad hoc `nohup`, or a random worktree.

## Robot safety

No physical robot motion unless the user explicitly asks for it in the
current turn. Local checks, deploys, and service restarts are okay when
requested; motion endpoints are not.
