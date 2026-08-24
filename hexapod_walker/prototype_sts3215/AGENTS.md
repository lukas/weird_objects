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

## Robot safety

No physical robot motion unless the user explicitly asks for it in the
current turn. Local checks, deploys, and service restarts are okay when
requested; motion endpoints are not.
