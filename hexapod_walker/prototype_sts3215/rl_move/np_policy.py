"""Portable RL policies as data: validate, upload, run anywhere.

The robot already runs plain-JSON numpy MLPs (export_policy_np.py
output — tanh(W1 x + b1) -> tanh(W2 h + b2) -> Wout h + bout, clipped
to [-1, 1]; no torch on the board).  This module makes that SAME file
an uploadable artifact: POST it to /api/rl/policies on a robot OR the
MuJoCo sim web session and it lands in ~/.hexapod_policies (outside
any deploy tree), shows up in the /api/rl/policies picker, and runs
through the existing /api/rl/* machinery — stand, walk, drive, roles.
Two robots given the same file walk with the same brain.

Artifact = the export_policy_np.py JSON:

    {"meta": {"name": "...", "notes": "...", "source": "...",
              "obs_dim": 68|72|74, "act_dim": 18,
              "hidden": [h1, h2], "activation": "tanh",
              "profile": {...},        # optional trained goal ramps
              "phase_hz": 0.1667},     # REQUIRED for obs 74
     "W1": [[...]], "b1": [...], "W2": [[...]], "b2": [...],
     "Wout": [[...]], "bout": [...]}

Validation is strict enough to make an upload safe to run: obs_dim
must fit a known slot (68 stance / 72 walk / 74 phase-walk), act_dim
18, all six matrices present with a consistent shape chain, every
value finite, and a smoke forward pass must return 18 finite actions.

CLI (mirrors dance_script.py):
    python -m rl_move.np_policy validate policies/foo.json
    python -m rl_move.np_policy push foo.json --host http://hexapod.local:8080
    python -m rl_move.np_policy pull foo --host http://robot-b.local:8080
"""
from __future__ import annotations

import json
import re
import urllib.request
from pathlib import Path

import numpy as np

UPLOAD_DIR = Path.home() / ".hexapod_policies"
MAX_POLICY_BYTES = 8_000_000
NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")
KNOWN_OBS = (68, 72, 74)
_MATS = ("W1", "b1", "W2", "b2", "Wout", "bout")


def validate_np_policy(obj) -> tuple[list[str], dict]:
    """Return (errors, info).  Empty errors == runnable policy."""
    errs: list[str] = []
    info: dict = {}
    if not isinstance(obj, dict):
        return (["policy must be a JSON object"], info)
    meta = obj.get("meta")
    if not isinstance(meta, dict):
        return (["missing meta object"], info)
    obs = meta.get("obs_dim")
    act = meta.get("act_dim")
    info = {"name": meta.get("name") or "", "obs_dim": obs,
            "act_dim": act, "source": meta.get("source", ""),
            "notes": meta.get("notes", "")}
    if obs not in KNOWN_OBS:
        errs.append(f"obs_dim {obs!r} fits no slot "
                    f"(68 stance / 72 walk / 74 phase-walk)")
    if act != 18:
        errs.append(f"act_dim must be 18, got {act!r}")
    if meta.get("activation", "tanh") != "tanh":
        errs.append("activation must be tanh (export_policy_np contract)")
    if obs == 74 and not meta.get("phase_hz"):
        errs.append("obs 74 (phase clock) requires meta.phase_hz")
    for k in _MATS:
        if k not in obj:
            errs.append(f"missing matrix {k}")
    if errs:
        return (errs, info)
    try:
        W1 = np.asarray(obj["W1"], dtype=np.float64)
        b1 = np.asarray(obj["b1"], dtype=np.float64)
        W2 = np.asarray(obj["W2"], dtype=np.float64)
        b2 = np.asarray(obj["b2"], dtype=np.float64)
        Wo = np.asarray(obj["Wout"], dtype=np.float64)
        bo = np.asarray(obj["bout"], dtype=np.float64)
    except (TypeError, ValueError) as e:
        return ([f"matrices are not numeric arrays: {e}"], info)
    chain = [("W1", W1, (b1.shape[0], obs)), ("b1", b1, (W1.shape[0],)),
             ("W2", W2, (b2.shape[0], W1.shape[0])),
             ("b2", b2, (W2.shape[0],)),
             ("Wout", Wo, (18, W2.shape[0])), ("bout", bo, (18,))]
    for name, m, want in chain:
        if m.shape != want:
            errs.append(f"{name} shape {m.shape} != {want}")
    for name, m, _ in chain:
        if not np.all(np.isfinite(m)):
            errs.append(f"{name} contains non-finite values")
            break
    if errs:
        return (errs, info)
    # Smoke forward pass — an uploaded brain must at least not explode.
    h = np.tanh(W1 @ np.zeros(obs) + b1)
    h = np.tanh(W2 @ h + b2)
    a = np.clip(Wo @ h + bo, -1.0, 1.0)
    if a.shape != (18,) or not np.all(np.isfinite(a)):
        errs.append("smoke forward pass failed")
    info["hidden"] = [int(W1.shape[0]), int(W2.shape[0])]
    return (errs, info)


class _Space:
    def __init__(self, n: int):
        self.shape = (n,)


class NumpyMLPModel:
    """SB3-shaped adapter around a numpy MLP policy.

    Exposes exactly the surface the sim web session uses on PPO
    checkpoints — .predict(obs, deterministic=True) -> (action, None),
    .observation_space.shape, .action_space.shape — so an uploaded
    JSON policy plugs into the same stance/walk/role slots.
    """

    def __init__(self, obj: dict, path: Path | None = None):
        self.meta = dict(obj["meta"])
        self.path = path
        self.W1 = np.asarray(obj["W1"], dtype=np.float64)
        self.b1 = np.asarray(obj["b1"], dtype=np.float64)
        self.W2 = np.asarray(obj["W2"], dtype=np.float64)
        self.b2 = np.asarray(obj["b2"], dtype=np.float64)
        self.Wo = np.asarray(obj["Wout"], dtype=np.float64)
        self.bo = np.asarray(obj["bout"], dtype=np.float64)
        self.observation_space = _Space(int(self.meta["obs_dim"]))
        self.action_space = _Space(int(self.meta.get("act_dim", 18)))
        self.hidden = [int(self.W1.shape[0]), int(self.W2.shape[0])]

    def act(self, obs: np.ndarray) -> np.ndarray:
        h = np.tanh(self.W1 @ obs + self.b1)
        h = np.tanh(self.W2 @ h + self.b2)
        return np.clip(self.Wo @ h + self.bo, -1.0, 1.0)

    def predict(self, obs, deterministic: bool = True, **_kw):
        return self.act(np.asarray(obs, dtype=np.float64)), None


def load_np_policy(path) -> NumpyMLPModel:
    obj = json.loads(Path(path).read_text())
    errs, _ = validate_np_policy(obj)
    if errs:
        raise ValueError(f"{Path(path).name}: " + "; ".join(errs[:3]))
    return NumpyMLPModel(obj, Path(path))


def np_policy_obs_width(path) -> int | None:
    """meta.obs_dim of a policy JSON, or None if unreadable."""
    try:
        return int(json.loads(Path(path).read_text())["meta"]["obs_dim"])
    except (OSError, ValueError, KeyError, TypeError):
        return None


def safe_policy_name(name: str) -> str | None:
    """Sanitized file stem for an uploaded policy, or None if invalid."""
    stem = Path(str(name)).name
    if stem.endswith(".json"):
        stem = stem[:-5]
    return stem if NAME_RE.match(stem) else None


# ---------------------------------------------------------------------------
# CLI

def _cli() -> None:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("validate", help="validate a policy JSON")
    v.add_argument("file")
    p = sub.add_parser("push", help="upload a policy to a robot or sim")
    p.add_argument("file")
    p.add_argument("--host", default="http://hexapod.local:8080")
    p.add_argument("--name", default=None,
                   help="store under this name (default: file stem)")
    g = sub.add_parser("pull", help="download an uploaded policy")
    g.add_argument("name")
    g.add_argument("--host", default="http://hexapod.local:8080")
    g.add_argument("-o", "--out", default=None)
    args = ap.parse_args()

    if args.cmd == "validate":
        obj = json.loads(Path(args.file).read_text())
        errs, info = validate_np_policy(obj)
        if errs:
            raise SystemExit("INVALID: " + "; ".join(errs))
        print(f"ok: obs {info['obs_dim']}, hidden {info['hidden']}, "
              f"name {info['name'] or Path(args.file).stem!r}")
    elif args.cmd == "push":
        path = Path(args.file)
        body = path.read_bytes()
        if len(body) > MAX_POLICY_BYTES:
            raise SystemExit(f"{path} too big")
        name = safe_policy_name(args.name or path.stem)
        if name is None:
            raise SystemExit("bad name (want [A-Za-z0-9._-]{1,64})")
        req = urllib.request.Request(
            args.host.rstrip("/") + "/api/rl/policies?name=" + name,
            data=body, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=60) as r:
            print(f"{path.name}: {r.read().decode()}")
    elif args.cmd == "pull":
        url = (args.host.rstrip("/") + "/api/rl/policies/"
               + str(args.name))
        with urllib.request.urlopen(url, timeout=30) as r:
            data = r.read()
        out = Path(args.out or f"{args.name}.json")
        out.write_bytes(data)
        print(f"pulled {args.name} -> {out} ({len(data)/1024:.0f} KB)")


if __name__ == "__main__":
    _cli()
