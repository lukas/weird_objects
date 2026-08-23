"""PPO trainer on batched MJX physics (MjxVecEnv) — separate entry point.

This is the EXPERIMENTAL large-batch trainer for the MJX/Warp backend.
It deliberately does NOT touch ``train_ppo_sim.py`` (the campaign's
production trainer keeps running unchanged); shared conventions
(task map, --cfg-set, W&B env file, policy dir) are imported from it.

Differences from the production trainer, on purpose:

- Physics: one ``MjxVecEnv`` (hundreds..thousands of envs, one
  accelerator) instead of Dummy/Subproc C-MuJoCo envs.
- Hyperparameters default to the large-batch regime: short rollouts
  (``--n-steps 16``) so an update happens every ~65k transitions at
  4096 envs, instead of 256-step rollouts tuned for 8-48 envs. These
  are STARTING points — the recipe rework is phase-2 item 6 in
  MJX_PORT.md and nothing here is validated for wall-clock learning yet.
- Monitoring eval/video runs in a background C-MuJoCo process, preserving
  the cross-simulator behavioral A/B without occupying the training GPU.
  Recovery curriculum decisions use a separate small deterministic MJX
  pool on the training backend; C evaluation is never an admission signal.
- v1 backend limits apply (MJX_PORT.md): model-field DR is OFF (shared
  nominal model), actuation/sensing DR is per-env as usual.

Typical GPU-pod use (after HEXAPOD_MJX=1 setup, mujoco-warp installed):

    python -m rl_move.sim.train_ppo_mjx --task joint_walk \
        --n-envs 4096 --impl warp --steps 20000000 --run-name mjx-trial0

Laptop smoke (CPU XLA, tiny batch):

    python -m rl_move.sim.train_ppo_mjx --smoke --no-wandb
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .mjx_backend import mjx_is_available  # noqa: E402
from .servo_model import SimServoParams  # noqa: E402
from .train_ppo_sim import (  # noqa: E402
    ENV_CLASSES, POLICY_DIR, WANDB_ENTITY_DEFAULT, WANDB_PROJECT_DEFAULT,
    _learning_line, _load_wandb_env, _parse_cfg_set, _parse_goal_mix,
    _resolved_reward_cfg, _reward_notes, _warn_if_defaults,
)


def _recover_episode_outcome(info: dict) -> tuple[int, bool] | None:
    """Extract one exact terminal training outcome from an env info."""
    if not any(str(k).startswith("recover_episode_bucket_")
               for k in info):
        return None
    bucket = int(float(info.get("recover_start_bucket", -1)))
    if bucket < 0:
        return None
    success = bool(
        info.get("recover_success", 0.0) > 0.0
        or info.get("termination_reason") == "recover_success")
    return bucket, success


def _recover_episode_training_error(info: dict) -> tuple[int, float] | None:
    """Extract sampler-only terminal shortfall from a training episode."""
    outcome = _recover_episode_outcome(info)
    if outcome is None or "recover_training_error" not in info:
        return None
    bucket, _success = outcome
    error = float(np.clip(info["recover_training_error"], 0.0, 1.0))
    return bucket, error


def _restore_recover_curriculum_from_sidecar(venv, sidecar_path) -> dict:
    """Restore promotion-time recovery curriculum state on EVERY env.

    Backs the default-off --recover-init-curriculum flag (operator order
    2026-08-18: continue the finished recover-any21-pop3 cohort from its
    exact final state instead of restarting the frontier at B0).  Accepts
    a promotion ``*.curriculum.json`` sidecar (the ``curriculum`` key) or
    a bare curriculum dict, applies it via the same env method the
    in-run rollback/adoption machinery uses, and verifies the fleet is
    synchronized afterwards.  Returns the curriculum dict applied.
    """
    payload = json.loads(Path(sidecar_path).read_text())
    curriculum = payload.get("curriculum", payload)
    venv.env_method("restore_recover_curriculum_checkpoint_state",
                    curriculum)
    active = [int(value) for value in venv.get_attr("_rec_active_n")]
    expected = int(curriculum["active_n"])
    if len(active) != venv.num_envs or set(active) != {expected}:
        counts = {value: active.count(value)
                  for value in sorted(set(active))}
        raise RuntimeError(
            "recover curriculum restore desynchronized the training "
            f"fleet: expected active_n={expected}, counts={counts}")
    return curriculum


def _recover_cert_bucket_plan(frontier: int, retention_count: int,
                              cursor: int,
                              weak_bucket: int | None) -> tuple[list[int], int]:
    """Frontier plus a weak bucket and rotating old-bucket assays."""
    frontier = max(0, int(frontier))
    old = list(range(frontier))
    buckets = [frontier]
    weak = -1 if weak_bucket is None else int(weak_bucket)
    if weak in old:
        buckets.append(weak)
    if not old or retention_count <= 0:
        return buckets, 0
    cursor = int(cursor) % len(old)
    scanned = 0
    added = 0
    while scanned < len(old) and added < int(retention_count):
        bucket = old[(cursor + scanned) % len(old)]
        scanned += 1
        if bucket in buckets:
            continue
        buckets.append(bucket)
        added += 1
    return buckets, (cursor + scanned) % len(old)


def _recover_update_regression_timers(
        failed_since: dict[int, int], gate_fractions: dict[int, float],
        step: int, threshold: float, rollback_after_steps: int
        ) -> list[int]:
    """Update per-bucket regression timers and return timed-out buckets."""
    for bucket in list(failed_since):
        if bucket not in gate_fractions:
            failed_since.pop(bucket, None)
    for bucket, fraction in gate_fractions.items():
        if float(fraction) < float(threshold):
            failed_since.setdefault(int(bucket), int(step))
        else:
            failed_since.pop(int(bucket), None)
    return sorted(
        bucket for bucket, failed_at in failed_since.items()
        if int(step) - failed_at >= int(rollback_after_steps))


def _recover_update_admission_all(vec_env, cert_round: int) -> tuple[dict, int]:
    """Atomically advance every training env and verify agreement.

    Recovery curriculum state lives on each host-side shim. Updating only
    env zero makes certification appear to advance while nearly every PPO
    rollout remains on B0, so treat a divergent fleet as a fatal error.
    """
    admissions = vec_env.env_method(
        "_recover_update_admission", int(cert_round))
    if not admissions:
        raise RuntimeError("recovery admission updated zero training envs")
    canonical = admissions[0]
    fields = ("active_before", "active_after", "promoted")
    expected = tuple(canonical[field] for field in fields)
    divergent = [
        index for index, admission in enumerate(admissions)
        if tuple(admission[field] for field in fields) != expected
    ]
    if divergent:
        preview = divergent[:8]
        raise RuntimeError(
            "recovery curriculum desynchronized across training envs; "
            f"canonical={expected}, divergent_indices={preview}, "
            f"divergent_count={len(divergent)}")
    return canonical, len(admissions)


def _recover_score_payload(state: dict, best_score: float = 0.0,
                           cert_ages: dict[int, int] | None = None
                           ) -> tuple[dict, float]:
    """Build the dedicated W&B recovery scoreboard.

    Bucket B contributes B+1 points times its latest deterministic success
    fraction. The denominator includes every curriculum bucket, including
    locked/untested ones, so the normalized score rises as harder abilities
    are unlocked rather than renormalizing the task underneath the policy.
    """
    total = int(state["total_buckets"])
    maximum = total * (total + 1) / 2.0
    rows = state.get("buckets", {})
    training_errors = state.get("training_errors", {})
    points = 0.0
    certified_weight = 0.0
    payload = {
        "RECOVER_SCORE/max_unlocked_bucket": float(
            state["max_unlocked_bucket"]),
        "RECOVER_SCORE/focus_bucket": float(state["focus_bucket"]),
        "RECOVER_SCORE/weakest_bucket": float(state["weakest_bucket"]),
        "RECOVER_SCORE/maximum_points": maximum,
    }
    gate_fractions = []
    for bucket in range(total):
        key = str(bucket)
        row = rows.get(key)
        weight = float(bucket + 1)
        if row is not None:
            fraction = float(row["success_fraction"])
            gate_fraction = float(row["gate_fraction"])
            bucket_points = weight * fraction
            points += bucket_points
            certified_weight += weight
            gate_fractions.append(gate_fraction)
            stem = f"RECOVER_SCORE/bucket_{bucket:02d}"
            payload[f"{stem}_success_fraction"] = fraction
            payload[f"{stem}_gate_fraction"] = gate_fraction
            payload[f"{stem}_successes"] = float(row["successes"])
            payload[f"{stem}_episodes"] = float(row["episodes"])
            payload[f"{stem}_points"] = bucket_points
            if cert_ages is not None and bucket in cert_ages:
                payload[f"{stem}_cert_age_rounds"] = float(
                    cert_ages[bucket])
        probability = state.get("sample_probabilities", {}).get(key)
        if probability is not None:
            payload[
                f"RECOVER_SCORE/bucket_{bucket:02d}_sample_probability"
            ] = float(probability)
        training_error = training_errors.get(key)
        if training_error is not None:
            stem = f"RECOVER_SCORE/bucket_{bucket:02d}"
            payload[f"{stem}_training_error_ema"] = float(
                training_error["ema"])
            payload[f"{stem}_training_error_episodes"] = float(
                training_error["episodes"])
            payload[f"{stem}_training_error_priority"] = float(
                training_error.get("priority", 0.0))
    score = points / maximum if maximum > 0.0 else 0.0
    best = max(float(best_score), score)
    payload.update({
        "RECOVER_SCORE/overall_points": points,
        "RECOVER_SCORE/overall_weighted_success": score,
        "RECOVER_SCORE/best_overall_weighted_success": best,
        "RECOVER_SCORE/certified_weight_fraction": (
            certified_weight / maximum if maximum > 0.0 else 0.0),
        "RECOVER_SCORE/min_certified_gate_fraction": (
            min(gate_fractions) if gate_fractions else 0.0),
    })
    return payload, best


def _recover_population_record(summary: dict, kind: str,
                               bucket: int) -> dict | None:
    """Decode one atomic candidate/winner/ack record from W&B summary."""
    raw = summary.get(
        f"recover_population/{kind}_B{int(bucket):02d}")
    if isinstance(raw, dict) and set(raw) == {"last"}:
        raw = raw["last"]
    if not raw:
        return None
    if isinstance(raw, str):
        try:
            raw = json.loads(raw)
        except json.JSONDecodeError:
            return None
    return dict(raw) if isinstance(raw, dict) else None


def _recover_population_choose_candidate(
        peer_rows: list[tuple[int, str, str, dict]], bucket: int,
        parent_fingerprint: str, population_id: str) -> dict | None:
    """Elect the earliest valid candidate, with member as the tie-break."""
    candidates = []
    for member, run_id, run_name, summary in peer_rows:
        row = _recover_population_record(summary, "candidate", bucket)
        if row is None:
            continue
        if (str(row.get("population_id", "")) != str(population_id)
                or int(row.get("bucket", -1)) != int(bucket)
                or int(row.get("member", -1)) != int(member)
                or str(row.get("run_id", "")) != str(run_id)
                or str(row.get("run_name", "")) != str(run_name)
                or str(row.get("parent_fingerprint", ""))
                != str(parent_fingerprint)
                or not row.get("policy_sha256")
                or not row.get("curriculum_sha256")
                or not row.get("policy_file")
                or not row.get("curriculum_file")):
            continue
        candidates.append(row)
    if not candidates:
        return None
    return min(candidates, key=lambda row: (
        int(row.get("time_ns", 0)), int(row["member"])))


def _recover_population_all_acked(
        peer_rows: list[tuple[int, str, str, dict]], winner: dict,
        population_id: str, expected_members: int) -> bool:
    """Require an identity-bound ACK from every cohort member."""
    if len(peer_rows) != int(expected_members):
        return False
    bucket = int(winner["bucket"])
    fingerprint = str(winner["policy_sha256"])
    for member, run_id, run_name, summary in peer_rows:
        ack = _recover_population_record(summary, "ack", bucket)
        if (ack is None
                or str(ack.get("population_id", "")) != str(population_id)
                or int(ack.get("bucket", -1)) != bucket
                or int(ack.get("member", -1)) != int(member)
                or str(ack.get("run_id", "")) != str(run_id)
                or str(ack.get("run_name", "")) != str(run_name)
                or str(ack.get("policy_sha256", "")) != fingerprint):
            return False
    return True


def _recover_population_release(
        leader_summary: dict, winner: dict,
        population_id: str) -> dict | None:
    """Return a valid leader release for one fully adopted winner."""
    bucket = int(winner["bucket"])
    row = _recover_population_record(leader_summary, "release", bucket)
    if (row is None
            or str(row.get("population_id", "")) != str(population_id)
            or int(row.get("bucket", -1)) != bucket
            or str(row.get("policy_sha256", ""))
            != str(winner["policy_sha256"])):
        return None
    return row


def _recover_population_all_ready(
        peer_rows: list[tuple[int, str, str, dict]], bucket: int,
        population_id: str, root_fingerprint: str,
        bootstrap_steps: int, expected_members: int) -> bool:
    """Require every seeded member to reach the same root budget."""
    if len(peer_rows) != int(expected_members):
        return False
    for member, run_id, run_name, summary in peer_rows:
        ready = _recover_population_record(summary, "ready", bucket)
        if (ready is None
                or str(ready.get("population_id", ""))
                != str(population_id)
                or int(ready.get("bucket", -1)) != int(bucket)
                or int(ready.get("member", -1)) != int(member)
                or str(ready.get("run_id", "")) != str(run_id)
                or str(ready.get("run_name", "")) != str(run_name)
                or str(ready.get("root_fingerprint", ""))
                != str(root_fingerprint)
                or int(ready.get("bootstrap_steps", -1))
                != int(bootstrap_steps)):
            return False
    return True


def _recover_population_start(
        leader_summary: dict, bucket: int, population_id: str,
        root_fingerprint: str, bootstrap_steps: int) -> dict | None:
    """Return a valid leader release for the initial seeded race."""
    row = _recover_population_record(leader_summary, "start", bucket)
    if (row is None
            or str(row.get("population_id", "")) != str(population_id)
            or int(row.get("bucket", -1)) != int(bucket)
            or str(row.get("root_fingerprint", ""))
            != str(root_fingerprint)
            or int(row.get("bootstrap_steps", -1))
            != int(bootstrap_steps)):
        return None
    return row


class _RecoverPopulation:
    """W&B-backed best-of-N checkpoint election for recovery training."""

    def __init__(self, args, run, initial_bucket: int):
        self.population_id = str(args.recover_population_id)
        self.member = int(args.recover_population_member)
        self.peer_names = tuple(
            name.strip() for name in args.recover_population_runs.split(",")
            if name.strip())
        self.peer_ids = tuple(
            run_id.strip()
            for run_id in args.recover_population_run_ids.split(",")
            if run_id.strip())
        if len(self.peer_ids) != len(self.peer_names):
            raise RuntimeError("recovery population W&B id roster mismatch")
        self.run = run
        self.entity = str(run.entity)
        self.project = str(run.project)
        self.project_path = f"{self.entity}/{self.project}"
        # Public Api run/list objects retain negative lookups inside the
        # active W&B service process. InternalApi.run_resume_status issues a
        # fresh GraphQL read and is specifically designed to see a run that
        # did not exist on an earlier call.
        self._internal_api = None
        self.api = None
        self.poll_seconds = float(args.recover_population_poll_seconds)
        self.barrier_timeout = float(
            args.recover_population_barrier_timeout_seconds)
        self.initial_bucket = int(initial_bucket)
        self.adopted_bucket = int(initial_bucket)
        self.parent_fingerprint = f"root:{self.population_id}"
        self.bootstrap_steps = int(
            args.n_envs * args.n_steps
            * args.recover_population_bootstrap_rollouts)
        self._start_ready = False
        self._race_started = False
        self._next_poll = 0.0
        self._peer_ids = dict(zip(self.peer_names, self.peer_ids))
        expected_run_id = self.peer_ids[self.member]
        if str(run.id) != expected_run_id:
            raise RuntimeError(
                "recovery population local W&B id mismatch: expected "
                f"{expected_run_id}, got {run.id}")
        self._local_summary: dict = {}
        self._sync_count = 0
        self._api_runs: dict[str, object] = {}
        self._pending_candidates: dict[int, dict] = {}
        self._pending_acks: dict[int, tuple[dict, int, int]] = {}

    def _summary_update(self, values: dict) -> None:
        self.run.summary.update(values)
        self._local_summary.update(values)

    def publish_candidate(self, checkpoint: dict) -> None:
        import hashlib
        path = Path(checkpoint["path"])
        metadata_path = Path(checkpoint["metadata_path"])
        bucket = int(checkpoint["bucket"])
        row = {
            "population_id": self.population_id,
            "bucket": bucket,
            "member": self.member,
            "run_id": str(self.run.id),
            "run_name": self.peer_names[self.member],
            "step": int(checkpoint["step"]),
            "time_ns": time.time_ns(),
            "parent_fingerprint": self.parent_fingerprint,
            "policy_file": path.relative_to(POLICY_DIR).as_posix(),
            "curriculum_file": metadata_path.relative_to(
                POLICY_DIR).as_posix(),
            "policy_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "curriculum_sha256": hashlib.sha256(
                metadata_path.read_bytes()).hexdigest(),
        }
        self._pending_candidates[bucket] = row
        self._flush_candidates()

    def _flush_candidates(self) -> None:
        """Upload files before making an atomic candidate record visible."""
        import wandb
        for bucket, row in list(self._pending_candidates.items()):
            try:
                for key in ("policy_file", "curriculum_file"):
                    rel = self._checked_relative_path(row[key])
                    path = POLICY_DIR / rel
                    if not path.is_file():
                        raise FileNotFoundError(path)
                    self.run.save(
                        str(path), base_path=str(POLICY_DIR), policy="now")
                self._summary_update({
                    f"recover_population/candidate_B{bucket:02d}":
                        json.dumps(row, sort_keys=True),
                    "recover_population/latest_candidate_bucket": bucket,
                    "recover_latest_promotion_checkpoint":
                        str(row["policy_file"]),
                })
            except Exception as exc:
                print("[recover-pop] candidate upload deferred for "
                      f"B{bucket}: {exc}")
                continue
            del self._pending_candidates[bucket]
            try:
                wandb.log({
                    "global_step": int(row["step"]),
                    "RECOVER_POPULATION/candidate_bucket": float(bucket),
                    "RECOVER_POPULATION/candidate_member": float(self.member),
                })
            except Exception as exc:
                print(f"[recover-pop] candidate metric deferred: {exc}")
            print("[recover-pop] published retained candidate "
                  f"B{bucket} from member {self.member}")

    def _peer_rows(self) -> list[tuple[int, str, str, dict]]:
        if self._internal_api is None:
            from wandb.sdk.internal.internal_api import Api as InternalApi
            self._internal_api = InternalApi()
        rows = []
        for member, name in enumerate(self.peer_names):
            run_id = self._peer_ids.get(name)
            if run_id is None:
                continue
            payload = self._internal_api.run_resume_status(
                self.entity, self.project, run_id)
            if payload is None:
                continue
            if str(payload.get("name", "")) != run_id:
                raise RuntimeError(
                    f"recovery population peer id mismatch for {run_id}")
            display_name = str(payload.get("displayName", ""))
            if display_name != name:
                raise RuntimeError(
                    "recovery population peer name mismatch for "
                    f"{run_id}: expected {name}, got {display_name}")
            raw_summary = payload.get("summaryMetrics") or {}
            if isinstance(raw_summary, str):
                raw_summary = json.loads(raw_summary)
            if not isinstance(raw_summary, dict):
                raise RuntimeError(
                    f"invalid W&B summary for recovery peer {run_id}")
            summary = dict(raw_summary)
            if run_id == str(self.run.id):
                summary.update(self._local_summary)
            rows.append((member, run_id, name, summary))
        return rows

    def _winner(self, leader_summary: dict, bucket: int) -> dict | None:
        row = _recover_population_record(leader_summary, "winner", bucket)
        if (row is None
                or row.get("population_id") != self.population_id
                or int(row.get("bucket", -1)) != int(bucket)):
            return None
        return row

    def _all_acked(self, peer_rows, winner: dict) -> bool:
        return _recover_population_all_acked(
            peer_rows, winner, self.population_id, len(self.peer_names))

    def _release(self, leader_summary: dict,
                 winner: dict) -> dict | None:
        return _recover_population_release(
            leader_summary, winner, self.population_id)

    def _start(self, leader_summary: dict) -> dict | None:
        return _recover_population_start(
            leader_summary, self.initial_bucket, self.population_id,
            self.parent_fingerprint, self.bootstrap_steps)

    def _elect(self, peer_rows, leader_summary: dict) -> dict:
        if self.member != 0 or len(peer_rows) != len(self.peer_names):
            return leader_summary
        last_bucket = self.initial_bucket
        last_winner = None
        while True:
            row = self._winner(leader_summary, last_bucket + 1)
            if row is None:
                break
            last_bucket += 1
            last_winner = row
        if last_winner is None and self._start(leader_summary) is None:
            return leader_summary
        # ACKs alone are not enough: every member blocks after ACK until the
        # leader publishes this release. That gives each bucket three fresh
        # branches from the exact same parent instead of letting one member
        # build a multi-bucket head start while slower peers are restoring.
        if last_winner is not None and self._release(
                leader_summary, last_winner) is None:
            return leader_summary
        parent = (f"root:{self.population_id}" if last_winner is None
                  else str(last_winner["policy_sha256"]))
        candidate = _recover_population_choose_candidate(
            peer_rows, last_bucket + 1, parent, self.population_id)
        if candidate is None:
            return leader_summary
        winner = dict(candidate)
        winner["elected_time_ns"] = time.time_ns()
        key = f"recover_population/winner_B{last_bucket + 1:02d}"
        values = {
            key: json.dumps(winner, sort_keys=True),
            "recover_population/latest_winner_bucket": last_bucket + 1,
        }
        self._summary_update(values)
        leader_summary.update(values)
        import wandb
        wandb.log({
            "global_step": int(self.run.summary.get("global_step", 0)),
            "RECOVER_POPULATION/winner_bucket": float(last_bucket + 1),
            "RECOVER_POPULATION/winner_member": float(winner["member"]),
        })
        print("[recover-pop] elected member "
              f"{winner['member']} candidate for B{last_bucket + 1}")
        return leader_summary

    @staticmethod
    def _checked_relative_path(raw: str) -> Path:
        path = Path(str(raw))
        if path.is_absolute() or ".." in path.parts:
            raise ValueError(f"unsafe recovery population file {raw!r}")
        return path

    def _download(self, winner: dict) -> dict:
        import hashlib
        run_id = str(winner["run_id"])
        policy_rel = self._checked_relative_path(winner["policy_file"])
        curriculum_rel = self._checked_relative_path(
            winner["curriculum_file"])
        if run_id == str(self.run.id):
            policy_path = POLICY_DIR / policy_rel
            curriculum_path = POLICY_DIR / curriculum_rel
        else:
            import wandb
            if self.api is None:
                self.api = wandb.Api(timeout=15)
            api_run = self._api_runs.get(run_id)
            if api_run is None:
                api_run = self.api.run(
                    f"{self.project_path}/{run_id}")
                self._api_runs[run_id] = api_run
            root = (POLICY_DIR / "recover_population" /
                    self.population_id /
                    f"B{int(winner['bucket']):02d}_m{int(winner['member'])}")
            root.mkdir(parents=True, exist_ok=True)
            for rel in (policy_rel, curriculum_rel):
                remote = api_run.file(rel.as_posix())
                if remote is None:
                    raise FileNotFoundError(
                        f"W&B run {run_id} has no file {rel.as_posix()}")
                remote.download(root=str(root), replace=True)
            policy_path = root / policy_rel
            curriculum_path = root / curriculum_rel
        for path, key in ((policy_path, "policy_sha256"),
                          (curriculum_path, "curriculum_sha256")):
            if not path.is_file():
                raise FileNotFoundError(path)
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != str(winner[key]):
                raise RuntimeError(
                    f"recovery population checksum mismatch for {path}")
        metadata = json.loads(curriculum_path.read_text())
        if int(metadata["promotion_bucket"]) != int(winner["bucket"]):
            raise RuntimeError("recovery population curriculum bucket "
                               "does not match the elected winner")
        return {
            "path": policy_path,
            "metadata_path": curriculum_path,
            "bucket": int(winner["bucket"]),
            "step": int(winner["step"]),
            "curriculum": metadata["curriculum"],
            "cert_round": int(metadata.get("cert_round", 0)),
            "population_record": winner,
        }

    def poll(self) -> dict | None:
        now = time.monotonic()
        if now < self._next_poll:
            return None
        self._next_poll = now + self.poll_seconds
        self._flush_candidates()
        self._flush_acks()
        try:
            peer_rows = self._peer_rows()
            if len(peer_rows) != len(self.peer_names):
                print("[recover-pop] waiting for all peer W&B runs: "
                      f"{len(peer_rows)}/{len(self.peer_names)} visible")
                return None
            leader_summary = peer_rows[0][3]
            leader_summary = self._elect(peer_rows, leader_summary)
            winner = self._winner(leader_summary, self.adopted_bucket + 1)
            if winner is None:
                return None
            if str(winner.get("parent_fingerprint", "")) != (
                    self.parent_fingerprint):
                raise RuntimeError(
                    "recovery population winner has the wrong parent "
                    f"fingerprint for B{self.adopted_bucket + 1}")
            return self._download(winner)
        except Exception as exc:
            print(f"[recover-pop] poll deferred: {exc}")
            return None

    def acknowledge(self, checkpoint: dict, global_step: int) -> None:
        winner = checkpoint["population_record"]
        bucket = int(checkpoint["bucket"])
        self.adopted_bucket = bucket
        self.parent_fingerprint = str(winner["policy_sha256"])
        self._sync_count += 1
        ack = {
            "population_id": self.population_id,
            "bucket": bucket,
            "member": self.member,
            "run_id": str(self.run.id),
            "run_name": self.peer_names[self.member],
            "policy_sha256": self.parent_fingerprint,
            "winner_run_id": str(winner["run_id"]),
            "ack_time_ns": time.time_ns(),
        }
        self._pending_acks[bucket] = (
            ack, int(global_step), int(winner["member"]))
        self._flush_acks()

    def wait_for_start(self, global_step: int) -> None:
        """Equalize seeded B0 budgets before the first candidate can win."""
        if self._race_started or int(global_step) < self.bootstrap_steps:
            return
        import wandb
        started = time.monotonic()
        deadline = started + self.barrier_timeout
        next_status = started
        sleep_seconds = min(max(self.poll_seconds, 1.0), 5.0)
        while True:
            if not self._start_ready:
                ready = {
                    "population_id": self.population_id,
                    "bucket": self.initial_bucket,
                    "member": self.member,
                    "run_id": str(self.run.id),
                    "run_name": self.peer_names[self.member],
                    "root_fingerprint": self.parent_fingerprint,
                    "bootstrap_steps": self.bootstrap_steps,
                    "ready_time_ns": time.time_ns(),
                }
                try:
                    self._summary_update({
                        f"recover_population/ready_B{self.initial_bucket:02d}":
                            json.dumps(ready, sort_keys=True),
                        "recover_population/bootstrap_ready_bucket":
                            self.initial_bucket,
                    })
                    self._start_ready = True
                    try:
                        wandb.log({
                            "global_step": int(global_step),
                            "RECOVER_POPULATION/bootstrap_ready": 1.0,
                        })
                    except Exception as exc:
                        print("[recover-pop] bootstrap-ready metric deferred: "
                              f"{exc}")
                except Exception as exc:
                    print("[recover-pop] bootstrap readiness deferred: "
                          f"{exc}")
            try:
                peer_rows = self._peer_rows()
                if len(peer_rows) == len(self.peer_names):
                    leader_summary = peer_rows[0][3]
                    release = self._start(leader_summary)
                    if (release is None and self.member == 0
                            and _recover_population_all_ready(
                                peer_rows, self.initial_bucket,
                                self.population_id,
                                self.parent_fingerprint,
                                self.bootstrap_steps,
                                len(self.peer_names))):
                        release = {
                            "population_id": self.population_id,
                            "bucket": self.initial_bucket,
                            "root_fingerprint": self.parent_fingerprint,
                            "bootstrap_steps": self.bootstrap_steps,
                            "member_count": len(self.peer_names),
                            "start_time_ns": time.time_ns(),
                        }
                        values = {
                            f"recover_population/start_B{self.initial_bucket:02d}":
                                json.dumps(release, sort_keys=True),
                            "recover_population/start_bucket":
                                self.initial_bucket,
                        }
                        self._summary_update(values)
                        leader_summary.update(values)
                        try:
                            wandb.log({
                                "global_step": int(global_step),
                                "RECOVER_POPULATION/start_release": 1.0,
                            })
                        except Exception as exc:
                            print("[recover-pop] start metric deferred: "
                                  f"{exc}")
                        print("[recover-pop] leader RELEASED initial B"
                              f"{self.initial_bucket} race after all "
                              f"{len(self.peer_names)} members reached "
                              f"{self.bootstrap_steps:,} steps")
                    if release is not None:
                        waited = time.monotonic() - started
                        self._race_started = True
                        try:
                            wandb.log({
                                "global_step": int(global_step),
                                "RECOVER_POPULATION/start_observed": 1.0,
                                "RECOVER_POPULATION/start_wait_seconds":
                                    float(waited),
                            })
                        except Exception as exc:
                            print("[recover-pop] start-observed metric "
                                  f"deferred: {exc}")
                        print("[recover-pop] member "
                              f"{self.member} STARTED initial B"
                              f"{self.initial_bucket} race after "
                              f"{waited:.1f}s")
                        return
            except Exception as exc:
                print(f"[recover-pop] start poll deferred: {exc}")
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError(
                    "recovery population timed out waiting for all members "
                    f"to reach the {self.bootstrap_steps:,}-step initial "
                    f"race barrier after {self.barrier_timeout:.0f}s")
            if now >= next_status:
                print("[recover-pop] member "
                      f"{self.member} WAITING at initial B"
                      f"{self.initial_bucket} bootstrap barrier")
                next_status = now + 60.0
            time.sleep(min(sleep_seconds, deadline - now))

    def wait_for_release(self, checkpoint: dict, global_step: int) -> None:
        """Block rollout collection until every member ACKs this winner."""
        import wandb
        winner = checkpoint["population_record"]
        bucket = int(checkpoint["bucket"])
        started = time.monotonic()
        deadline = started + self.barrier_timeout
        next_status = started
        sleep_seconds = min(max(self.poll_seconds, 1.0), 5.0)
        while True:
            self._flush_acks()
            try:
                peer_rows = self._peer_rows()
                if len(peer_rows) == len(self.peer_names):
                    leader_summary = peer_rows[0][3]
                    release = self._release(leader_summary, winner)
                    if (release is None and self.member == 0
                            and self._all_acked(peer_rows, winner)):
                        release = {
                            "population_id": self.population_id,
                            "bucket": bucket,
                            "policy_sha256": str(winner["policy_sha256"]),
                            "winner_run_id": str(winner["run_id"]),
                            "member_count": len(self.peer_names),
                            "release_time_ns": time.time_ns(),
                        }
                        values = {
                            f"recover_population/release_B{bucket:02d}":
                                json.dumps(release, sort_keys=True),
                            "recover_population/latest_release_bucket":
                                bucket,
                        }
                        self._summary_update(values)
                        leader_summary.update(values)
                        try:
                            wandb.log({
                                "global_step": int(global_step),
                                "RECOVER_POPULATION/release_bucket":
                                    float(bucket),
                            })
                        except Exception as exc:
                            print("[recover-pop] release metric deferred: "
                                  f"{exc}")
                        print("[recover-pop] leader RELEASED B"
                              f"{bucket} after all {len(self.peer_names)} "
                              "members ACKed")
                    if release is not None:
                        waited = time.monotonic() - started
                        try:
                            wandb.log({
                                "global_step": int(global_step),
                                "RECOVER_POPULATION/released_bucket":
                                    float(bucket),
                                "RECOVER_POPULATION/release_wait_seconds":
                                    float(waited),
                            })
                        except Exception as exc:
                            print("[recover-pop] release-observed metric "
                                  f"deferred: {exc}")
                        print("[recover-pop] member "
                              f"{self.member} observed RELEASE B{bucket} "
                              f"after {waited:.1f}s; next race may start")
                        return
            except Exception as exc:
                print(f"[recover-pop] release poll deferred: {exc}")
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError(
                    "recovery population timed out waiting for all members "
                    f"to ACK/release B{bucket} after "
                    f"{self.barrier_timeout:.0f}s")
            if now >= next_status:
                print("[recover-pop] member "
                      f"{self.member} WAITING at B{bucket} ACK barrier")
                next_status = now + 60.0
            time.sleep(min(sleep_seconds, deadline - now))

    def _flush_acks(self) -> None:
        import wandb
        for bucket, (ack, global_step, winner_member) in list(
                self._pending_acks.items()):
            try:
                self._summary_update({
                    f"recover_population/ack_B{bucket:02d}":
                        json.dumps(ack, sort_keys=True),
                    "recover_population/latest_ack_bucket": bucket,
                })
            except Exception as exc:
                print(f"[recover-pop] ACK deferred for B{bucket}: {exc}")
                continue
            del self._pending_acks[bucket]
            try:
                wandb.log({
                    "global_step": global_step,
                    "RECOVER_POPULATION/adopted_bucket": float(bucket),
                    "RECOVER_POPULATION/ack_bucket": float(bucket),
                    "RECOVER_POPULATION/winner_member": float(winner_member),
                    "RECOVER_POPULATION/sync_count": float(self._sync_count),
                })
            except Exception as exc:
                print(f"[recover-pop] ACK metric deferred: {exc}")


def _run_recover_cert_kind(vec_env, model, kind: str) -> dict:
    """One deterministic first-episode recovery assay on an MJX VecEnv."""
    n_envs = int(vec_env.num_envs)
    vec_env.set_attr("force_recover_start", str(kind))
    obs = vec_env.reset()
    state = None
    episode_start = np.ones(n_envs, dtype=bool)
    finished = np.zeros(n_envs, dtype=bool)
    outcomes = np.zeros(n_envs, dtype=bool)
    finish_ticks = np.zeros(n_envs, dtype=np.int64)
    max_ticks = int(getattr(vec_env, "_episode_steps", 0)) + 2
    if max_ticks <= 2:
        raise RuntimeError("MJX recovery cert env has no episode horizon")
    ticks = 0
    while not bool(np.all(finished)):
        actions, state = model.predict(
            obs, state=state, episode_start=episode_start,
            deterministic=True)
        obs, _rewards, dones, infos = vec_env.step(actions)
        ticks += 1
        episode_start = np.asarray(dones, dtype=bool)
        for i in np.flatnonzero(np.asarray(dones) & ~finished):
            info = infos[int(i)]
            outcomes[i] = bool(
                info.get("recover_success", 0.0) > 0.0
                or info.get("termination_reason") == "recover_success")
            finished[i] = True
            finish_ticks[i] = ticks
        if ticks > max_ticks:
            missing = np.flatnonzero(~finished).tolist()
            raise RuntimeError(
                f"MJX recovery certification exceeded the episode "
                f"horizon for envs {missing}")
    dt = float(getattr(vec_env, "_dt", 0.0))
    return {
        "kind": str(kind),
        "outcomes": outcomes.tolist(),
        "successes": int(outcomes.sum()),
        "episodes": n_envs,
        "success": float(outcomes.mean()),
        "time_mean_s": float(finish_ticks.mean() * dt),
    }


def _env_kwargs(args, params: SimServoParams | None = None) -> dict:
    """Per-shim-env kwargs — mirrors train_ppo_sim._build_env, minus the
    model-DR pieces the shared-model backend can't honor yet.

    ``params=None`` resolves the actuator set from the run's cfg
    (bus.servo_params: "" = air fit, "loaded" = loaded bench fit)."""
    kw = dict(randomize=not args.no_dr,
              dr_scale=args.dr_scale,
              episode_seconds=getattr(
                  args, "training_episode_seconds", args.episode_seconds))
    overrides = _parse_cfg_set(args.cfg_set)
    external_recover_cert = (
        args.recover_cert_every > 0
        and args.recover_cert_envs > 0
        and float(_parse_goal_mix(args.goal_mix).get("recover", 0.0)) > 0.0)
    if overrides or external_recover_cert:
        from rl_move.config import load_config
        cfg = load_config()
        for dotted, val in overrides.items():
            node = cfg
            *path, leaf = dotted.split(".")
            for k in path:
                node = node.setdefault(k, {})
            node[leaf] = val
        if external_recover_cert:
            cfg.setdefault("goal", {})[
                "recover_external_certification"] = 1.0
        kw["cfg"] = cfg
    kw["params"] = (params if params is not None
                    else SimServoParams.from_cfg(kw.get("cfg")))
    return kw


def _resolve_impl(requested: str) -> str | None:
    """'auto' = warp when importable (GPU pods), else the mjx default."""
    if requested == "auto":
        try:
            import mujoco_warp  # noqa: F401
            return "warp"
        except Exception:
            return None
    return None if requested == "default" else requested


def _unwrap_vec(venv):
    """Innermost VecEnv under any wrapper chain (VecMonitor etc.)."""
    core = venv
    while hasattr(core, "venv"):
        core = core.venv
    return core


def _assert_gpu_physics(venv, impl: str | None) -> None:
    """Fail-closed all-GPU-physics guard (operator order 2026-08-18,
    fb_20260818T065930_03b422): a run launched with
    --require-gpu-physics must train on batched MJX/Warp physics — the
    actual VecEnv class is checked, so a SubprocVecEnv/DummyVecEnv of
    C-MuJoCo envs (CPU physics, whatever device Torch runs on) is
    REFUSED, and so is the XLA/jax fallback impl. Raises SystemExit;
    unit-tested in rl_move/tests/test_walkcurr_mjx.py."""
    from .mjx_sharded_vec_env import MjxShardedVecEnv
    from .mjx_vec_env import MjxVecEnv
    core = _unwrap_vec(venv)
    if not isinstance(core, (MjxVecEnv, MjxShardedVecEnv)):
        raise SystemExit(
            "--require-gpu-physics: the training VecEnv is "
            f"{type(core).__name__}, not MjxVecEnv/MjxShardedVecEnv — "
            "training physics would NOT run on the GPU. SubprocVecEnv/"
            "C-MuJoCo training physics is forbidden for this run "
            "(fb_20260818T065930_03b422).")
    if impl != "warp":
        raise SystemExit(
            f"--require-gpu-physics: physics impl is {impl!r}, not "
            "'warp' — the CUDA-batched Warp backend is required "
            "(fb_20260818T065930_03b422).")
    print(f"[backend] training physics VERIFIED: "
          f"{type(core).__name__} impl=warp n_envs={core.num_envs} "
          "(batched GPU physics; SubprocVecEnv refused by contract)",
          flush=True)


def apply_walkcurr_post_promo_schedule(model, epochs: int,
                                       actor_lr: float,
                                       actor_lr_final: float) -> dict:
    """Frontier-gated update-schedule handover (default OFF; operator
    order fb_20260818T085648_2a0a60): on the walk curriculum's FIRST
    promotion (B0 mastered) the acquisition-strength update (high
    actor LR / extra epochs, needed to ignite walking from scratch)
    hands over to the consolidation recipe proven by
    cw-dynrep-criticD-40m1 (fewer epochs, decaying actor LR). Mutates
    ``model.n_epochs`` and the update_health actor group in place;
    returns {knob: (old, new)} for logging. Unit-tested in
    rl_move/tests/test_walkcurr_mjx.py."""
    changed: dict = {}
    if epochs > 0:
        changed["n_epochs"] = (int(model.n_epochs), int(epochs))
        model.n_epochs = int(epochs)
    if actor_lr > 0.0:
        st = getattr(model, "_ac_state", None)
        if st is None:
            raise RuntimeError(
                "--walkcurr-post-promo-actor-lr requires --actor-lr "
                "(update_health actor/critic groups not attached)")
        final = float(actor_lr_final) if actor_lr_final > 0.0 \
            else float(actor_lr)
        changed["actor_lr"] = (float(st["actor_lr"]), float(actor_lr))
        changed["actor_lr_final"] = (float(st["actor_lr_final"]), final)
        st["actor_lr"] = float(actor_lr)
        st["actor_lr_final"] = final
    return changed


def _init_wandb(args, params: SimServoParams):
    if args.no_wandb:
        return None
    _load_wandb_env()
    try:
        import wandb
    except Exception:
        print("[wandb] not installed — logging skipped")
        return None
    import os
    if not os.environ.get("WANDB_API_KEY"):
        try:
            if not wandb.api.api_key:
                raise RuntimeError
        except Exception:
            print("[wandb] no API key — logging skipped")
            return None
    # Plain-English objective FIRST (operator 08-10: the overview must
    # open with what the run is learning, not lineage babble).
    notes = (_learning_line(args) + "\n\n" + (args.notes or "")).strip()
    notes += "\n\n" + _reward_notes(args.cfg_set)
    _contract = getattr(args, "_motor_contract", None)
    if _contract is None:
        from .servo_model import motor_contract
        _contract = motor_contract(params=params,
                                   backend=f"mjx_tickparams:{args.impl}")
    wandb_identity = {}
    if args.recover_population_id:
        population_ids = tuple(
            run_id.strip()
            for run_id in args.recover_population_run_ids.split(",")
            if run_id.strip())
        wandb_identity = {
            "id": population_ids[args.recover_population_member],
            "resume": "never",
        }
    run = wandb.init(
        entity=WANDB_ENTITY_DEFAULT,
        project=os.environ.get("WANDB_PROJECT", WANDB_PROJECT_DEFAULT),
        # Research tracks (operator 08-11) arrive as WANDB_TAGS
        # (track:<id>), which wandb.init honors natively.
        group=(args.recover_population_id or "mjx-trainer"),
        job_type=(f"recover-member-{args.recover_population_member}"
                  if args.recover_population_id else None),
        name=args.run_name, notes=notes,
        **wandb_identity,
        sync_tensorboard=True,   # SB3 train/* metrics, like the campaign
        config={"trainer": "train_ppo_mjx", "task": args.task,
                # Resolved motor contract (fb_20260820T000059): stashed
                # on args by main from the actual env params/cfg;
                # fallback resolves from the run's params (from_cfg is
                # the single enforcement point either way).
                "motor_contract": _contract,
                "n_envs": args.n_envs, "impl": args.impl,
                "n_steps": args.n_steps, "batch_size": args.batch_size,
                "learning_rate": args.lr, "seed": args.seed,
                "dr_scale": args.dr_scale, "no_dr": args.no_dr,
                "predictive_actor": bool(args.predictive_actor),
                "predictive_live": bool(args.predictive_live),
                "cfg_set": args.cfg_set,
                "reward_cfg": _resolved_reward_cfg(args.cfg_set),
                "episode_seconds": getattr(
                    args, "training_episode_seconds", args.episode_seconds),
                "eval_episode_seconds": args.episode_seconds,
                "walk_curriculum": bool(args.walk_curriculum),
                "walk_curriculum_version": args.walk_curriculum_version,
                "walkcurr_cert_every": args.walkcurr_cert_every,
                "walkcurr_cert_episodes": args.walkcurr_cert_episodes,
                "mjx_iterations": args.mjx_iterations,
                "mjx_ls_iterations": args.mjx_ls_iterations,
                "recover_cert_every": args.recover_cert_every,
                "recover_cert_envs": args.recover_cert_envs,
                "recover_retention_buckets": args.recover_retention_buckets,
                "recover_full_retention_every": (
                    args.recover_full_retention_every),
                "recover_rollback_after_steps": (
                    args.recover_rollback_after_steps),
                "recover_rollback_fraction": (
                    args.recover_rollback_fraction),
                "recover_population_id": args.recover_population_id,
                "recover_population_member": (
                    args.recover_population_member),
                "recover_population_runs": (
                    args.recover_population_runs.split(",")
                    if args.recover_population_runs else []),
                "recover_population_run_ids": (
                    args.recover_population_run_ids.split(",")
                    if args.recover_population_run_ids else []),
                "recover_population_barrier_timeout_seconds": (
                    args.recover_population_barrier_timeout_seconds),
                "recover_population_bootstrap_rollouts": (
                    args.recover_population_bootstrap_rollouts),
                "recover_population_bootstrap_steps": (
                    args.n_envs * args.n_steps
                    * args.recover_population_bootstrap_rollouts),
                "recover_replay_mix": {
                    "focus": 0.50, "recent_three": 0.25,
                    "weakest": 0.15, "uniform_older": 0.10,
                    "training_error_overlay": 0.10},
                "recover_buckets": {
                    str(i): list(family) for i, family in enumerate(
                        getattr(ENV_CLASSES[args.task],
                                "RECOVER_FAMILIES", ()))},
                "model_dr": False,  # v1 backend limit, see MJX_PORT.md
                "sim_model_source": params.source})
    # Same headline-score pinning as the C trainer (the periodic eval
    # is shared code, so MJX runs emit identical SCORE/* names).
    run.define_metric("SCORE/*", step_metric="global_step",
                      summary="last")
    run.define_metric("eval/*", step_metric="global_step")
    run.define_metric("CERT/*", step_metric="global_step", summary="last")
    run.define_metric("TRAIN/*", step_metric="global_step", summary="last")
    run.define_metric("RECOVER_SCORE/*", step_metric="global_step",
                      summary="last")
    run.define_metric("RECOVER_GUARD/*", step_metric="global_step",
                      summary="last")
    run.define_metric("RECOVER_POPULATION/*", step_metric="global_step",
                      summary="last")
    print(f"[wandb] logging to {run.url or 'offline run dir'}")
    return run


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", choices=sorted(ENV_CLASSES),
                    default="joint_walk")
    ap.add_argument("--steps", type=int, default=5_000_000)
    ap.add_argument("--n-envs", type=int, default=1024)
    ap.add_argument("--impl", default="auto",
                    choices=["auto", "warp", "jax", "default"],
                    help="mjx backend; auto = warp if installed (GPU "
                         "pods), else the XLA default (laptop smoke)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--no-dr", action="store_true")
    ap.add_argument("--dr-scale", type=float, default=1.0)
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="SEC.KEY=VAL")
    ap.add_argument("--goal-mix", type=str, default=None,
                    help='e.g. "walk=0.7,rise=0.15" (goal tasks only)')
    ap.add_argument("--pool-per-env", type=int, default=2)
    ap.add_argument("--host-workers", type=int, default=0,
                    help="shard the per-env host halves across N worker "
                         "processes (MjxShardedVecEnv) — THE throughput "
                         "knob; size to the node's spare cores. 0 = "
                         "in-process reference implementation")
    ap.add_argument("--mjx-iterations", type=int, default=None,
                    help="solver iterations (default: 1 under warp — "
                         "parity-checked on-pod; 8 otherwise)")
    ap.add_argument("--mjx-ls-iterations", type=int, default=None)
    # Large-batch PPO knobs (defaults are STARTING points, unvalidated).
    ap.add_argument("--n-steps", type=int, default=16,
                    help="rollout length per env per update")
    ap.add_argument("--batch-size", type=int, default=8192)
    ap.add_argument("--gamma", type=float, default=None,
                    help="PPO discount. Default None = legacy exact: "
                         "0.99 on fresh/transplant constructors, the "
                         "checkpoint's own value on a plain --init-from "
                         "warm start. Long-horizon tasks (e.g. the "
                         "recover mode's 3-5 s recoveries) want 0.995.")
    ap.add_argument("--gae-lambda", type=float, default=None,
                    help="GAE lambda. Default None = legacy exact "
                         "(0.95 / checkpoint's own), same contract as "
                         "--gamma.")
    ap.add_argument("--n-epochs", type=int, default=5)
    ap.add_argument("--lr", type=float, default=3e-4)
    # Update-path protection (08-17, operator-approved
    # fb_20260817T005114 after the scratch3 late-run collapse; all
    # default OFF = legacy single-group optimizer, bit-exact).
    ap.add_argument("--actor-lr", type=float, default=0.0,
                    help="separate actor param-group LR (>0 enables "
                         "actor/critic optimizer param groups via "
                         "update_health.attach_actor_critic_lr)")
    ap.add_argument("--actor-lr-final", type=float, default=0.0,
                    help="linear-decay target for --actor-lr over the "
                         "run (0 = no decay)")
    ap.add_argument("--critic-lr", type=float, default=0.0,
                    help="critic param-group LR, constant (0 = --lr); "
                         "keeps the critic learning while the actor "
                         "is throttled")
    ap.add_argument("--kl-rollback", type=float, default=0.0,
                    help="hard realized-KL ceiling: snapshot the "
                         "policy before each PPO update, restore it "
                         "and cut the actor LR when train/approx_kl "
                         "exceeds this (0 = off; requires --actor-lr)")
    ap.add_argument("--kl-rollback-lr-factor", type=float, default=0.5,
                    help="actor-LR scale multiplier per rollback")
    ap.add_argument("--ev-stop-min", type=float, default=0.0,
                    help="critic explained-variance hard gate: stop "
                         "the run if the EV EMA is still below this "
                         "after --ev-stop-after steps (0 = off). "
                         "EV~0 is a value-learning hard failure, "
                         "never a plateau")
    ap.add_argument("--ev-stop-after", type=int, default=1_500_000)
    ap.add_argument("--best-ckpt", action="store_true",
                    help="retain <out>_best.zip at every periodic-eval "
                         "composite-health improvement (survival, "
                         "direction error, aligned velocity) and arm "
                         "the joint regression auto-stop")
    ap.add_argument("--regress-stop-n", type=int, default=3,
                    help="consecutive jointly-regressed assays "
                         "(reward AND survival AND direction) before "
                         "the --best-ckpt auto-stop fires")
    ap.add_argument("--ent-coef", type=float, default=1e-3)
    ap.add_argument("--ent-coef-final", type=float, default=None,
                    help="linearly anneal ent_coef from --ent-coef down "
                         "to this value over --ent-coef-anneal-frac of "
                         "--steps, then hold (mutates model.ent_coef "
                         "once per rollout, same pattern as the servo "
                         "profile ramp). Default None = OFF, bit-exact "
                         "legacy (fixed ent_coef all run). Motivated by "
                         "cw-dep-bcgait4-phasedir3-fwd-reprice (08-22): "
                         "PPO's action std stayed pegged ~0.355-0.365 "
                         "the whole 2M-step run (no entropy pressure "
                         "ever relaxed), which forces any slip/overspeed "
                         "price band wide enough to spare honest "
                         "exploration noise to also spare a drifted "
                         "deterministic mean — the recorded next lever "
                         "was 'anchor dose/std annealing, NOT new "
                         "behavior charges'.")
    ap.add_argument("--ent-coef-anneal-frac", type=float, default=1.0,
                    help="fraction of --steps over which the ent-coef "
                         "anneal completes (only used when "
                         "--ent-coef-final is set); 1.0 = anneal across "
                         "the whole run, 0.5 = reach the final value "
                         "at the halfway point and hold.")
    ap.add_argument("--target-kl", type=float, default=0.02)
    # AMP style reward (AMP_LOCOMOTION.md §5.2/§7, wired 08-22).
    # Default 0.0 = OFF, bit-exact legacy: no env cfg injection, no
    # wrapper, no callback, no discriminator — nothing constructed.
    ap.add_argument("--amp-style-weight", type=float, default=0.0,
                    help="blend weight for the AMP discriminator style "
                         "reward: r = amp_task_weight * r_env + "
                         "amp_style_weight * r_style (r_style in "
                         "[0,1]/tick). >0 auto-injects "
                         "goal.amp_style_obs=1 into the env cfg, wraps "
                         "the vec env in AMPStyleVecWrapper and trains "
                         "the discriminator online each rollout. Brief "
                         "§5.2 first sweep: 0.3 / 0.5 / 0.7.")
    ap.add_argument("--amp-task-weight", type=float, default=1.0,
                    help="task-reward weight in the AMP blend (only "
                         "read when --amp-style-weight > 0). Brief "
                         "§5.2 mixtures pair 0.7/0.3, 0.5/0.5, 0.3/0.7 "
                         "— note the brief's task reward is its own "
                         "compact §5.1 kernel set, so size this to the "
                         "env reward actually configured.")
    ap.add_argument("--amp-motion-lib", type=str, default=None,
                    help="motion-library npz for the discriminator's "
                         "real transitions + feature normalization + "
                         "neutral pose (default: teacher_v1.npz).")
    ap.add_argument("--amp-disc-lr", type=float, default=3e-4)
    ap.add_argument("--amp-disc-steps", type=int, default=4,
                    help="discriminator minibatch updates per PPO "
                         "rollout (rollout-end callback).")
    ap.add_argument("--amp-disc-batch", type=int, default=512)
    ap.add_argument("--amp-gp-weight", type=float, default=10.0,
                    help="R1 gradient-penalty weight (brief §16: 10).")
    ap.add_argument("--amp-replay", type=int, default=500_000,
                    help="policy-transition ring replay rows (the "
                         "discriminator's fake side).")
    ap.add_argument("--amp-disc-init", type=str, default=None,
                    help="warm-start the discriminator from a prior "
                         "run's <out>.amp_disc.pt (continuations).")
    ap.add_argument("--log-std-init", type=float, default=-1.0)
    ap.add_argument("--warm-log-std-override", type=float, default=None,
                    help="after a --init-from warm start loads the "
                         "checkpoint's OWN log_std parameter(s), "
                         "forcibly RESET them to this value (log-space; "
                         "std=exp(value)) before training starts. Only "
                         "applies with --init-from; a no-op otherwise "
                         "(--log-std-init only affects from-scratch/"
                         "transplant builds, never a plain warm start, "
                         "so there was previously no way to start a "
                         "warm-started fine-tune below the parent's own "
                         "std). Default None = OFF, bit-exact (parent's "
                         "log_std kept, current default behavior). "
                         "Motivated by cw-dep-bcgait4-phasedir4-"
                         "entanneal (08-22): --ent-coef-final annealed "
                         "ent_coef 10x (0.000951->0.0001, confirmed via "
                         "wandb ent_coef_anneal/value) but the warm-"
                         "started log_std barely moved (std 0.368->"
                         "0.352 over the full 2M-step run) because the "
                         "entropy bonus is too small a fraction of the "
                         "total PPO loss to meaningfully drag it down "
                         "in 2M steps -- a direct override is a "
                         "guaranteed-to-move lever instead of a slow, "
                         "indirect one. Works on plain MLP policies "
                         "(policy.log_std) and gru-experts policies "
                         "(all four policy._log_stds() reset "
                         "identically); a policy with neither attribute "
                         "prints a warning and is left untouched.")
    ap.add_argument("--log-std-final", type=float, default=None,
                    help="linearly anneal the policy's log_std "
                         "PARAMETER down to this value (log-space) over "
                         "--log-std-anneal-frac of --steps, then hold — "
                         "the anneal FORCIBLY re-sets the parameter at "
                         "every rollout end (the --warm-log-std-"
                         "override pattern applied on a schedule), so "
                         "unlike --ent-coef-final it is guaranteed to "
                         "move. Start value = --warm-log-std-override "
                         "when set, else the policy's own mean log_std "
                         "at launch. Default None = OFF, bit-exact "
                         "legacy (no callback constructed). Motivated "
                         "by the cw-dep-bcgait4-phasedir8 dig-in "
                         "(08-22, logs/ckpt_eval/pd8_digin_regime/): "
                         "per-stance drag pricing has NO separating "
                         "allowance in the NOISY optimization regime "
                         "(honest clone at std 0.135 needs allow>=48mm "
                         "untaxed while the pd6 det drag cheat pays "
                         "ZERO beyond 36mm) — the repair is to make "
                         "the optimization regime CONVERGE to the det "
                         "regime where the full-stack pricing is "
                         "measured-aligned (clone 1031 > slow 978 > "
                         "drag 639), i.e. anneal exploration noise "
                         "away instead of widening det-blind bands.")
    ap.add_argument("--log-std-anneal-frac", type=float, default=1.0,
                    help="fraction of --steps over which the log-std "
                         "anneal completes (only used when "
                         "--log-std-final is set); 1.0 = whole run.")
    ap.add_argument("--net-arch", type=str, default="128,128",
                    help="MLP hidden sizes, comma-separated (from-"
                         "scratch and transplant builds only — a plain "
                         "--init-from warm start keeps the parent's "
                         "stored architecture). Flagship unified-policy "
                         "arms use 256,256 (RL_PLAN Architecture).")
    ap.add_argument("--gru", action="store_true",
                    help="recurrent GRU actor-critic (sb3-contrib "
                         "RecurrentPPO + gru_policy.py) instead of the "
                         "frame-stack MLP; run single-frame obs "
                         "(obs.history_frames=1). BPTT window = "
                         "--n-steps, so recurrent runs want longer "
                         "rollouts (e.g. --n-steps 64) than the MLP "
                         "large-batch default of 16. From-scratch or "
                         "GRU-parent warm starts only")
    ap.add_argument("--gru-hidden-size", type=int, default=128,
                    help="GRU hidden units per layer (actor and critic "
                         "each get their own single-layer GRU)")
    ap.add_argument("--gru-dual", action="store_true",
                    help="mode-gated dual-core GRU (gru_policy."
                         "DualGruActorCriticPolicy): separate locomotion"
                         " and stance cores+heads routed per tick by the"
                         " obs.mode_onehot tail — REQUIRES --cfg-set "
                         "obs.mode_onehot=1. Born from the cw-arch-gru-"
                         "anchor1..3 closure: one shared trunk cannot "
                         "hold anchored stance and a displacing walk at "
                         "once. Implies --gru")
    ap.add_argument("--gru-experts", action="store_true",
                    help="mode-gated FOUR-expert GRU (gru_policy."
                         "ModeExpertsGruActorCriticPolicy): fully "
                         "isolated rise/hold/lower/locomotion experts, "
                         "each with its own actor GRU, critic GRU, "
                         "heads and learnable log_std; only the active "
                         "expert's output/gradient is selected. "
                         "Operator directive fb_20260815T013349_488ffd. "
                         "Requires --cfg-set obs.mode_onehot=1. "
                         "Implies --gru; exclusive with --gru-dual")
    ap.add_argument("--gru-experts-adapter", type=int, default=0,
                    help="hidden width of the optional transition "
                         "adapter (zero-init residual MLP on the "
                         "selected action mean; 0 = no adapter module, "
                         "bit-exact expert-only path). Only with "
                         "--gru-experts")
    ap.add_argument("--gru-experts-adapter-scale", type=float,
                    default=0.05,
                    help="residual multiplier for the transition "
                         "adapter (default 0.05)")
    ap.add_argument("--gru-experts-freeze", action="store_true",
                    help="freeze all four expert ACTOR bodies (cores, "
                         "actor latents, action heads, per-expert "
                         "log_std); critics + transition adapter keep "
                         "training. Arm A stage 1 (frozen-expert "
                         "transition-adapter composition). Requires an "
                         "adapter on the policy")
    ap.add_argument("--transformer", action="store_true",
                    help="causal-transformer actor-critic (transformer_"
                         "policy.py) over the env-side frame stack: "
                         "attends over the obs.history_frames window "
                         "instead of flattening it into one MLP input. "
                         "REQUIRES --cfg-set obs.history_frames=K "
                         "(K>=2; the hist16 lineage uses 16). Stock "
                         "non-recurrent PPO — n_steps/batching identical "
                         "to the frame-stack MLP. From-scratch or "
                         "transformer-parent warm starts only")
    ap.add_argument("--tf-layers", type=int, default=2,
                    help="transformer encoder layers (per actor/critic)")
    ap.add_argument("--tf-width", type=int, default=128,
                    help="transformer d_model (token width)")
    ap.add_argument("--tf-heads", type=int, default=4,
                    help="attention heads")
    ap.add_argument("--tf-ff", type=int, default=256,
                    help="feed-forward hidden width inside each layer")
    ap.add_argument("--asym-critic", action="store_true",
                    help="asymmetric (privileged) actor-critic: mask the "
                         "privileged measured-velocity obs on the actor "
                         "path only, critic sees them (walk task). Port "
                         "of train_ppo_sim's --asym-critic "
                         "(asym_policy.AsymActorCriticPolicy) onto the "
                         "GPU/Warp trainer — AMP_LOCOMOTION.md M0 "
                         "'actor/critic observation split'. MLP-only "
                         "(mutually exclusive with --gru/--transformer/"
                         "--critic-encoder/--obs-pad-transplant); "
                         "from-scratch or asym-parent warm starts only.")
    ap.add_argument("--device", default="auto",
                    help="torch device for PPO (auto: cuda if available "
                         "— the big-batch MLP pays off on GPU)")
    ap.add_argument("--init-from", type=Path, default=None,
                    help="warm-start from a train_ppo_sim checkpoint "
                         "(same task/obs config)")
    ap.add_argument("--init-from-actor-only", action="store_true",
                    help="with --critic-encoder: copy ONLY the actor "
                         "weights from --init-from (e.g. a scripted-"
                         "gait BC-INIT or gait-hardened checkpoint "
                         "trained on the plain single-frame obs) into "
                         "a FRESH condition-D model — fresh critic, "
                         "fresh frozen-encoder residual, zero-padded "
                         "actor first-layer columns across the obs "
                         "widening (single frame -> history-stacked, "
                         "newest-first, so the extra dims are the "
                         "OLDER frames appended at the tail). The "
                         "critic (value_net/vf_*/value_gate/"
                         "latent_adapter) and the frozen predictor "
                         "snapshot are never read from --init-from. "
                         "Operator addendum fb_20260818T085834_588d9a "
                         "(walkcurr4 tournament arms B/C).")
    ap.add_argument("--init-from-policy-backbone", action="store_true",
                    help="with --predictive-live: transplant the plain "
                         "source actor AND ordinary raw critic across the "
                         "history-stack widening; predictive adapters, "
                         "gates and transformer snapshot stay fresh")
    ap.add_argument("--obs-pad-transplant", type=int, default=0,
                    help="warm-start across an obs WIDENING of N dims "
                         "appended at the obs tail (e.g. walk phase "
                         "clock, +2); zero-pads first-layer columns so "
                         "the parent policy is bit-identical until "
                         "training moves them (port of the "
                         "train_ppo_sim mechanism)")
    ap.add_argument("--eval-every", type=int, default=1_000_000,
                    help="background per-mode eval on a C-MuJoCo env every "
                         "N steps (0 = off). Doubles as a continuous "
                         "MJX-vs-C behavioral A/B — the eval env is real "
                         "C physics. Default is 5x the campaign's 200k "
                         "because MJX runs ~10x the steps/s.")
    ap.add_argument("--video-every", type=int, default=2_000_000,
                    help="background telemetry-overlay video reel every "
                         "N steps (0 = off); rendered on a C-MuJoCo env")
    ap.add_argument("--video-episodes", type=int, default=4)
    ap.add_argument("--recover-cert-every", type=int, default=1_000_000,
                    help="deterministic same-backend MJX recovery "
                         "certification every N training env-steps; this "
                         "alone controls monotonic recovery curriculum "
                         "admission (0 = off)")
    ap.add_argument("--recover-cert-envs", type=int, default=8,
                    help="parallel deterministic MJX episodes per active "
                         "recovery start kind")
    ap.add_argument("--recover-retention-buckets", type=int, default=3,
                    help="rotating previously unlocked buckets assayed at "
                         "each recovery certification; the current weakest "
                         "old bucket is assayed in addition. Whenever the "
                         "frontier passes, every remaining unlocked bucket "
                         "is assayed before promotion (default: 3 routine)")
    ap.add_argument("--recover-full-retention-every", type=int, default=2,
                    help="run a fresh full unlocked-bucket retention suite "
                         "every N certification rounds in addition to every "
                         "promotion attempt (0 = promotion attempts only)")
    ap.add_argument("--recover-rollback-after-steps", type=int,
                    default=4_000_000,
                    help="restore the latest promotion checkpoint when the "
                         "same retained bucket remains severely regressed "
                         "for this many training env-steps (0 = disable)")
    ap.add_argument("--recover-rollback-fraction", type=float, default=0.60,
                    help="a retained bucket below this deterministic gate "
                         "fraction starts/continues its rollback timer")
    ap.add_argument("--recover-population-id", type=str, default=None,
                    help="shared id for synchronized best-of-N recovery "
                         "learners (requires W&B and --recover-population-"
                         "runs)")
    ap.add_argument("--recover-population-member", type=int, default=-1,
                    help="zero-based member index; member 0 elects the "
                         "first retention-clean promotion")
    ap.add_argument("--recover-population-runs", type=str, default=None,
                    help="comma-separated W&B run display names in member "
                         "order; all members load each elected checkpoint")
    ap.add_argument("--recover-population-run-ids", type=str, default=None,
                    help="comma-separated predeclared W&B run ids in member "
                         "order; required to avoid cached name discovery")
    ap.add_argument("--recover-population-poll-seconds", type=float,
                    default=20.0,
                    help="minimum W&B peer-poll interval at PPO rollout "
                         "boundaries")
    ap.add_argument("--recover-population-barrier-timeout-seconds", type=float,
                    default=900.0,
                    help="fail a cohort member if the all-ACK release barrier "
                         "does not complete within this wall time")
    ap.add_argument("--recover-population-bootstrap-rollouts", type=int,
                    default=10,
                    help="equal per-seed rollout budget before member 0 "
                         "releases the initial recovery race")
    ap.add_argument("--recover-init-curriculum", type=Path, default=None,
                    help="restore the recovery curriculum/frontier state "
                         "on every env at startup from a promotion "
                         "*.curriculum.json sidecar (exact-state "
                         "continuation of a finished recover run; "
                         "default off = fresh B0)")
    # -- all-GPU condition-D + walk curriculum (operator order
    # 2026-08-18, fb_20260818T065930_03b422; every flag default-off =
    # bit-exact legacy trainer) -------------------------------------
    ap.add_argument("--require-gpu-physics", action="store_true",
                    help="fail-closed guard: refuse to train unless the "
                         "training VecEnv is MjxVecEnv/MjxShardedVecEnv "
                         "with impl=warp AND Torch runs on CUDA "
                         "(_assert_gpu_physics)")
    ap.add_argument("--critic-encoder", type=Path, default=None,
                    help="condition-D decoupled predictive critic "
                         "(rl_move/dynamics/predictive_critic.py): "
                         "scratch-A actor + frozen dynamics-transformer "
                         "critic residual. Path to the pretrained "
                         "encoder checkpoint; requires "
                         "--critic-encoder-md5 and --cfg-set "
                         "obs.history_frames=16")
    ap.add_argument("--critic-encoder-md5", type=str, default=None,
                    help="md5 the encoder file must match (refuse to "
                         "train otherwise)")
    ap.add_argument("--anchor-data", type=str,
                    default="rl_move/dynamics/datasets/v5_mjx_fresh",
                    help="pretraining corpus for the predictive "
                         "critic's fixed probe batch + start-of-run "
                         "heldout reference (condition D reads, never "
                         "trains on it)")
    ap.add_argument("--predictive-actor", action="store_true",
                    help="feed the verified frozen predictive snapshot "
                         "to BOTH actor and critic through independent "
                         "zero-initialized residual gates; unlike "
                         "--predictive-live, the encoder never updates")
    ap.add_argument("--predictive-live", action="store_true",
                    help="train the dynamics transformer continuously on "
                         "live curriculum walking plus retained corpus "
                         "rehearsal, and feed its boundary-gated stable "
                         "snapshot to BOTH actor and critic")
    ap.add_argument("--pred-batch-size", type=int, default=256)
    ap.add_argument("--pred-steps-per-iter", type=int, default=8)
    ap.add_argument("--pred-lr", type=float, default=1e-4)
    ap.add_argument("--pred-rehearsal-frac", type=float, default=0.25)
    ap.add_argument("--pred-capture-envs", type=int, default=128,
                    help="MJX worlds harvested into live replay; bounded "
                         "to avoid retaining every 60-second episode")
    ap.add_argument("--pred-live-walk-frac", type=float, default=1.0)
    ap.add_argument("--pred-live-min-windows", type=int, default=512)
    ap.add_argument("--pred-snapshot-boundary-steps", type=int,
                    default=1_000_000)
    ap.add_argument("--pred-snapshot-drift-guard", type=float, default=0.05)
    ap.add_argument("--pred-gate-heldout-band", type=float, default=0.15)
    ap.add_argument("--pred-gate-live-improve", type=float, default=0.0)
    ap.add_argument("--pred-gate-rise-band", type=float, default=0.05)
    ap.add_argument("--pred-gate-value-jump", type=float, default=0.10)
    ap.add_argument("--pred-gate-action-kl", type=float, default=0.002)
    ap.add_argument("--pred-metrics-every", type=int, default=1_000_000,
                    help="live heldout composition/per-bin W&B cadence")
    ap.add_argument("--walk-curriculum", action="store_true",
                    help="adaptive competence+retention walk-command "
                         "curriculum (walk_task WALKCURR_BUCKETS*), "
                         "certified on THIS trainer's MJX backend; "
                         "sets cfg goal.walk_curriculum + "
                         "goal.walk_pure at env construction")
    ap.add_argument("--walk-curriculum-version", type=int, default=1,
                    choices=(1, 2, 3, 4, 5),
                    help="1 = WALKCURR_BUCKETS (walkcurr1), 2 = "
                         "WALKCURR_BUCKETS_V2 ignition ladder "
                         "(walkcurr2), 3 = WALKCURR_BUCKETS_V3 "
                         "actor-init bridge ladder (operator order "
                         "fb_20260818T102844_116d4c), 4 = sustained "
                         "joystick ladder (10/20/40/60-second retained "
                         "survival before DR/lateral/rear), 5 = fast "
                         "anti-skate ladder adjacent to bcgait1-hard1 "
                         "(operator order fb_20260820T075230_4a90c6; "
                         "strict slip/direction/height gates, pairs "
                         "with reward.k_loadslip_excess)")
    ap.add_argument("--walkcurr-cert-every", type=int, default=500_000,
                    help="deterministic certification cadence (steps)")
    ap.add_argument("--walkcurr-cert-episodes", type=int, default=8,
                    help="held-out episodes per bucket per cert round "
                         "(= cert vec-env size)")
    ap.add_argument("--walkcurr-post-promo-epochs", type=int, default=0,
                    help="frontier-gated update schedule (default 0 = "
                         "OFF, bit-exact): on the walk curriculum's "
                         "FIRST promotion switch PPO n_epochs to this "
                         "value (operator order "
                         "fb_20260818T085648_2a0a60: acquisition-"
                         "strength updates until B0 certifies, then "
                         "the proven consolidation recipe)")
    ap.add_argument("--walkcurr-post-promo-actor-lr", type=float,
                    default=0.0,
                    help="frontier-gated schedule (0 = OFF): on the "
                         "first promotion set the update_health actor "
                         "group to this LR (requires --actor-lr)")
    ap.add_argument("--walkcurr-post-promo-actor-lr-final", type=float,
                    default=0.0,
                    help="linear-decay target for the post-promotion "
                         "actor LR over the remaining run (0 = "
                         "constant at --walkcurr-post-promo-actor-lr)")
    ap.add_argument("--walkcurr-fail-streak", type=int, default=2,
                    help="consecutive retained-failure rounds before "
                         "rollback to the last promotion")
    ap.add_argument("--actor-freeze-steps", type=int, default=0,
                    help="freeze the update_health ACTOR param group "
                         "(lr=0) for the first N env-steps so a fresh "
                         "critic adapts to a transplanted actor "
                         "without erasing it; 0 = off, bit-exact "
                         "(requires --actor-lr; operator order "
                         "fb_20260818T102844_116d4c)")
    ap.add_argument("--walkcurr-cert-at-init", action="store_true",
                    help="run the frontier bucket's exact "
                         "deterministic certification on the INITIAL "
                         "policy, before any PPO update, and log it "
                         "(walkcurr/pre_b0_*); aborts if the initial "
                         "policy falls, or under-tracks "
                         "--walkcurr-precert-min-prog (fix the "
                         "transplant/obs mapping instead of training "
                         "over it — fb_20260818T102844_116d4c item 6)")
    ap.add_argument("--walkcurr-precert-min-prog", type=float,
                    default=0.0,
                    help="minimum pre-PPO b0 cmd_prog_frac before "
                         "training may start (0 = survival-only bar; "
                         "requires --walkcurr-cert-at-init)")
    ap.add_argument("--walkcurr-precert-only", action="store_true",
                    help="exit (rc 0 pass / 3 fail) right after the "
                         "pre-PPO certification — the on-pod preflight "
                         "mode; requires --walkcurr-cert-at-init")
    ap.add_argument("--walkcurr-precert-buckets", type=int, default=1,
                    help="how many curriculum buckets (b0..bN-1) the "
                         "pre-PPO certification assays; b0 keeps the "
                         "abort bar, b1+ are logged/summary-only "
                         "(walkcurr/pre_bN_*; bridge2 spec "
                         "fb_20260818T112826_9ed832 item 2 — requires "
                         "--walkcurr-cert-at-init)")
    ap.add_argument("--walkcurr-actor-only-rollback",
                    action="store_true",
                    help="curriculum retention rollback restores ONLY "
                         "the actor tensors of the promotion "
                         "checkpoint (critic/value head, predictive "
                         "residual, frozen encoder and the critic "
                         "optimizer moments are never reset; actor "
                         "Adam moments restart fresh). Default off = "
                         "bit-exact whole-policy rollback "
                         "(fb_20260818T112826_9ed832 item 1)")
    ap.add_argument("--actor-freeze-ev-threshold", type=float,
                    default=0.0,
                    help="arm the critic-EV readiness gate on the "
                         "actor freeze: the actor stays frozen past "
                         "--actor-freeze-steps until train/"
                         "explained_variance >= this for "
                         "--actor-freeze-ev-windows consecutive "
                         "updates (0 = off, bit-exact fixed-step "
                         "freeze; fb_20260818T112826_9ed832 item 2)")
    ap.add_argument("--actor-freeze-ev-windows", type=int, default=3,
                    help="consecutive updates the critic EV must hold "
                         "the readiness threshold before the freeze "
                         "may release")
    ap.add_argument("--actor-freeze-max-steps", type=int, default=0,
                    help="fail-closed cap: abort if the readiness "
                         "gate has not released by this many env-"
                         "steps (required >0 when "
                         "--actor-freeze-ev-threshold is set)")
    ap.add_argument("--no-canary", action="store_true",
                    help="disable the fixed-seed canary probes + "
                         "regression auto-stop (on by default for warm "
                         "starts, same as the campaign trainer)")
    ap.add_argument("--canary-stop-after", type=int, default=3,
                    help="consecutive full-group canary failures before "
                         "auto-stop (0 = monitor only)")
    ap.add_argument("--run-name", type=str, default=None)
    ap.add_argument("--notes", type=str, default=None)
    ap.add_argument("--out-name", type=str, default=None)
    ap.add_argument("--save-every", type=int, default=1_000_000,
                    help="checkpoint every N env-steps (0 = end only)")
    ap.add_argument("--no-wandb", action="store_true")
    ap.add_argument("--smoke", action="store_true",
                    help="tiny CPU run to validate the pipeline")
    args = ap.parse_args(argv)

    if args.recover_full_retention_every < 0:
        ap.error("--recover-full-retention-every must be >= 0")
    if args.recover_rollback_after_steps < 0:
        ap.error("--recover-rollback-after-steps must be >= 0")
    if not 0.0 <= args.recover_rollback_fraction <= 1.0:
        ap.error("--recover-rollback-fraction must be in [0, 1]")
    population_runs = [
        name.strip() for name in (args.recover_population_runs or "").split(",")
        if name.strip()]
    population_run_ids = [
        run_id.strip()
        for run_id in (args.recover_population_run_ids or "").split(",")
        if run_id.strip()]
    if args.recover_population_id:
        if args.no_wandb:
            ap.error("--recover-population-id requires W&B")
        if len(population_runs) < 2:
            ap.error("--recover-population-runs needs at least two runs")
        if len(set(population_runs)) != len(population_runs):
            ap.error("--recover-population-runs contains duplicates")
        if len(population_run_ids) != len(population_runs):
            ap.error("--recover-population-run-ids must provide one "
                     "predeclared id per roster member")
        if len(set(population_run_ids)) != len(population_run_ids):
            ap.error("--recover-population-run-ids contains duplicates")
        if any("/" in run_id for run_id in population_run_ids):
            ap.error("--recover-population-run-ids cannot contain '/'")
        if not 0 <= args.recover_population_member < len(population_runs):
            ap.error("--recover-population-member is outside the roster")
        if args.run_name != population_runs[args.recover_population_member]:
            ap.error("--run-name must match this member's population roster "
                     "entry")
        if args.recover_cert_every <= 0 or args.recover_cert_envs <= 0:
            ap.error("recovery population sync requires deterministic "
                     "recovery certification")
        if float(_parse_goal_mix(args.goal_mix).get("recover", 0.0)) <= 0.0:
            ap.error("recovery population sync requires recover episodes")
        if args.recover_population_poll_seconds <= 0.0:
            ap.error("--recover-population-poll-seconds must be > 0")
        if args.recover_population_barrier_timeout_seconds <= 0.0:
            ap.error("--recover-population-barrier-timeout-seconds must be > 0")
        if args.recover_population_bootstrap_rollouts <= 0:
            ap.error("--recover-population-bootstrap-rollouts must be > 0")
        bootstrap_steps = (
            args.n_envs * args.n_steps
            * args.recover_population_bootstrap_rollouts)
        if bootstrap_steps >= args.recover_cert_every:
            ap.error("recovery population bootstrap budget must be smaller "
                     "than --recover-cert-every")
    elif (args.recover_population_runs or args.recover_population_run_ids
          or args.recover_population_member >= 0):
        ap.error("population roster/member requires --recover-population-id")

    if args.recover_init_curriculum is not None:
        if not args.recover_init_curriculum.is_file():
            ap.error("--recover-init-curriculum: no such file "
                     f"{args.recover_init_curriculum}")
        if float(_parse_goal_mix(args.goal_mix).get("recover", 0.0)) <= 0.0:
            ap.error("--recover-init-curriculum requires recover episodes "
                     "in --goal-mix")

    if not mjx_is_available():
        raise SystemExit("mujoco-mjx / jax not installed — "
                         "pip install -r rl_move/sim/requirements-mjx.txt")
    if args.smoke:
        args.n_envs = 4
        args.steps = 256
        args.n_steps = 8
        args.batch_size = 32
        args.episode_seconds = min(args.episode_seconds, 5.0)
        args.save_every = 0
        args.eval_every = 0
        args.video_every = 0
        args.recover_cert_every = 0
        args.device = "cpu"
    # Canaries ride the periodic C-env eval (campaign parity): on by
    # default for warm starts, since the failure class is a warm-started
    # run silently destroying a parent skill. Read by the bg-eval child.
    args.canary = (bool(args.init_from) and not args.no_canary
                   and args.eval_every > 0)

    # -- walk curriculum + condition-D wiring (fb_20260818T065930) ----
    if args.walk_curriculum:
        if args.task != "joint_walk":
            raise SystemExit("--walk-curriculum requires --task "
                             "joint_walk (the curriculum owns the walk "
                             "command distribution)")
        if args.goal_mix:
            raise SystemExit(
                "--walk-curriculum owns the whole command distribution "
                "(cfg goal.walk_pure=1 is set at env CONSTRUCTION); "
                "drop --goal-mix")
        if args.best_ckpt or args.ev_stop_min > 0.0:
            raise SystemExit(
                "--walk-curriculum owns best-checkpoint selection "
                "(best = last retention-clean promotion, never reward/"
                "latest); drop --best-ckpt/--ev-stop-min")
        if (args.init_from is not None and not args.init_from_actor_only
                and not args.init_from_policy_backbone
                and args.walk_curriculum_version != 5):
            raise SystemExit("--walk-curriculum is a fresh-actor "
                             "acquisition contract (walkcurr lineage); "
                             "a full-checkpoint --init-from is not wired "
                             "(pass --init-from-actor-only for an "
                             "actor-init acquisition recipe — the "
                             "curriculum still owns cert/reset-pool/"
                             "promotion state fresh; EXCEPTION: V5 is "
                             "an adjacent-continuation ladder by "
                             "operator order fb_20260820T075230_4a90c6 "
                             "— warm full-checkpoint --init-from is "
                             "allowed there, curriculum state still "
                             "starts fresh)")
        if (args.walkcurr_post_promo_actor_lr > 0.0
                and args.actor_lr <= 0.0):
            raise SystemExit("--walkcurr-post-promo-actor-lr requires "
                             "--actor-lr (it retargets the "
                             "update_health actor param group)")
        if args.walk_curriculum_version in (4, 5):
            if args.walk_curriculum_version == 5:
                from .walk_task import WALKCURR_BUCKETS_V5 as _wc_tbl
            else:
                from .walk_task import WALKCURR_BUCKETS_V4 as _wc_tbl
            required_s = max(float(b["duration_s"]) for b in _wc_tbl)
            args.training_episode_seconds = max(
                float(args.episode_seconds), required_s)
            print(f"[walkcurr] V{args.walk_curriculum_version} "
                  "training/cert horizon: "
                  f"{args.training_episode_seconds:g}s maximum; plain "
                  f"background eval remains {args.episode_seconds:g}s")
        # Pure-walk diet + curriculum version ride the env cfg so the
        # shim envs are born with them (no post-construction mutation;
        # the MJX vec envs mint reset pools during reset()). The
        # injection is reverted right after the TRAINING env kwargs are
        # built: the background C-env eval/video worker reads
        # args.cfg_set and must keep the PLAIN command distribution
        # (transfer-trainer parity — eval curves stay comparable to the
        # fixed-sampling parents, and an eval env must never run the
        # trainer-broadcast curriculum).
        _plain_cfg_set = list(args.cfg_set or [])
        args.cfg_set = _plain_cfg_set + [
            f"goal.walk_curriculum={args.walk_curriculum_version}",
            "goal.walk_pure=1",
        ]
        print("[walkcurr] cfg injected at construction: "
              f"goal.walk_curriculum={args.walk_curriculum_version}, "
              "goal.walk_pure=1")
    elif (args.walkcurr_post_promo_epochs > 0
          or args.walkcurr_post_promo_actor_lr > 0.0):
        raise SystemExit("--walkcurr-post-promo-* requires "
                         "--walk-curriculum (the trigger is the "
                         "curriculum's first promotion)")
    if args.actor_freeze_steps > 0 and args.actor_lr <= 0.0:
        raise SystemExit("--actor-freeze-steps requires --actor-lr "
                         "(it zeroes the update_health actor param "
                         "group)")
    if args.actor_freeze_ev_threshold > 0.0:
        if args.actor_freeze_steps <= 0:
            raise SystemExit("--actor-freeze-ev-threshold requires "
                             "--actor-freeze-steps (it gates an armed "
                             "freeze window)")
        if args.actor_freeze_max_steps <= args.actor_freeze_steps:
            raise SystemExit("--actor-freeze-ev-threshold requires "
                             "--actor-freeze-max-steps > "
                             "--actor-freeze-steps (the fail-closed "
                             "cap)")
    elif args.actor_freeze_max_steps > 0:
        raise SystemExit("--actor-freeze-max-steps requires "
                         "--actor-freeze-ev-threshold (it caps the "
                         "readiness gate)")
    if args.walkcurr_actor_only_rollback and not args.walk_curriculum:
        raise SystemExit("--walkcurr-actor-only-rollback requires "
                         "--walk-curriculum (it changes the "
                         "curriculum's retention rollback)")
    if args.walkcurr_actor_only_rollback and args.actor_lr <= 0.0:
        raise SystemExit("--walkcurr-actor-only-rollback requires "
                         "--actor-lr (the actor/critic optimizer "
                         "split defines the actor group whose moments "
                         "it resets)")
    if (args.walkcurr_precert_buckets != 1
            and not args.walkcurr_cert_at_init):
        raise SystemExit("--walkcurr-precert-buckets requires "
                         "--walkcurr-cert-at-init")
    if args.walkcurr_precert_buckets < 1:
        raise SystemExit("--walkcurr-precert-buckets must be >= 1")
    if args.walkcurr_cert_at_init and not args.walk_curriculum:
        raise SystemExit("--walkcurr-cert-at-init requires "
                         "--walk-curriculum (it runs the curriculum's "
                         "own deterministic cert)")
    if ((args.walkcurr_precert_only
         or args.walkcurr_precert_min_prog > 0.0)
            and not args.walkcurr_cert_at_init):
        raise SystemExit("--walkcurr-precert-only/"
                         "--walkcurr-precert-min-prog require "
                         "--walkcurr-cert-at-init")
    if args.init_from_actor_only and args.critic_encoder is None:
        raise SystemExit("--init-from-actor-only requires "
                         "--critic-encoder (condition D actor-only "
                         "transfer; a plain run should just use "
                         "--init-from)")
    if args.init_from_actor_only and args.init_from is None:
        raise SystemExit("--init-from-actor-only requires --init-from")
    if args.init_from_policy_backbone:
        if not args.predictive_live:
            raise SystemExit("--init-from-policy-backbone requires "
                             "--predictive-live")
        if args.init_from is None:
            raise SystemExit("--init-from-policy-backbone requires "
                             "--init-from")
        if args.init_from_actor_only:
            raise SystemExit("--init-from-policy-backbone and "
                             "--init-from-actor-only are exclusive")
    if args.predictive_actor and args.critic_encoder is None:
        raise SystemExit("--predictive-actor requires --critic-encoder")
    if args.predictive_live:
        if args.critic_encoder is None:
            raise SystemExit("--predictive-live requires --critic-encoder "
                             "to seed the online/stable transformers")
        if not args.require_gpu_physics:
            raise SystemExit("--predictive-live requires "
                             "--require-gpu-physics (fail closed instead "
                             "of silently moving physics or learning to CPU)")
        if (args.task != "joint_walk" or not args.walk_curriculum
                or args.walk_curriculum_version != 4):
            raise SystemExit("--predictive-live currently requires "
                             "--task joint_walk --walk-curriculum "
                             "--walk-curriculum-version 4")
        if args.pred_capture_envs <= 0 or args.pred_capture_envs > args.n_envs:
            raise SystemExit("--pred-capture-envs must be in [1, --n-envs]")
        if not 0.0 <= args.pred_rehearsal_frac <= 1.0:
            raise SystemExit("--pred-rehearsal-frac must be in [0, 1]")
        if not 0.0 <= args.pred_live_walk_frac <= 1.0:
            raise SystemExit("--pred-live-walk-frac must be in [0, 1]")
        if args.pred_snapshot_boundary_steps <= 0:
            raise SystemExit("--pred-snapshot-boundary-steps must be > 0")
        if args.pred_gate_action_kl < 0.0:
            raise SystemExit("--pred-gate-action-kl must be >= 0")
    if args.critic_encoder is not None:
        if args.gru or args.gru_dual or args.gru_experts \
                or args.transformer:
            raise SystemExit("--critic-encoder uses a raw MLP policy plus "
                             "the pretrained dynamics transformer; drop "
                             "--gru*/--transformer (with --predictive-live, "
                             "the dynamics transformer conditions both "
                             "actor and critic)")
        if (args.init_from is not None and not args.init_from_actor_only
                and not args.init_from_policy_backbone):
            raise SystemExit("--critic-encoder warm start is not "
                             "wired for a full-checkpoint --init-from; "
                             "condition D trains a fresh critic — pass "
                             "--init-from-actor-only for the actor-only "
                             "transfer (operator addendum "
                             "fb_20260818T085834_588d9a)")
        if ((args.init_from_actor_only or args.init_from_policy_backbone)
                and args.obs_pad_transplant):
            _transplant_flag = ("--init-from-policy-backbone"
                                if args.init_from_policy_backbone
                                else "--init-from-actor-only")
            raise SystemExit(f"{_transplant_flag} does not compose "
                             "with --obs-pad-transplant (pick one obs-"
                             "widening transplant path)")
        if not args.critic_encoder_md5:
            raise SystemExit("--critic-encoder requires "
                             "--critic-encoder-md5 (refuse to train on "
                             "an unverified transformer)")
        enc_path = args.critic_encoder
        if not enc_path.is_absolute():
            enc_path = _PROTO / enc_path
        if not enc_path.exists():
            raise SystemExit(f"--critic-encoder {enc_path} not found")
        import hashlib
        got = hashlib.md5(enc_path.read_bytes()).hexdigest()
        if got != args.critic_encoder_md5.strip().lower():
            raise SystemExit(
                f"encoder md5 mismatch: {enc_path} is {got}, expected "
                f"{args.critic_encoder_md5} — wrong/corrupt pretrained "
                "transformer, refusing to train")
        print(f"[encoder] md5 verified: {got} ({enc_path.name})",
              flush=True)
        args.critic_encoder = enc_path

    impl = _resolve_impl(args.impl)
    if args.require_gpu_physics:
        if impl != "warp":
            raise SystemExit(
                f"--require-gpu-physics: resolved impl is "
                f"{impl or 'jax(default)'}, not warp — refusing "
                "(fb_20260818T065930_03b422)")
        import torch as _th
        if not _th.cuda.is_available():
            raise SystemExit("--require-gpu-physics: torch.cuda is not "
                             "available — refusing")
        if args.device in ("cpu",):
            raise SystemExit("--require-gpu-physics conflicts with "
                             "--device cpu")
    iters = args.mjx_iterations if args.mjx_iterations is not None \
        else (1 if impl == "warp" else 8)
    ls_iters = args.mjx_ls_iterations if args.mjx_ls_iterations is not None \
        else (4 if impl == "warp" else 8)

    if args.amp_style_weight > 0.0:
        # AMP style blend needs the env's per-tick obs_style emission
        # (sim_env._post_step, default off). Injected into cfg_set so
        # training, cert and eval/video workers all agree on the env
        # contract; the blend itself lives ONLY in the vec wrapper
        # below (eval harness scores raw task metrics, unchanged).
        args.cfg_set = list(args.cfg_set or []) + ["goal.amp_style_obs=1"]
        print(f"[amp-style] ON: task_w={args.amp_task_weight} "
              f"style_w={args.amp_style_weight} "
              f"lib={args.amp_motion_lib or 'teacher_v1.npz(default)'} "
              f"disc lr={args.amp_disc_lr} steps/rollout="
              f"{args.amp_disc_steps} batch={args.amp_disc_batch} "
              f"replay={args.amp_replay}")
    env_kw = _env_kwargs(args)      # resolves params via bus.servo_params
    if args.walk_curriculum:
        # training/cert envs keep the curriculum via env_kw's cfg; the
        # C-env eval/video worker (which reads args.cfg_set) gets the
        # plain distribution back (see the injection comment above)
        args.cfg_set = _plain_cfg_set
    params = env_kw["params"]
    _warn_if_defaults(params)
    # Resolved motor contract (fb_20260820T000059): printed from the
    # ACTUAL env params + cfg the shim envs are built from, so the pod
    # log records the enforced velocity ceiling, not the launch args.
    from .servo_model import motor_contract, motor_contract_line
    _contract = motor_contract(env_kw.get("cfg"), params=params,
                               backend=f"mjx_tickparams:{impl or 'mjx'}")
    print(motor_contract_line(_contract))
    args._motor_contract = _contract  # picked up by _init_wandb
    env_cls = ENV_CLASSES[args.task]
    if args.predictive_live:
        # Same joint-walk task with pool-safe frame/privileged-label hooks.
        # Only --pred-capture-envs rows emit per-tick arrays (enabled after
        # VecEnv construction), so long V4 horizons stay memory-bounded.
        from rl_move.dynamics.collector_env import DynrepCollectWalkEnv
        env_cls = DynrepCollectWalkEnv

    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import VecMonitor

    # Mirror-symmetry regularizer (cfg-gated, default off — see
    # rl_move/sim/mirror.py). Read via --cfg-set so respec --cfg can
    # tune it; the key also rides into the env cfg dict, harmlessly.
    mirror_coef = float(_parse_cfg_set(args.cfg_set).get(
        "train.mirror_loss_coef", 0.0) or 0.0)
    if mirror_coef > 0.0:
        from .mirror import attach_mirror, make_mirror_ppo_class
        algo_cls = make_mirror_ppo_class()
        print(f"[mjx-train] mirror symmetry loss ON (coef={mirror_coef})")
    else:
        algo_cls = PPO
    # Reference BC anchor (cfg-gated, default off — see
    # rl_move/sim/bc_anchor.py; rise lever (a), RL_PLAN queue 2a).
    # Composes with MirrorPPO if both are requested.
    bc_coef = float(_parse_cfg_set(args.cfg_set).get(
        "train.bc_anchor_coef", 0.0) or 0.0)
    if bc_coef > 0.0:
        from .bc_anchor import make_bc_anchor_ppo_class
        algo_cls = make_bc_anchor_ppo_class(algo_cls)
        print(f"[mjx-train] BC anchor loss ON (coef={bc_coef})")

    policy_cls: str | type = "MlpPolicy"
    extra_pk: dict = {}
    if args.gru_dual and args.gru_experts:
        raise SystemExit("--gru-dual and --gru-experts are exclusive")
    if args.gru_dual or args.gru_experts:
        args.gru = True
        if float(_parse_cfg_set(args.cfg_set).get(
                "obs.mode_onehot", 0.0)) <= 0.0:
            raise SystemExit(
                f"--gru-{'dual' if args.gru_dual else 'experts'} requires "
                "--cfg-set obs.mode_onehot=1 (the policy routes by the "
                "obs-tail skill one-hot)")
    if args.gru:
        if mirror_coef > 0.0:
            raise SystemExit("--gru + mirror loss is not implemented "
                             "(it wraps stock PPO)")
        if args.obs_pad_transplant:
            raise SystemExit("--gru + --obs-pad-transplant is not "
                             "implemented (recurrent weights don't "
                             "transplant from MLP checkpoints)")
        from sb3_contrib import RecurrentPPO
        from .gru_policy import (DualGruActorCriticPolicy,
                                 GruActorCriticPolicy,
                                 ModeExpertsGruActorCriticPolicy)
        algo_cls = RecurrentPPO
        if bc_coef > 0.0:
            # Recurrent BC anchor: pairs carry the rollout hidden state
            # and the aux step runs one GRU cell step from it (see
            # bc_anchor.py::_bc_policy_mean).
            from .bc_anchor import make_bc_anchor_ppo_class
            algo_cls = make_bc_anchor_ppo_class(RecurrentPPO)
        policy_cls = (ModeExpertsGruActorCriticPolicy if args.gru_experts
                      else DualGruActorCriticPolicy if args.gru_dual
                      else GruActorCriticPolicy)
        extra_pk = dict(lstm_hidden_size=args.gru_hidden_size)
        if args.gru_experts:
            extra_pk.update(
                experts_adapter_hidden=args.gru_experts_adapter,
                experts_adapter_scale=args.gru_experts_adapter_scale)
        cores_note = (" x4 isolated mode experts" if args.gru_experts
                      else " x2 mode-gated cores" if args.gru_dual
                      else "")
        print(f"[mjx-train] GRU policy: hidden {args.gru_hidden_size}"
              f"{cores_note}, "
              f"BPTT window = n_steps = {args.n_steps} "
              f"({args.n_steps / 25.0:.2f}s at 25 Hz)")
    if args.transformer:
        if args.gru:
            raise SystemExit("--transformer and --gru are mutually "
                             "exclusive (pick one memory mechanism)")
        if args.obs_pad_transplant:
            raise SystemExit("--transformer + --obs-pad-transplant is not "
                             "implemented (transformer weights don't "
                             "transplant from MLP checkpoints)")
        hist = int(float(_parse_cfg_set(args.cfg_set).get(
            "obs.history_frames", 1)))
        if hist < 2:
            raise SystemExit(
                "--transformer needs the env-side frame stack: add "
                "--cfg-set obs.history_frames=K (K>=2; hist16 lineage "
                "uses 16)")
        from .transformer_policy import TransformerActorCriticPolicy
        policy_cls = TransformerActorCriticPolicy
        extra_pk = dict(n_frames=hist, d_model=args.tf_width,
                        n_layers=args.tf_layers, n_heads=args.tf_heads,
                        ff_dim=args.tf_ff)
        print(f"[mjx-train] transformer policy: {args.tf_layers} layers, "
              f"d_model {args.tf_width}, {args.tf_heads} heads, "
              f"ff {args.tf_ff}, context {hist} frames "
              f"({hist / 25.0:.2f}s at 25 Hz), separate actor/critic")

    if args.asym_critic:
        # Privileged (asymmetric) critic, ported from train_ppo_sim.py
        # (AMP_LOCOMOTION.md M0: "actor/critic observation split
        # works" — the GPU/Warp trainer had GRU/history/transformer
        # already but no obs-masked critic). Same policy class
        # (asym_policy.AsymActorCriticPolicy); only the vec-env backend
        # differs, so this is a straight wiring port, not a rewrite.
        # privileged_idx needs the constructed venv's obs width, which
        # doesn't exist yet here — filled in right after venv comes up,
        # mirroring --critic-encoder's post-venv frame_width patch
        # below.
        if args.gru or args.transformer or args.critic_encoder is not None:
            raise SystemExit(
                "--asym-critic is MLP-actor only (mirrors train_ppo_"
                "sim's --gru + --asym-critic restriction); drop "
                "--gru/--transformer/--critic-encoder")
        if args.obs_pad_transplant:
            raise SystemExit(
                "--obs-pad-transplant + --asym-critic is not "
                "implemented (privileged_idx would shift); do one at "
                "a time")
        from .asym_policy import AsymActorCriticPolicy
        policy_cls = AsymActorCriticPolicy
        print("[mjx-train] asym (privileged) critic ON — obs split "
              "resolved after venv construction")

    if args.critic_encoder is not None:
        # Condition-D decoupled predictive critic (operator order
        # 2026-08-18 fb_20260818T065930_03b422: port the exact frozen
        # vt2ovznc critic-D policy onto batched GPU physics). The
        # policy/PPO classes are backend-agnostic torch; construction
        # matches train_ppo_transfer's condition-D branch bit-for-bit
        # (scratch-A actor init at the same seed, frozen snapshot
        # inside the policy, zero-init value gate).
        if args.gru or args.transformer:
            raise SystemExit("--critic-encoder is MLP-actor only")
        if mirror_coef > 0.0:
            raise SystemExit("--critic-encoder does not compose with "
                             "mirror loss (untested optimizer "
                             "interaction); drop it")
        hist = int(float(_parse_cfg_set(args.cfg_set).get(
            "obs.history_frames", 1)))
        if hist < 2:
            raise SystemExit(
                "--critic-encoder needs the env-side frame stack: add "
                "--cfg-set obs.history_frames=K (condition-D lineage "
                "uses 16)")
        from rl_move.dynamics.predictive_critic import (
            PredictiveCriticPPO, PredictiveCriticPolicy)
        algo_cls = PredictiveCriticPPO
        if bc_coef > 0.0:
            # Both wrappers perform a separate optimizer step via
            # cooperative super(): predictive PPO owns the ordinary PPO
            # update/snapshot invariant, then BCAnchorPPO applies the
            # existing recovery mentor loss. Frozen predictor tensors are
            # requires_grad=False and absent from the shared optimizer.
            from .bc_anchor import make_bc_anchor_ppo_class
            algo_cls = make_bc_anchor_ppo_class(algo_cls)
        policy_cls = PredictiveCriticPolicy
        extra_pk = dict(predictor_ckpt=str(args.critic_encoder),
                        history=hist,
                        actor_residual_enabled=(args.predictive_actor
                                                or args.predictive_live))
        # frame_width is set after venv construction.
        if args.predictive_live:
            pred_note = ("live online transformer + boundary-gated "
                         "actor/critic snapshot")
        elif args.predictive_actor:
            pred_note = "frozen actor+critic snapshot"
        else:
            pred_note = "frozen critic-only snapshot"
        print(f"[mjx-train] predictive representation: {pred_note}; "
              f"seed {args.critic_encoder.name} "
              f"(md5 {args.critic_encoder_md5}), history {hist}")

    print(f"[mjx-train] task={args.task} n_envs={args.n_envs} "
          f"impl={impl or 'jax(default)'} iterations={iters}/{ls_iters} "
          f"host_workers={args.host_workers or 'in-process'} "
          f"rollout={args.n_envs * args.n_steps}/update "
          f"servo_params={Path(params.source).name}")
    t0 = time.monotonic()
    vec_kw = dict(env_kwargs=env_kw, seed=args.seed,
                  impl=impl, pool_per_env=args.pool_per_env,
                  mjx_iterations=iters, mjx_ls_iterations=ls_iters)
    if args.host_workers > 0:
        from .mjx_sharded_vec_env import MjxShardedVecEnv
        venv = MjxShardedVecEnv(env_cls, args.n_envs,
                                host_workers=args.host_workers, **vec_kw)
    else:
        from .mjx_vec_env import MjxVecEnv
        venv = MjxVecEnv(env_cls, args.n_envs, **vec_kw)
    gm = _parse_goal_mix(args.goal_mix)
    if gm:
        if not hasattr(env_cls, "set_goal_mix"):
            raise SystemExit(f"--goal-mix needs a goal task, not {args.task}")
        venv.env_method("set_goal_mix", gm)
    amp_wrap = None
    if args.amp_style_weight > 0.0:
        # Under VecMonitor so ep_rew_mean reflects the BLENDED reward
        # (the actual PPO training signal). Style reward is computed on
        # the host from info["amp_obs_style"] — worker/backend agnostic.
        from .amp_style_vec import AMPStyleVecWrapper
        amp_wrap = AMPStyleVecWrapper(
            venv, style_weight=args.amp_style_weight,
            task_weight=args.amp_task_weight,
            motion_lib=args.amp_motion_lib, replay_size=args.amp_replay,
            disc_lr=args.amp_disc_lr, gp_weight=args.amp_gp_weight,
            seed=args.seed, disc_init=args.amp_disc_init)
        venv = amp_wrap
    venv = VecMonitor(venv)
    print(f"[mjx-train] vec env up in {time.monotonic() - t0:.1f}s "
          f"(compile + {args.pool_per_env + 1} reset choreographies)")
    if args.require_gpu_physics:
        _assert_gpu_physics(venv, impl)

    # Servo-profile RAMP-IN (08-20, fast anti-skate option (b),
    # q_20260820T0830Z): bus.profile_ramp_steps > 0 anneals the live
    # write profile from the fitted regime to the cfg target dose over
    # that many global env steps. Armed HERE (frac 0 before anything
    # measures the policy — the V5 pre-PPO B0 cert then guards the
    # transplant at the ramp START instead of dying zero-shot at the
    # full dose); advanced once per rollout by _ProfileRampCb below;
    # the walkcurr cert env mirrors the training frac so promotions
    # certify the dose actually being trained. Envs constructed from a
    # ramp cfg that never receive a broadcast (eval_checkpoint, play,
    # the periodic C-env evals) sit at the TARGET dose by design.
    _prof_ramp_steps = 0
    if env_kw.get("cfg") is not None:
        from rl_move.config import cfg_get as _cfg_get
        _prof_ramp_steps = int(float(_cfg_get(
            env_kw["cfg"], "bus", "profile_ramp_steps",
            default=0) or 0))

    def _profile_ramp_frac_at(step: int) -> float:
        return min(1.0, float(step) / float(_prof_ramp_steps))

    def _profile_ramp_apply(target_venv, step: int) -> dict | None:
        """Broadcast the frac for ``step`` to a vec env; returns the
        applied profile (first env's row). No-op when the ramp is off."""
        if _prof_ramp_steps <= 0:
            return None
        f = _profile_ramp_frac_at(step)
        return target_venv.env_method("apply_profile_ramp_frac", f)[0]

    if _prof_ramp_steps > 0:
        r0 = _profile_ramp_apply(venv, 0)
        print(f"[profile-ramp] armed: {_prof_ramp_steps:,} global env "
              "steps from fitted profile to target dose; step-0 "
              f"profile: write_speed={r0['write_speed_counts_s']:.0f} "
              f"counts/s acc={r0['write_acc']:g} "
              f"max_delta_q={r0['max_delta_q_deg']:g} deg/tick")
        if _prof_ramp_steps >= args.steps:
            print("[profile-ramp] WARNING: profile_ramp_steps "
                  f"({_prof_ramp_steps:,}) >= --steps ({args.steps:,}) "
                  "— the policy will NEVER train at the full target "
                  "dose in this run")

    # Drag-stance allowance RAMP (08-22, phasedir9 seed-lottery dig-in
    # follow-up — see walk_task.py's __init__ block for the full
    # mechanism/why). Same cfg-armed / trainer-driven / default-OFF
    # contract as the profile ramp above; deliberately NOT mirrored
    # into the walkcurr certification env (that mechanism is
    # walkcurr-specific and unused by this lineage) — periodic
    # eval_checkpoint/C-env evals stay at the calibrated TARGET
    # allowance by construction (armed-but-unbroadcast override is
    # None, see walk_task.py).
    _da_ramp_steps = 0
    if env_kw.get("cfg") is not None:
        from rl_move.config import cfg_get as _cfg_get_da
        _da_ramp_steps = int(float(_cfg_get_da(
            env_kw["cfg"], "reward", "drag_stance_allow_ramp_steps",
            default=0) or 0))

    def _drag_allow_ramp_frac_at(step: int) -> float:
        return min(1.0, float(step) / float(_da_ramp_steps))

    def _drag_allow_ramp_apply(target_venv, step: int) -> dict | None:
        """Broadcast the frac for ``step`` to a vec env; returns the
        applied allowance. No-op when the ramp is off."""
        if _da_ramp_steps <= 0:
            return None
        f = _drag_allow_ramp_frac_at(step)
        return target_venv.env_method("apply_drag_allow_frac", f)[0]

    if _da_ramp_steps > 0:
        d0 = _drag_allow_ramp_apply(venv, 0)
        print(f"[drag-allow-ramp] armed: {_da_ramp_steps:,} global env "
              f"steps from a loose allowance to the cfg target; step-0 "
              f"allow={d0['allow_mm']:.1f}mm")
        if _da_ramp_steps >= args.steps:
            print("[drag-allow-ramp] WARNING: "
                  f"drag_stance_allow_ramp_steps ({_da_ramp_steps:,}) "
                  f">= --steps ({args.steps:,}) — the policy will "
                  "NEVER train at the target allowance in this run")

    # Termination-penalty RAMP (08-22, freeprog-term400-stall dig-in
    # follow-up — see walk_task.py's __init__ block for the mechanism).
    # Same cfg-armed / trainer-driven / default-OFF contract.
    _tp_ramp_steps = 0
    if env_kw.get("cfg") is not None:
        from rl_move.config import cfg_get as _cfg_get_tp
        _tp_ramp_steps = int(float(_cfg_get_tp(
            env_kw["cfg"], "reward", "term_penalty_ramp_steps",
            default=0) or 0))

    def _term_penalty_ramp_frac_at(step: int) -> float:
        return min(1.0, float(step) / float(_tp_ramp_steps))

    def _term_penalty_ramp_apply(target_venv, step: int) -> dict | None:
        if _tp_ramp_steps <= 0:
            return None
        f = _term_penalty_ramp_frac_at(step)
        return target_venv.env_method("apply_term_penalty_frac", f)[0]

    if _tp_ramp_steps > 0:
        # NOTE: must not be named t0 — that shadows the outer wall-clock
        # timer (t0 = time.monotonic() near the top of this function),
        # corrupting the final `dt = time.monotonic() - t0` computation
        # for the rest of the run (caught on-pod smoke, 08-22).
        _tp0 = _term_penalty_ramp_apply(venv, 0)
        print(f"[term-penalty-ramp] armed: {_tp_ramp_steps:,} global "
              "env steps from a lenient termination charge to the cfg "
              f"target; step-0 term_penalty={_tp0['term_penalty']:.1f}")
        if _tp_ramp_steps >= args.steps:
            print("[term-penalty-ramp] WARNING: "
                  f"term_penalty_ramp_steps ({_tp_ramp_steps:,}) >= "
                  f"--steps ({args.steps:,}) — the policy will NEVER "
                  "train at the full deterrent in this run")
    if args.predictive_live:
        capture_indices = list(range(args.pred_capture_envs))
        venv.env_method("dynrep_capture_enable", True,
                        indices=capture_indices)
        print("[dynrep-live] capture armed on "
              f"{len(capture_indices)}/{args.n_envs} MJX worlds; "
              "physics, PPO, replay tensors and transformer updates use GPU")
    if args.critic_encoder is not None:
        n_obs = int(np.prod(venv.observation_space.shape))
        hist = int(extra_pk["history"])
        if n_obs % hist:
            raise SystemExit(f"obs dim {n_obs} not divisible by "
                             f"history {hist} — obs.history_frames "
                             "mismatch with the env stack")
        extra_pk["frame_width"] = n_obs // hist
    if args.asym_critic:
        # Same helper train_ppo_sim uses (obs.history_frames/walk_phase_
        # obs/walk_yaw_cmd/mode_onehot tail accounting) — reused, not
        # duplicated, so the two trainers can never disagree about which
        # obs dims are privileged.
        from .train_ppo_sim import _privileged_idx
        n_obs = int(np.prod(venv.observation_space.shape))
        extra_pk["privileged_idx"] = _privileged_idx(args, n_obs)
        print(f"[mjx-train] asym-critic privileged idx "
              f"(obs width {n_obs}): {extra_pk['privileged_idx']}")
    if args.walk_curriculum:
        # Construction proof: every training env must be born pure-walk
        # with the curriculum armed (fail-closed; _walkcurr_prepare_
        # episode re-asserts p_walk per episode on every backend).
        _wc_states = venv.env_method("walkcurr_state", indices=0)
        print("[walkcurr] armed at construction: "
              f"{_wc_states[0]['total_buckets']} buckets, active_n="
              f"{_wc_states[0]['active_n']} (frontier B"
              f"{_wc_states[0]['frontier_bucket']})")
        # Realized-DR proof line (operator order
        # fb_20260818T085648_2a0a60: never silently train B0 at the
        # CLI --dr-scale): each episode's DomainRandomizer is REBUILT
        # from the drawn bucket's dr field
        # (walk_task._walkcurr_prepare_episode), so the bucket table
        # below — not --dr-scale — is the DR the curriculum realizes.
        from .walk_task import WALKCURR_BUCKETS as _WC1
        from .walk_task import WALKCURR_BUCKETS_V2 as _WC2
        from .walk_task import WALKCURR_BUCKETS_V3 as _WC3
        from .walk_task import WALKCURR_BUCKETS_V4 as _WC4
        from .walk_task import WALKCURR_BUCKETS_V5 as _WC5
        _tbl = {1: _WC1, 2: _WC2, 3: _WC3, 4: _WC4,
                5: _WC5}[args.walk_curriculum_version]
        print("[walkcurr] realized per-bucket DR (overrides --dr-scale "
              f"{args.dr_scale:g} per episode): "
              + " ".join(f"b{i}={row['dr']:g}"
                         for i, row in enumerate(_tbl)))

    run = _init_wandb(args, params)
    if args.recover_population_id and run is None:
        raise SystemExit(
            "recovery population sync could not initialize W&B; refusing "
            "to start an unsynchronized cohort member")

    # Checkpoint lineage via W&B artifacts (operator, 08-09): declare
    # the parent checkpoint as an input so the W&B artifact DAG shows
    # run/checkpoint ancestry. Best-effort — parents trained before
    # this feature have no artifact, and that must never block a run.
    if run is not None and args.init_from is not None:
        try:
            import wandb
            run.use_artifact(f"ckpt-{args.init_from.stem}:latest")
            print(f"[wandb] lineage: consumes ckpt-{args.init_from.stem}")
        except Exception:
            print(f"[wandb] no artifact for parent {args.init_from.stem} "
                  "(pre-artifact lineage) — continuing")

    tb_dir = None if run is None else str(POLICY_DIR / "tb")
    net_arch = [int(x) for x in str(args.net_arch).split(",") if x.strip()]
    if args.init_from is not None and (
            args.init_from_actor_only or args.init_from_policy_backbone):
        # Condition-D actor-only transfer (operator addendum
        # fb_20260818T085834_588d9a, walkcurr4 tournament arms B/C):
        # build the model EXACTLY as the fresh (no-init-from) branch
        # below would — fresh critic, fresh zero-gated predictive
        # residual, scratch-A actor init — then overwrite ONLY the
        # actor-marked tensors with the source checkpoint's weights.
        from rl_move.dynamics.predictive_critic import (
            actor_only_transplant, raw_policy_backbone_transplant)
        model = algo_cls(
            policy_cls, venv,
            n_steps=args.n_steps, batch_size=args.batch_size,
            n_epochs=args.n_epochs, learning_rate=args.lr,
            gamma=(0.99 if args.gamma is None else args.gamma),
            gae_lambda=(0.95 if args.gae_lambda is None
                        else args.gae_lambda),
            ent_coef=args.ent_coef,
            clip_range=0.2,
            target_kl=(args.target_kl if args.target_kl > 0 else None),
            policy_kwargs=dict(net_arch=net_arch,
                               log_std_init=args.log_std_init,
                               **extra_pk),
            seed=args.seed, verbose=1, device=args.device,
            tensorboard_log=tb_dir)
        old = PPO.load(args.init_from, device="cpu")
        copied = (raw_policy_backbone_transplant(old, model)
                  if args.init_from_policy_backbone
                  else actor_only_transplant(old, model))
        del old
        scope = ("raw actor+critic backbone"
                 if args.init_from_policy_backbone else "actor-only")
        print(f"[mjx-train] {scope} transplant from {args.init_from}: "
              f"{len(copied)} tensors copied ({copied}); predictive "
              "adapters/gates + transformer snapshot untouched")
    elif args.init_from is not None:
        if args.obs_pad_transplant:
            # Obs-widening warm start (port of train_ppo_sim's
            # --obs-pad-transplant): parent weights copy exactly, the
            # policy/value first layers gain N zero columns for the new
            # tail dims. Optimizer state is fresh (architecture changed).
            from .train_ppo_sim import pad_obs_transplant
            old = PPO.load(args.init_from, device="cpu")
            model = algo_cls(
                "MlpPolicy", venv,
                n_steps=args.n_steps, batch_size=args.batch_size,
                n_epochs=args.n_epochs, learning_rate=args.lr,
                gamma=(0.99 if args.gamma is None else args.gamma),
                gae_lambda=(0.95 if args.gae_lambda is None
                            else args.gae_lambda),
                ent_coef=args.ent_coef,
                clip_range=0.2,
                target_kl=(args.target_kl if args.target_kl > 0
                           else None),
                policy_kwargs=dict(net_arch=net_arch,
                                   log_std_init=args.log_std_init),
                seed=args.seed, verbose=1, device=args.device,
                tensorboard_log=tb_dir)
            pad_obs_transplant(old, model, args.obs_pad_transplant)
            model.num_timesteps = old.num_timesteps
            del old
            print(f"[mjx-train] warm start from {args.init_from} "
                  f"(+{args.obs_pad_transplant} obs-pad transplant)")
        else:
            from .gru_policy import is_recurrent_checkpoint
            if args.gru and not is_recurrent_checkpoint(args.init_from):
                raise SystemExit(
                    "--gru cannot warm-start from an MLP checkpoint "
                    f"({args.init_from}); GRU runs start from scratch "
                    "or from a previous GRU checkpoint")
            _ld_kw = {}
            if args.gamma is not None:
                _ld_kw["gamma"] = args.gamma
            if args.gae_lambda is not None:
                _ld_kw["gae_lambda"] = args.gae_lambda
            model = algo_cls.load(args.init_from, env=venv,
                                  device=args.device,
                             n_steps=args.n_steps,
                             batch_size=args.batch_size,
                             n_epochs=args.n_epochs, learning_rate=args.lr,
                             ent_coef=args.ent_coef, **_ld_kw,
                             target_kl=(args.target_kl or None),
                             tensorboard_log=tb_dir)
            # A plain --init-from warm start keeps the checkpoint's own
            # architecture. --net-arch used to be a hard error here, but
            # respec-cloned continuations legitimately carry the SAME
            # value the checkpoint was built with (08-11: every retry of
            # cw-uni-flag-a1-h1 crashed at 0 steps on this). Accept a
            # matching --net-arch; refuse only a genuine mismatch.
            if net_arch != [128, 128]:
                ck = getattr(model.policy, "net_arch", None)
                if isinstance(ck, dict):  # SB3 dict(pi=..., vf=...) form
                    ck = ck.get("pi", ck.get("vf"))
                if ck is not None and list(ck) != net_arch:
                    raise SystemExit(
                        f"--net-arch {net_arch} conflicts with the "
                        f"checkpoint's architecture {list(ck)} on a plain "
                        "--init-from warm start (the checkpoint carries "
                        "its own architecture); drop the flag or start "
                        "from scratch")
                print(f"[mjx-train] --net-arch {net_arch} matches the "
                      "warm-start checkpoint; proceeding")
            print(f"[mjx-train] warm start from {args.init_from}")
            if args.asym_critic:
                from .asym_policy import AsymActorCriticPolicy
                if not isinstance(model.policy, AsymActorCriticPolicy):
                    # Same transplant train_ppo_sim does: state_dict
                    # keys match exactly (the actor mask is a
                    # non-persistent buffer), so an MLP champion loads
                    # into the asym policy with zero drift; optimizer
                    # state is fresh (architecture changed).
                    old = model
                    model = algo_cls(
                        AsymActorCriticPolicy, venv,
                        n_steps=args.n_steps, batch_size=args.batch_size,
                        n_epochs=args.n_epochs, learning_rate=args.lr,
                        gamma=(0.99 if args.gamma is None
                               else args.gamma),
                        gae_lambda=(0.95 if args.gae_lambda is None
                                    else args.gae_lambda),
                        ent_coef=args.ent_coef, clip_range=0.2,
                        target_kl=(args.target_kl if args.target_kl > 0
                                   else None),
                        policy_kwargs=dict(
                            net_arch=net_arch,
                            log_std_init=args.log_std_init,
                            privileged_idx=extra_pk["privileged_idx"]),
                        seed=args.seed, verbose=1, device=args.device,
                        tensorboard_log=tb_dir)
                    res = model.policy.load_state_dict(
                        old.policy.state_dict(), strict=True)
                    model.num_timesteps = old.num_timesteps
                    del old
                    print("[mjx-train] asym-critic transplant: champion "
                          f"weights loaded ({res}); actor masks obs "
                          f"dims {model.policy.privileged_idx}")
    else:
        model = algo_cls(
            policy_cls, venv,
            n_steps=args.n_steps, batch_size=args.batch_size,
            n_epochs=args.n_epochs, learning_rate=args.lr,
            gamma=(0.99 if args.gamma is None else args.gamma),
            gae_lambda=(0.95 if args.gae_lambda is None
                        else args.gae_lambda),
            ent_coef=args.ent_coef,
            clip_range=0.2,
            target_kl=(args.target_kl if args.target_kl > 0 else None),
            policy_kwargs=dict(net_arch=net_arch,
                               log_std_init=args.log_std_init,
                               **extra_pk),
            seed=args.seed, verbose=1, device=args.device,
            tensorboard_log=tb_dir)

    if args.warm_log_std_override is not None and args.init_from is not None:
        import math as _math
        import torch as _th
        with _th.no_grad():
            pol = model.policy
            _std = _math.exp(float(args.warm_log_std_override))
            if hasattr(pol, "_log_stds"):
                for _ls in pol._log_stds():
                    _ls.data.fill_(float(args.warm_log_std_override))
                print("[mjx-train] warm-log-std-override: reset ALL "
                      f"per-expert log_std to "
                      f"{args.warm_log_std_override} (std={_std:.3f})")
            elif hasattr(pol, "log_std"):
                pol.log_std.data.fill_(float(args.warm_log_std_override))
                print("[mjx-train] warm-log-std-override: reset "
                      f"log_std to {args.warm_log_std_override} "
                      f"(std={_std:.3f})")
            else:
                print("[mjx-train] warm-log-std-override: WARNING no "
                      "log_std attribute found on policy, skipped")

    if args.gru_experts_freeze:
        from .gru_policy import ModeExpertsGruActorCriticPolicy as _MEP
        if not isinstance(model.policy, _MEP):
            raise SystemExit(
                "--gru-experts-freeze needs a ModeExperts policy "
                "(pass --gru-experts, or warm-start from an experts "
                "checkpoint)")
        if model.policy.experts_adapter is None:
            raise SystemExit(
                "--gru-experts-freeze with no transition adapter on "
                "the policy would train nothing actor-side; distill "
                "the init with an adapter or drop the freeze")
        model.policy.set_experts_frozen(True)
        print("[mjx-train] mode-experts ACTOR bodies FROZEN (cores, "
              "actor latents, heads, per-expert log_std); training "
              "only the transition adapter + critics")

    if mirror_coef > 0.0:
        attach_mirror(model, coef=mirror_coef, task=args.task,
                      cfg=env_kw.get("cfg"),
                      obs_dim=int(venv.observation_space.shape[0]))
    if bc_coef > 0.0:
        from .bc_anchor import attach_bc_anchor
        attach_bc_anchor(model, coef=bc_coef, cfg=env_kw.get("cfg"),
                         task=args.task)
    # Update-path protection (fb_20260817T005114; default off).
    if args.actor_lr > 0.0:
        from .update_health import (CRITIC_MARKERS,
                                    attach_actor_critic_lr,
                                    attach_kl_rollback)
        # Condition D adds two critic-only modules (value_gate,
        # latent_adapter) that share no substring with the stock SB3
        # markers — extend them so those modules ride the CRITIC lr,
        # not the (smaller, decaying) actor one (same extension as
        # train_ppo_transfer; test_dynrep_predictive_critic.py pins
        # the split).
        _markers = (CRITIC_MARKERS + ("value_gate", "latent_adapter")
                    if args.critic_encoder is not None else None)
        attach_actor_critic_lr(
            model, args.actor_lr,
            (args.actor_lr_final if args.actor_lr_final > 0.0
             else None),
            (args.critic_lr if args.critic_lr > 0.0 else args.lr),
            critic_markers=_markers)
        st = model._ac_state
        print(f"[update-health] actor/critic param groups: "
              f"{st['n_actor']} actor tensors @ {st['actor_lr']:.2e}"
              f"{' -> %.2e' % st['actor_lr_final'] if st['actor_lr_final'] != st['actor_lr'] else ''}, "
              f"{st['n_critic']} critic tensors @ "
              f"{st['critic_lr']:.2e} (constant)")
        if args.actor_freeze_steps > 0:
            from .update_health import set_actor_freeze
            set_actor_freeze(model, args.actor_freeze_steps)
            print("[update-health] actor FROZEN (group lr=0) until "
                  f"{args.actor_freeze_steps:,} env-steps; critic "
                  "learns throughout (train/actor_frozen logged; "
                  "operator order fb_20260818T102844_116d4c)")
            if args.actor_freeze_ev_threshold > 0.0:
                from .update_health import attach_ev_readiness_release
                attach_ev_readiness_release(
                    model, args.actor_freeze_ev_threshold,
                    args.actor_freeze_ev_windows,
                    args.actor_freeze_max_steps)
                print("[update-health] freeze release ALSO gated on "
                      "critic readiness: train/explained_variance >= "
                      f"{args.actor_freeze_ev_threshold} for "
                      f"{args.actor_freeze_ev_windows} consecutive "
                      "updates, fail-closed abort at "
                      f"{args.actor_freeze_max_steps:,} steps "
                      "(fb_20260818T112826_9ed832 item 2)")
        if args.kl_rollback > 0.0:
            attach_kl_rollback(model, args.kl_rollback,
                               lr_factor=args.kl_rollback_lr_factor)
            print(f"[update-health] transactional updates armed: "
                  f"rollback + actor-LR x{args.kl_rollback_lr_factor} "
                  f"on realized KL > {args.kl_rollback}")
    elif args.kl_rollback > 0.0:
        raise SystemExit("--kl-rollback requires --actor-lr (the "
                         "rollback reduces the actor-LR scale)")

    live_store = None
    pred_live_cb = None
    pred_online_path = None
    if args.critic_encoder is not None:
        # The policy-facing transformer is immutable during each complete
        # rollout+GAE+PPO iteration. In live mode a separate online copy
        # learns on CUDA, then must pass every boundary gate before an atomic
        # snapshot replacement can affect either actor or critic.
        import torch
        from rl_move.dynamics import data as dd
        from rl_move.dynamics.joint_aux import priv_group_metrics
        from rl_move.dynamics.live_replay import prediction_accuracy_metrics
        from rl_move.dynamics.model import dynamics_loss
        from rl_move.dynamics.predictive_critic import PredictorConfig
        from rl_move.dynamics.train_ppo_transfer import (
            anchor_batch_to_torch)
        anchor_path = Path(args.anchor_data)
        if not anchor_path.is_absolute():
            anchor_path = _PROTO / anchor_path
        eps = dd.load_dataset(anchor_path)
        enc_ckpt = torch.load(args.critic_encoder, map_location="cpu",
                              weights_only=False)
        stats = dd.Stats.from_dict(enc_ckpt["stats"])
        snap = model.policy.critic_predictor
        hist = int(extra_pk["history"])
        pred_cfg = PredictorConfig(
            mode="live" if args.predictive_live else "frozen",
            batch_size=args.pred_batch_size,
            rehearsal_frac=args.pred_rehearsal_frac,
            steps_per_iter=args.pred_steps_per_iter,
            lr=args.pred_lr,
            drift_guard=args.pred_snapshot_drift_guard,
            snapshot_boundary_steps=args.pred_snapshot_boundary_steps,
            gate_heldout_band=args.pred_gate_heldout_band,
            gate_live_improve=args.pred_gate_live_improve,
            gate_rise_band=args.pred_gate_rise_band,
            gate_value_jump_frac=args.pred_gate_value_jump,
            gate_action_kl=args.pred_gate_action_kl)

        if args.predictive_live:
            if model.device.type != "cuda":
                raise SystemExit("--predictive-live resolved Torch device "
                                 f"to {model.device}, not CUDA")
            sampler_cls = dd.GpuWindowSampler
            sampler_kw = {"device": model.device}
        else:
            sampler_cls = dd.WindowSampler
            sampler_kw = {}
        rehearsal = sampler_cls(
            eps, stats, hist, snap.horizons, split="train",
            seed=args.seed, **sampler_kw)
        heldout = sampler_cls(
            eps, stats, hist, snap.horizons, split="val",
            seed=args.seed + 1, **sampler_kw)

        def _pred_sink(payload: dict, step: int) -> None:
            if run is not None:
                import wandb
                wandb.log({"global_step": step, **payload})

        def _model_val(m, sampler, n_windows=2048) -> tuple[float | None,
                                                             dict]:
            was_training = m.training
            m.eval()
            total, n_b, sums = 0.0, 0, {}
            with torch.no_grad():
                for b in sampler.val_batches(n_windows,
                                             args.pred_batch_size):
                    bt = anchor_batch_to_torch(b, device=model.device)
                    out_p = m(bt["hist"], bt["fut_actions"])
                    loss, metrics = dynamics_loss(
                        out_p, bt, pred_cfg.lambdas, m)
                    metrics.update(priv_group_metrics(out_p, bt, prefix=""))
                    metrics.update(prediction_accuracy_metrics(
                        out_p, bt, stats, prefix="physical/"))
                    total += float(loss)
                    n_b += 1
                    for key, value in metrics.items():
                        sums[key] = sums.get(key, 0.0) + float(value)
            if was_training:
                m.train()
            return ((total / n_b if n_b else None),
                    {k: v / max(n_b, 1) for k, v in sums.items()})

        live_batch_fn = None
        gate_fns = None
        pretrained_ref = {"total": float("nan")}
        if args.predictive_live:
            from rl_move.dynamics.live_replay import (
                LiveWindowStore, stratified_live_batch)
            live_store = LiveWindowStore(
                stats, hist, snap.horizons, model.device,
                max_walk_windows=30_000, max_rise_windows=1,
                max_val_windows=5_000, windows_per_episode=64,
                val_every=8, seed=args.seed)
            rise_eps = [ep for ep in eps if ep.mode in ("rise", "raise")]
            if not rise_eps:
                raise SystemExit("--predictive-live anchor corpus has no "
                                 "rise episodes for retention gating")
            rise_heldout = dd.GpuWindowSampler(
                rise_eps, stats, hist, snap.horizons, split="val",
                device=model.device, seed=args.seed + 2)

            def live_batch_fn(n: int):
                return stratified_live_batch(
                    live_store, rehearsal, anchor_batch_to_torch,
                    model.device, n,
                    rehearsal_frac=args.pred_rehearsal_frac,
                    walk_frac=args.pred_live_walk_frac,
                    min_live_windows=args.pred_live_min_windows)

            def corpus_val(m) -> float:
                value, _ = _model_val(m, heldout)
                return float(value)

            def live_val(m, mode: str) -> float | None:
                if mode == "rise":
                    value, _ = _model_val(m, rise_heldout)
                    return value
                if live_store.num_windows("walk", "val") < 128:
                    return None
                was_training = m.training
                m.eval()
                total, n_b = 0.0, 0
                with torch.no_grad():
                    for bt in live_store.val_batches(
                            "walk", args.pred_batch_size,
                            max_windows=2048):
                        out_p = m(bt["hist"], bt["fut_actions"])
                        loss, _ = dynamics_loss(
                            out_p, bt, pred_cfg.lambdas, m)
                        total += float(loss)
                        n_b += 1
                if was_training:
                    m.train()
                return total / n_b if n_b else None

            gate_fns = {
                "corpus_val": corpus_val,
                "live_val": live_val,
                "pretrained_ref": lambda: pretrained_ref["total"],
            }

        model.configure_predictor(
            pred_cfg, rehearsal, anchor_batch_to_torch,
            metrics_sink=_pred_sink, live_batch_fn=live_batch_fn,
            gate_fns=gate_fns)
        snap.eval()
        _ref, _ref_metrics = _model_val(snap, heldout, n_windows=1024)
        pretrained_ref["total"] = float(_ref)
        _sv = int(model.policy.snapshot_version.item())
        mode_note = "live actor+critic" if args.predictive_live else "frozen"
        print(f"[dynrep-{mode_note}] heldout pred loss at start: "
              f"{_ref:.3f} (pretrained, untouched); "
              f"snapshot_version={_sv}")
        if run is not None:
            import wandb
            run.config.update({
                "critic_encoder": str(args.critic_encoder),
                "critic_encoder_md5": args.critic_encoder_md5,
                "pred_mode": pred_cfg.mode,
                "pred_actor_conditioning": args.predictive_live,
                "pred_device": str(model.device),
                "pred_batch_size": args.pred_batch_size,
                "pred_steps_per_iter": args.pred_steps_per_iter,
                "pred_rehearsal_frac": args.pred_rehearsal_frac,
                "pred_capture_envs": (args.pred_capture_envs
                                      if args.predictive_live else 0),
                "pred_snapshot_boundary_steps":
                    args.pred_snapshot_boundary_steps,
                "pred_gate_action_kl": args.pred_gate_action_kl,
            }, allow_val_change=True)
            wandb.log({
                "global_step": 0,
                "anchor/pretrained_loss": _ref,
                "pred/snapshot_version": _sv,
                **{f"pred/heldout/{k}": v
                   for k, v in _ref_metrics.items()},
            })

    out_name = args.out_name or (
        f"ppo_mjx_{args.task}" + (f"_{args.run_name}" if args.run_name
                                  else ""))
    out_path = POLICY_DIR / f"{out_name}.zip"
    if args.predictive_live:
        pred_online_path = POLICY_DIR / f"dyn_online_{out_name}.pt"
    POLICY_DIR.mkdir(parents=True, exist_ok=True)

    class _Track(BaseCallback):
        """fps, ep stats, env/reward-part means (campaign parity, sampled
        across envs to stay cheap at B=4096), termination-reason counts,
        periodic checkpoints. Mirrors train_ppo_sim's
        _make_reward_parts_callback so MJX runs chart like C runs."""

        PART_KEYS = (
            "reward_task", "reward_roll", "reward_pitch", "reward_height",
            "reward_gyro", "reward_action", "reward_action_delta",
            "reward_current", "reward_unload", "reward_rise_progress",
            "reward_rise_milestone", "reward_rise_finish",
            "reward_curl_progress", "reward_curl_milestone", "reward_walk",
            "reward_walk_prog", "reward_swing", "reward_current_hot",
            "reward_stance", "reward_clearance", "reward_flag_leg",
            "reward_current_max", "reward_termination",
            "reward_phase_contact", "reward_support_margin",
            "reward_load_even", "reward_step_event", "reward_drag",
            "reward_park_duty", "reward_end_posture", "reward_effort",
            "reward_walk_yaw", "reward_quad_clear", "reward_quad_plant",
            # Gate factors (08-10 postgate1 dig-in: rise_posture_factor
            # was computed in-env but never logged, blinding triage to
            # the gate's live value).
            "rise_posture_factor", "rise_income_factor")
        AUX_ABS = ("roll_deg", "pitch_deg")       # logged as abs_<k>
        AUX = ("track_err_deg", "height_err_mm", "mean_current_a",
               "walk_vel_err", "walk_speed",
               "walk_direction_err_deg",
               "phase_agreement",
               "walk_anchor_frac",
               "walk_step_denied", "walk_step_bank_m",
               "walk_loadslip_ratio", "walk_loadslip_factor",
               "walk_yaw_err",
               "quad_clear_mm", "quad_fronts_off",
               "quad_planted_frac")  # own names
        # Any OTHER numeric scalar the env drops into info is logged
        # as a plain mean under env/<k> (operator 08-10: the whitelist
        # above kept drifting behind the envs — reward_rise_ref,
        # walk_wz, rise_plant_factor etc. were computed but invisible
        # in W&B, and W&B is the primary triage surface). The lists
        # stay for their special semantics (AUX* means abs()).
        SKIP = ("TimeLimit.truncated", "terminal_observation",
                "termination_reason", "goal_mode", "walk_bucket",
                "episode", "bc_target", "amp_obs_style")
        SAMPLE = 256      # envs sampled per step for the means

        def __init__(self):
            super().__init__()
            self._t0 = time.monotonic()
            self._last_save = 0
            self._sum: dict[str, float] = {}
            self._cnt: dict[str, int] = {}
            self._terms: dict[str, int] = {}
            # Mode-experts active-tick accounting (directive
            # fb_20260815T013349_488ffd: report ACTIVE ticks per
            # expert, not just total env steps). Cumulative over the
            # whole run; indices follow gru_policy.EXPERTS_ORDER.
            self._exp_ticks = np.zeros(4, dtype=np.float64)
            # Joystick command telemetry (08-15, operator directive
            # fb_20260815T114414, SIMPLIFIED by fb_20260815T115650):
            # cumulative ACTIVE-TICK accounting of the env's
            # goal.walk_cmd_metrics info keys — raw signed v_along and
            # ratio-of-sums (NOT mean of per-tick ratios). No
            # per-heading bins in training: uniform [-pi,pi] heading
            # sampling + the signed average already zeroes out
            # command-ignorant motion; fixed-direction panels are
            # held-out EVAL tools. Zero-cost when the env never emits
            # the keys.
            self._cmd_cum = {"along": 0.0, "cmd": 0.0, "cross": 0.0,
                             "wrong": 0.0, "n": 0.0}
            self._cmd_stride = 1
            # Overall optimization-progress metric (operator feedback
            # fb_20260815T131225_c8442f, 08-15): "is PPO still getting
            # more total reward per real transition" — computed
            # directly from the raw per-step scalar rewards SB3 hands
            # the callback (self.locals["rewards"], the actual PPO
            # training signal, captured before the truncation-bootstrap
            # adjustment further down collect_rollouts), NOT from
            # ep_rew_mean/ep_len_mean (those distort under changing
            # episode length / partial episodes). Per-rollout sum+count
            # reset every _on_rollout_end; cumulative + EMA never reset.
            self._reward_sum = 0.0
            self._reward_n = 0
            self._reward_sum_cum = 0.0
            self._reward_n_cum = 0
            self._reward_ema: float | None = None
            # Exact completed-episode recovery scores. These inspect every
            # env terminal, not the 256-env telemetry sample above.
            self._recover_window: dict[int, list[int]] = {}
            self._recover_cumulative: dict[int, list[int]] = {}
            self._recover_error_window: dict[int, list[float]] = {}
            self._recover_error_cumulative: dict[int, list[float]] = {}

        def _acc(self, k: str, v: float) -> None:
            self._sum[k] = self._sum.get(k, 0.0) + v
            self._cnt[k] = self._cnt.get(k, 0) + 1

        def _on_step(self) -> bool:
            rewards = self.locals.get("rewards")
            if rewards is not None:
                arr = np.asarray(rewards)
                self._reward_sum += float(arr.sum())
                self._reward_n += int(arr.size)
            if args.gru_experts:
                no = self.locals.get("new_obs")
                if no is not None and getattr(no, "ndim", 0) == 2 \
                        and no.shape[1] >= 6:
                    tail = no[:, -6:]
                    # EXPERTS_ORDER = (rise, hold, lower, loco);
                    # obs tail = (hold, rise, lower, walk, turn, quad)
                    self._exp_ticks[0] += float(tail[:, 1].sum())
                    self._exp_ticks[1] += float(tail[:, 0].sum())
                    self._exp_ticks[2] += float(tail[:, 2].sum())
                    self._exp_ticks[3] += float(tail[:, 3:].sum())
            infos = self.locals.get("infos", ())
            stride = max(1, len(infos) // self.SAMPLE)
            for info in infos[::stride]:
                for k in self.PART_KEYS:
                    if k in info:
                        self._acc(k, float(info[k]))
                for k in self.AUX_ABS:
                    if k in info:
                        self._acc(f"abs_{k}", abs(float(info[k])))
                for k in self.AUX:
                    if k in info:
                        self._acc(k, abs(float(info[k])))
                for k, v in info.items():
                    if (k in self.PART_KEYS or k in self.AUX
                            or k in self.AUX_ABS or k in self.SKIP):
                        continue
                    if isinstance(v, bool) or not isinstance(
                            v, (int, float, np.integer, np.floating)):
                        continue
                    self._acc(k, float(v))
                if "track_err_deg" in info:
                    self._acc("pct_within_1deg",
                              1.0 if float(info["track_err_deg"]) <= 1.0
                              else 0.0)
                if "v_along_cmd_m_s" in info:
                    # Cumulative active-tick command telemetry (see
                    # __init__); sums, so ratios come out as
                    # sum(v_along)/sum(cmd_speed), never mean-of-ratios.
                    self._cmd_stride = stride
                    al = float(info["v_along_cmd_m_s"])
                    self._cmd_cum["along"] += al
                    self._cmd_cum["cmd"] += float(
                        info.get("cmd_speed_m_s", 0.0))
                    self._cmd_cum["cross"] += float(
                        info.get("v_cross_abs_m_s", 0.0))
                    self._cmd_cum["wrong"] += float(
                        info.get("wrong_way", 0.0))
                    self._cmd_cum["n"] += 1.0
            dones = self.locals.get("dones")
            if dones is not None and np.any(dones):
                for i in np.flatnonzero(dones):
                    outcome = _recover_episode_outcome(infos[i])
                    if outcome is not None:
                        bucket, success = outcome
                        for bank in (self._recover_window,
                                     self._recover_cumulative):
                            counts = bank.setdefault(bucket, [0, 0])
                            counts[0] += int(success)
                            counts[1] += 1
                    training_error = _recover_episode_training_error(
                        infos[i])
                    if training_error is not None:
                        bucket, error = training_error
                        for bank in (self._recover_error_window,
                                     self._recover_error_cumulative):
                            values = bank.setdefault(bucket, [0.0, 0.0])
                            values[0] += error
                            values[1] += 1.0
                    r = infos[i].get("termination_reason") or (
                        "truncated"
                        if infos[i].get("TimeLimit.truncated")
                        else "done")
                    self._terms[r] = self._terms.get(r, 0) + 1
            return True

        def _on_rollout_end(self) -> None:
            if self._recover_error_window:
                # Every host env receives the same aggregate, avoiding 512
                # tiny local EMAs whose sparse bucket histories diverge.
                venv.env_method(
                    "apply_recover_training_error_batch",
                    {bucket: (values[0], int(values[1]))
                     for bucket, values in self._recover_error_window.items()})
            fps = self.num_timesteps / max(time.monotonic() - self._t0,
                                           1e-9)
            payload = {"time/env_steps_per_s": fps,
                       "time/total_env_steps": self.num_timesteps,
                       "global_step": self.num_timesteps}
            payload.update({f"env/{k}": self._sum[k] / self._cnt[k]
                            for k in self._sum})
            payload.update({f"terminations/{k}": v
                            for k, v in self._terms.items()})
            for bucket, (successes, episodes) in sorted(
                    self._recover_window.items()):
                stem = f"TRAIN/recover_bucket_{bucket}"
                payload[f"{stem}_success_fraction"] = successes / episodes
                payload[f"{stem}_successes"] = successes
                payload[f"{stem}_episodes"] = episodes
            for bucket, (successes, episodes) in sorted(
                    self._recover_cumulative.items()):
                stem = f"TRAIN/recover_bucket_{bucket}"
                payload[f"{stem}_success_fraction_cumulative"] = (
                    successes / episodes)
                payload[f"{stem}_episodes_cumulative"] = episodes
            for bucket, (error_sum, episodes) in sorted(
                    self._recover_error_window.items()):
                stem = f"TRAIN/recover_bucket_{bucket}"
                payload[f"{stem}_training_error_mean"] = (
                    error_sum / max(episodes, 1.0))
                payload[f"{stem}_training_error_episodes"] = episodes
            for bucket, (error_sum, episodes) in sorted(
                    self._recover_error_cumulative.items()):
                stem = f"TRAIN/recover_bucket_{bucket}"
                payload[f"{stem}_training_error_mean_cumulative"] = (
                    error_sum / max(episodes, 1.0))
            if self._reward_n > 0:
                # optimization/* (fb_20260815T131225_c8442f): "is PPO
                # continuing to get more total reward per real
                # transition" — an OPTIMIZATION/objective score, not a
                # behavioral-success claim; read it beside the task
                # (joystick/v_along_m_s) and safety (terminations/*)
                # metrics, never alone. reward/tick rising + task
                # rising = useful learning; reward/tick rising + task
                # falling = exploiting/prioritizing a different reward
                # term; reward/tick flat = optimization stalled.
                rpt = self._reward_sum / self._reward_n
                payload["optimization/reward_per_tick"] = rpt
                self._reward_sum_cum += self._reward_sum
                self._reward_n_cum += self._reward_n
                payload["optimization/reward_per_tick_cumulative"] = (
                    self._reward_sum_cum / self._reward_n_cum)
                self._reward_ema = (rpt if self._reward_ema is None else
                                    0.9 * self._reward_ema + 0.1 * rpt)
                payload["optimization/reward_per_tick_ema"] = (
                    self._reward_ema)
            self._reward_sum, self._reward_n = 0.0, 0
            if self._cmd_cum["n"] > 0:
                # Operator-named joystick command-following metrics
                # (fb_20260815T114414, simplified fb_20260815T115650).
                # HEADLINE (joystick/*): raw signed m/s along the
                # requested direction over active ticks — per-rollout
                # mean, active-tick-weighted cumulative mean, and the
                # active-tick audit count. Everything else (cross-track,
                # wrong-way, ratio-of-sums) is secondary under train/.
                # NO per-heading series here — fixed-direction checks
                # live in held-out EVAL only.
                c = self._cmd_cum
                n_r = self._cnt.get("v_along_cmd_m_s", 0)
                if n_r:
                    s_al = self._sum["v_along_cmd_m_s"]
                    s_cmd = self._sum.get("cmd_speed_m_s", 0.0)
                    payload["joystick/v_along_m_s"] = s_al / n_r
                    payload["train/cmd_speed_active_m_s"] = (
                        s_cmd / max(self._cnt.get("cmd_speed_m_s", 1), 1))
                    if s_cmd > 0.0:
                        payload["train/v_along_ratio_active"] = (
                            s_al / s_cmd)
                payload["joystick/v_along_m_s_cumulative"] = (
                    c["along"] / c["n"])
                if c["cmd"] > 0.0:
                    payload["train/v_along_ratio_active_cumulative"] = (
                        c["along"] / c["cmd"])
                payload["train/v_cross_abs_m_s"] = c["cross"] / c["n"]
                payload["train/wrong_way_frac"] = c["wrong"] / c["n"]
                # Estimate: sampled count x sample stride (the info
                # sweep reads every stride-th env).
                payload["joystick/active_ticks"] = (
                    c["n"] * self._cmd_stride)
            if args.gru_experts:
                from .gru_policy import EXPERTS_ORDER
                tot = max(float(self._exp_ticks.sum()), 1.0)
                for i, name in enumerate(EXPERTS_ORDER):
                    payload[f"experts/active_ticks_{name}"] = float(
                        self._exp_ticks[i])
                    payload[f"experts/tick_frac_{name}"] = float(
                        self._exp_ticks[i]) / tot
                pol = self.model.policy
                if hasattr(pol, "_log_stds"):
                    for name, ls in zip(EXPERTS_ORDER,
                                        pol._log_stds()):
                        payload[f"experts/std_{name}"] = float(
                            ls.detach().exp().mean())
            buf = self.model.ep_info_buffer
            if buf:
                payload["rollout/ep_rew_mean"] = float(
                    np.mean([e["r"] for e in buf]))
                payload["rollout/ep_len_mean"] = float(
                    np.mean([e["l"] for e in buf]))
            if run is not None:
                import wandb
                wandb.log(payload)
            self._sum, self._cnt, self._terms = {}, {}, {}
            self._recover_window = {}
            self._recover_error_window = {}
            if (args.save_every and self.num_timesteps - self._last_save
                    >= args.save_every):
                self._last_save = self.num_timesteps
                self.model.save(out_path)
                print(f"[mjx-train] checkpoint @ {self.num_timesteps:,} "
                      f"-> {out_path} ({fps:,.0f} env-steps/s)")


    if args.recover_init_curriculum is not None:
        _restored = _restore_recover_curriculum_from_sidecar(
            venv, args.recover_init_curriculum)
        print("[recover-init] curriculum restored from "
              f"{args.recover_init_curriculum}: "
              f"active_n={int(_restored['active_n'])} "
              f"(frontier B{int(_restored['active_n']) - 1}), "
              "cert stats carried from the sidecar")

    population = None
    if args.recover_population_id:
        initial_bucket = int(venv.get_attr(
            "_rec_active_n", indices=0)[0]) - 1
        population = _RecoverPopulation(
            args, run, initial_bucket=initial_bucket)
        print("[recover-pop] synchronized cohort armed: "
              f"{len(population.peer_names)} members, this member "
              f"{population.member}, initial B{initial_bucket}")

    callbacks: list = [_Track()]
    if _prof_ramp_steps > 0:
        class _ProfileRampCb(BaseCallback):
            """Advance the servo-profile ramp once per rollout (see the
            arming block after venv construction). Broadcasts stay on
            after frac hits 1.0 for one extra round (idempotent), then
            stop; W&B gets the live profile under profile_ramp/*."""

            def __init__(self):
                super().__init__()
                self._finished = False

            def _on_step(self) -> bool:
                return True

            def _on_rollout_end(self) -> None:
                if self._finished:
                    return
                vals = _profile_ramp_apply(venv, self.num_timesteps)
                if vals["frac"] >= 1.0:
                    self._finished = True
                    print("[profile-ramp] ramp complete @ "
                          f"{self.num_timesteps:,} steps — training at "
                          "the full target dose from here on")
                if run is not None:
                    import wandb
                    wandb.log({
                        "global_step": self.num_timesteps,
                        "profile_ramp/frac": vals["frac"],
                        "profile_ramp/write_speed_counts_s":
                            vals["write_speed_counts_s"],
                        "profile_ramp/write_acc": vals["write_acc"],
                        "profile_ramp/max_delta_q_deg":
                            vals["max_delta_q_deg"]})

        callbacks.append(_ProfileRampCb())
    if _da_ramp_steps > 0:
        class _DragAllowRampCb(BaseCallback):
            """Advance the drag-stance allowance ramp once per rollout
            (see the arming block after venv construction). Broadcasts
            stay on after frac hits 1.0 for one extra round (idempotent),
            then stop; W&B gets the live allowance under
            drag_allow_ramp/*."""

            def __init__(self):
                super().__init__()
                self._finished = False

            def _on_step(self) -> bool:
                return True

            def _on_rollout_end(self) -> None:
                if self._finished:
                    return
                vals = _drag_allow_ramp_apply(venv, self.num_timesteps)
                if vals["frac"] >= 1.0:
                    self._finished = True
                    print("[drag-allow-ramp] ramp complete @ "
                          f"{self.num_timesteps:,} steps — training at "
                          "the target allowance from here on")
                if run is not None:
                    import wandb
                    wandb.log({
                        "global_step": self.num_timesteps,
                        "drag_allow_ramp/frac": vals["frac"],
                        "drag_allow_ramp/allow_mm": vals["allow_mm"]})

        callbacks.append(_DragAllowRampCb())
    if _tp_ramp_steps > 0:
        class _TermPenaltyRampCb(BaseCallback):
            """Advance the termination-penalty ramp once per rollout
            (see the arming block after venv construction). W&B gets
            the live penalty under term_penalty_ramp/*."""

            def __init__(self):
                super().__init__()
                self._finished = False

            def _on_step(self) -> bool:
                return True

            def _on_rollout_end(self) -> None:
                if self._finished:
                    return
                vals = _term_penalty_ramp_apply(venv, self.num_timesteps)
                if vals["frac"] >= 1.0:
                    self._finished = True
                    print("[term-penalty-ramp] ramp complete @ "
                          f"{self.num_timesteps:,} steps — training at "
                          "the full deterrent from here on")
                if run is not None:
                    import wandb
                    wandb.log({
                        "global_step": self.num_timesteps,
                        "term_penalty_ramp/frac": vals["frac"],
                        "term_penalty_ramp/term_penalty":
                            vals["term_penalty"]})

        callbacks.append(_TermPenaltyRampCb())
    if args.ent_coef_final is not None:
        class _EntCoefAnnealCb(BaseCallback):
            """Linearly anneal model.ent_coef from args.ent_coef to
            args.ent_coef_final over ent_coef_anneal_frac * args.steps,
            then hold. SB3's PPO.train() reads self.ent_coef fresh each
            gradient step (a plain float attribute, not a resolved
            schedule) so mutating it between rollouts is safe and takes
            effect on the next update. Default OFF (--ent-coef-final
            unset) is bit-exact: this callback is never constructed."""

            def __init__(self):
                super().__init__()
                self._start = float(args.ent_coef)
                self._final = float(args.ent_coef_final)
                denom = max(1, int(args.ent_coef_anneal_frac
                                    * args.steps))
                self._denom = denom
                self._finished = False

            def _on_step(self) -> bool:
                return True

            def _on_rollout_end(self) -> None:
                frac = min(1.0, self.num_timesteps / self._denom)
                val = self._start + frac * (self._final - self._start)
                self.model.ent_coef = float(val)
                if frac >= 1.0 and not self._finished:
                    self._finished = True
                    print("[ent-coef-anneal] complete @ "
                          f"{self.num_timesteps:,} steps — holding "
                          f"ent_coef={val:.5f}")
                if run is not None:
                    import wandb
                    wandb.log({"global_step": self.num_timesteps,
                               "ent_coef_anneal/value": val,
                               "ent_coef_anneal/frac": frac})

        callbacks.append(_EntCoefAnnealCb())
    if args.log_std_final is not None:
        class _LogStdAnnealCb(BaseCallback):
            """Linearly anneal the policy's log_std parameter(s) to
            args.log_std_final over log_std_anneal_frac * args.steps,
            then hold. Unlike the entropy-bonus route this SETS the
            parameter directly (the proven --warm-log-std-override
            mechanism on a schedule), so PPO's own gradient cannot
            fight it between rollouts. Default OFF (--log-std-final
            unset): callback never constructed, bit-exact legacy.

            TIMING FIX (08-23, stdanneal45/swinganneal45 forensics):
            the set MUST happen at rollout START, not rollout end.
            Setting it between collection and train() shifts every
            stored log_prob, inflating first-minibatch approx_kl
            (~0.13 at a 2.5-nat/2M anneal rate) past SB3's
            target_kl=0.02 early-stop, which fires BEFORE
            optimizer.step() — both 08-23 amp anneal arms completed
            2M steps with ZERO weight updates while reward 'rose'
            purely from shrinking action noise. At rollout start the
            collected actions, buffer log_probs and train() all share
            one log_std, so ratios start at 1 as normal."""

            def __init__(self):
                super().__init__()
                if args.warm_log_std_override is not None:
                    self._start = float(args.warm_log_std_override)
                else:
                    import torch as _th
                    pol = model.policy
                    with _th.no_grad():
                        if hasattr(pol, "_log_stds"):
                            vals = [float(_ls.data.mean())
                                    for _ls in pol._log_stds()]
                            self._start = sum(vals) / len(vals)
                        elif hasattr(pol, "log_std"):
                            self._start = float(pol.log_std.data.mean())
                        else:
                            self._start = float(args.log_std_final)
                self._final = float(args.log_std_final)
                self._denom = max(1, int(args.log_std_anneal_frac
                                         * args.steps))
                self._finished = False

            def _on_step(self) -> bool:
                return True

            def _on_rollout_start(self) -> None:
                import math as _math
                import torch as _th
                frac = min(1.0, self.num_timesteps / self._denom)
                val = self._start + frac * (self._final - self._start)
                pol = self.model.policy
                with _th.no_grad():
                    if hasattr(pol, "_log_stds"):
                        for _ls in pol._log_stds():
                            _ls.data.fill_(float(val))
                    elif hasattr(pol, "log_std"):
                        pol.log_std.data.fill_(float(val))
                if frac >= 1.0 and not self._finished:
                    self._finished = True
                    print("[log-std-anneal] complete @ "
                          f"{self.num_timesteps:,} steps — holding "
                          f"log_std={val:.3f} "
                          f"(std={_math.exp(val):.3f})")
                if run is not None:
                    import wandb
                    wandb.log({"global_step": self.num_timesteps,
                               "log_std_anneal/value": float(val),
                               "log_std_anneal/std": _math.exp(val),
                               "log_std_anneal/frac": frac})

        callbacks.append(_LogStdAnnealCb())
    if amp_wrap is not None:
        class _AMPDiscCb(BaseCallback):
            """Online AMP discriminator update, once per PPO rollout
            (standard AMP co-training cadence): real minibatches from
            the motion library, fake minibatches from the wrapper's
            policy-transition replay. Never constructed when
            --amp-style-weight is 0 (bit-exact off path)."""

            def _on_step(self) -> bool:
                return True

            def _on_rollout_end(self) -> None:
                stats = amp_wrap.train_discriminator(
                    args.amp_disc_steps, args.amp_disc_batch)
                roll = amp_wrap.pop_rollout_stats()
                if run is not None:
                    import wandb
                    payload = {"global_step": self.num_timesteps,
                               "amp/style_reward_mean":
                                   roll["style_reward_mean"],
                               "amp/pairs_per_rollout": roll["pairs"]}
                    if stats is not None:
                        payload.update({f"amp/{k}": v
                                        for k, v in stats.items()})
                    wandb.log(payload)

        callbacks.append(_AMPDiscCb())
        print(f"[amp-style] discriminator co-training armed: "
              f"{args.amp_disc_steps} updates x {args.amp_disc_batch} "
              "batch per rollout")
    if args.predictive_live:
        class _LivePredictorCapture(BaseCallback):
            """Harvest a bounded MJX subset into CUDA live replay."""

            def __init__(self):
                super().__init__()
                self._episodes: dict[int, dict] = {}
                self._next_metrics = int(args.pred_metrics_every)

            def _start_episode(self, i: int, row: dict) -> None:
                self._episodes[i] = {
                    "frames": [np.asarray(row["frame"], dtype=np.float32)],
                    "priv": [np.asarray(row["priv"], dtype=np.float32)],
                    "actions": [],
                    "mode": str(row["mode"]),
                    "start_at": str(row["start_at"]),
                    "q_nom": np.asarray(row["q_nom"], dtype=np.float32),
                    "dr": float(row["dr"]),
                }

            def _on_training_start(self) -> None:
                rows = venv.env_method("dynrep_episode_initial",
                                       indices=capture_indices)
                for i, row in zip(capture_indices, rows):
                    self._start_episode(i, row)

            def _on_step(self) -> bool:
                infos = self.locals.get("infos", ())
                dones = np.asarray(self.locals.get(
                    "dones", np.zeros(len(infos), dtype=bool)))
                finished = []
                for i in capture_indices:
                    info = infos[i]
                    if "dynrep_frame" not in info:
                        continue
                    ep = self._episodes[i]
                    ep["actions"].append(np.asarray(
                        info["dynrep_action"], dtype=np.float32))
                    ep["frames"].append(np.asarray(
                        info["dynrep_frame"], dtype=np.float32))
                    ep["priv"].append(np.asarray(
                        info["dynrep_priv"], dtype=np.float32))
                    if i < len(dones) and dones[i]:
                        completed = {
                            **ep,
                            "frames": np.stack(ep["frames"]),
                            "actions": np.stack(ep["actions"]),
                            "priv": np.stack(ep["priv"]),
                            "reason": ("trunc" if info.get(
                                "TimeLimit.truncated") else "term"),
                        }
                        live_store.add_episode(completed)
                        finished.append(i)
                if finished:
                    rows = venv.env_method("dynrep_episode_initial",
                                           indices=finished)
                    for i, row in zip(finished, rows):
                        self._start_episode(i, row)
                return True

            def _save_online(self) -> None:
                torch.save({
                    "model": self.model._online_dyn.state_dict(),
                    "config": self.model._online_dyn.config(),
                    "source_encoder": str(args.critic_encoder),
                    "global_step": int(self.num_timesteps),
                    "snapshot_version": int(
                        self.model.policy.snapshot_version.item()),
                }, pred_online_path)

            def _on_rollout_end(self) -> None:
                payload = live_store.composition_report()
                if (args.pred_metrics_every > 0
                        and self.num_timesteps >= self._next_metrics):
                    self._next_metrics = (
                        (self.num_timesteps // args.pred_metrics_every) + 1
                    ) * args.pred_metrics_every
                    payload.update(live_store.bin_report(
                        self.model._online_dyn, pred_cfg.lambdas))
                    held_loss, held = _model_val(
                        self.model._online_dyn, heldout)
                    payload["pred/heldout/total"] = float(held_loss)
                    payload.update({f"pred/heldout/{k}": v
                                    for k, v in held.items()})
                    self._save_online()
                if run is not None:
                    import wandb
                    wandb.log({"global_step": self.num_timesteps,
                               **payload})

            def _on_training_end(self) -> None:
                self._save_online()

        pred_live_cb = _LivePredictorCapture()
        callbacks.append(pred_live_cb)
        print("[dynrep-live] online transformer training armed: "
              f"{args.pred_steps_per_iter} CUDA updates/PPO iteration, "
              f"{args.pred_rehearsal_frac:.0%} retained rehearsal, "
              f"snapshot boundary every "
              f"{args.pred_snapshot_boundary_steps:,} env-steps")
    if bc_coef > 0.0:
        from .bc_anchor import make_bc_collect_callback
        callbacks.append(make_bc_collect_callback())

    cert_cb = None
    if (args.recover_cert_every > 0 and args.recover_cert_envs > 0
            and float(gm.get("recover", 0.0)) > 0.0):
        class _MjxRecoverCert(BaseCallback):
            """Deterministic curriculum authority on training physics."""

            def __init__(self):
                super().__init__()
                self._next = int(args.recover_cert_every)
                self._env = None
                self._retention_cursor = 0
                self._cert_round = 0
                self._last_cert_round: dict[int, int] = {}
                self._best_score = 0.0
                self._promotion_dir = POLICY_DIR / "recover_promotions"
                self._promotion_dir.mkdir(parents=True, exist_ok=True)
                self._latest_promotion: dict | None = None
                self._pending_rollback: dict | None = None
                self._retention_failed_since: dict[int, int] = {}
                self._rollback_count = 0
                self._promotion_checkpoint_count = 0

            def _save_promotion_checkpoint(self, admission: dict) -> None:
                bucket = int(admission["active_after"]) - 1
                path = self._promotion_dir / (
                    f"{out_name}_promote_B{bucket:02d}_"
                    f"step{self.num_timesteps}.zip")
                self.model.save(path)
                curriculum = venv.env_method(
                    "recover_curriculum_checkpoint_state", indices=0)[0]
                metadata_path = path.with_suffix(".curriculum.json")
                metadata_path.write_text(json.dumps({
                    "policy": path.name,
                    "promotion_bucket": bucket,
                    "global_step": int(self.num_timesteps),
                    "cert_round": int(self._cert_round),
                    "curriculum": curriculum,
                }, indent=2, sort_keys=True) + "\n")
                self._latest_promotion = {
                    "path": path,
                    "metadata_path": metadata_path,
                    "bucket": bucket,
                    "step": int(self.num_timesteps),
                    "cert_round": int(self._cert_round),
                    "curriculum": curriculum,
                }
                self._promotion_checkpoint_count += 1
                if run is not None and population is None:
                    try:
                        run.save(str(path), base_path=str(POLICY_DIR),
                                 policy="now")
                        run.save(str(metadata_path),
                                 base_path=str(POLICY_DIR), policy="now")
                        run.summary["recover_latest_promotion_checkpoint"] = (
                            str(path.relative_to(POLICY_DIR)))
                    except Exception as exc:
                        print("[recover-guard] W&B checkpoint upload failed: "
                              f"{exc}")
                print(f"[recover-guard] promotion checkpoint B{bucket} "
                      f"@ {self.num_timesteps:,}: {path}")
                if population is not None:
                    population.publish_candidate(self._latest_promotion)

            def _restore_checkpoint(self, checkpoint: dict) -> None:
                # This hook runs after the preceding PPO update and before
                # collecting the next rollout, so replacing the policy does
                # not invalidate old-policy log probabilities.
                self.model.set_parameters(
                    str(checkpoint["path"]), exact_match=True,
                    device=self.model.device)
                venv.env_method(
                    "restore_recover_curriculum_checkpoint_state",
                    checkpoint["curriculum"])
                active = [int(value) for value in venv.get_attr(
                    "_rec_active_n")]
                expected = int(checkpoint["bucket"]) + 1
                if len(active) != venv.num_envs or set(active) != {expected}:
                    counts = {
                        value: active.count(value)
                        for value in sorted(set(active))
                    }
                    raise RuntimeError(
                        "recovery checkpoint restore desynchronized the "
                        f"training fleet: expected active_n={expected}, "
                        f"counts={counts}")
                self.model._last_obs = venv.reset()
                self.model._last_episode_starts = np.ones(
                    venv.num_envs, dtype=bool)
                self._retention_failed_since = {}

            def _on_rollout_start(self) -> None:
                if population is not None:
                    population.wait_for_start(self.num_timesteps)
                    checkpoint = population.poll()
                    if checkpoint is not None:
                        self._pending_rollback = None
                        self._restore_checkpoint(checkpoint)
                        self._latest_promotion = checkpoint
                        self._cert_round = max(
                            self._cert_round,
                            int(checkpoint.get("cert_round", 0)))
                        population.acknowledge(
                            checkpoint, self.num_timesteps)
                        print("[recover-pop] ADOPTED elected B"
                              f"{checkpoint['bucket']} checkpoint from "
                              f"member {checkpoint['population_record']['member']} "
                              f"at local step {self.num_timesteps:,}")
                        population.wait_for_release(
                            checkpoint, self.num_timesteps)
                        return
                if self._pending_rollback is None:
                    return
                rollback = self._pending_rollback
                self._pending_rollback = None
                checkpoint = rollback["checkpoint"]
                self._restore_checkpoint(checkpoint)
                self._rollback_count += 1
                payload = {
                    "global_step": self.num_timesteps,
                    "RECOVER_GUARD/rollback_applied": 1.0,
                    "RECOVER_GUARD/rollback_count": float(
                        self._rollback_count),
                    "RECOVER_GUARD/rollback_to_bucket": float(
                        checkpoint["bucket"]),
                    "RECOVER_GUARD/rollback_checkpoint_step": float(
                        checkpoint["step"]),
                    "RECOVER_GUARD/rollback_trigger_bucket": float(
                        rollback["trigger_bucket"]),
                }
                if run is not None:
                    import wandb
                    wandb.log(payload)
                print("[recover-guard] ROLLBACK applied: "
                      f"B{rollback['trigger_bucket']} remained below "
                      f"{args.recover_rollback_fraction:.2f}; restored "
                      f"promotion B{checkpoint['bucket']} from step "
                      f"{checkpoint['step']:,}")

            def _build(self):
                cert_kw = dict(vec_kw)
                cert_kw.update(
                    seed=args.seed + 515151,
                    pool_per_env=max(2, args.pool_per_env),
                    desync_episodes=False)
                if args.host_workers > 0:
                    from .mjx_sharded_vec_env import MjxShardedVecEnv
                    workers = min(args.recover_cert_envs,
                                  max(1, args.host_workers))
                    env = MjxShardedVecEnv(
                        env_cls, args.recover_cert_envs,
                        host_workers=workers, **cert_kw)
                else:
                    from .mjx_vec_env import MjxVecEnv
                    env = MjxVecEnv(
                        env_cls, args.recover_cert_envs, **cert_kw)
                env.env_method("set_goal_mix", gm)
                print("[recover-cert] deterministic MJX pool ready: "
                      f"{args.recover_cert_envs} envs, "
                      f"{impl or 'jax(default)'} backend")
                return env

            def _on_step(self) -> bool:
                return True

            def _on_rollout_end(self) -> None:
                if self.num_timesteps < self._next:
                    return
                self._next = ((self.num_timesteps
                               // args.recover_cert_every) + 1
                              ) * args.recover_cert_every
                if self._env is None:
                    self._env = self._build()
                active_before = int(venv.get_attr(
                    "_rec_active_n", indices=0)[0])
                frontier_before = int(venv.get_attr(
                    "_rec_focus_bucket", indices=0)[0])
                weak = venv.get_attr(
                    "_rec_weak_bucket", indices=0)[0]
                buckets, self._retention_cursor = (
                    _recover_cert_bucket_plan(
                        frontier_before, args.recover_retention_buckets,
                        self._retention_cursor, weak))
                rows_by_bucket = {}
                self._cert_round += 1

                def assay_bucket(bucket: int) -> None:
                    kinds = venv.env_method(
                        "_recover_family_kinds", bucket, indices=0)[0]
                    rows = []
                    for kind in kinds:
                        row = _run_recover_cert_kind(
                            self._env, self.model, kind)
                        rows.append(row)
                        venv.env_method(
                            "apply_recover_certification", kind,
                            row["outcomes"], False, self._cert_round)
                    rows_by_bucket[bucket] = rows
                    self._last_cert_round[bucket] = self._cert_round

                for bucket in buckets:
                    assay_bucket(bucket)

                # Routine rounds stay cheap (frontier + weak + rotating
                # history). Once the frontier is a promotion candidate, assay
                # every unlocked bucket in THIS round. Stale historical wins
                # can no longer promote a policy that has forgotten basics.
                gate = venv.env_method(
                    "_recover_admission_status", self._cert_round,
                    indices=0)[0]
                promotion_candidate = bool(
                    active_before < len(env_cls.RECOVER_FAMILIES)
                    and gate["frontier_passed"])
                periodic_full_due = bool(
                    frontier_before > 0
                    and args.recover_full_retention_every > 0
                    and self._cert_round
                    % args.recover_full_retention_every == 0)
                full_suite_attempted = bool(
                    promotion_candidate or periodic_full_due)
                if full_suite_attempted:
                    for bucket in range(frontier_before + 1):
                        if bucket not in rows_by_bucket:
                            buckets.append(bucket)
                            assay_bucket(bucket)
                admission, synchronized_envs = _recover_update_admission_all(
                    venv, self._cert_round)
                active_after = int(admission["active_after"])
                payload = {
                    "global_step": self.num_timesteps,
                    "CERT/recover_frontier_before": frontier_before,
                    "CERT/recover_frontier_after": active_after - 1,
                    "CERT/recover_max_unlocked_before": active_before - 1,
                    "CERT/recover_max_unlocked_after": active_after - 1,
                    "CERT/recover_assayed_bucket_count": len(buckets),
                    "CERT/recover_promotion_candidate": float(
                        promotion_candidate),
                    "CERT/recover_retention_suite_attempted": float(
                        full_suite_attempted),
                    "CERT/recover_retention_suite_passed": float(
                        full_suite_attempted
                        and admission["retention_passed"]),
                    "CERT/recover_retention_bucket_count": float(
                        admission["retention_bucket_count"]),
                    "CERT/recover_retention_min_gate_fraction": float(
                        admission["retention_min_gate_fraction"]),
                    "CERT/recover_retention_failed_bucket_count": float(
                        len(admission["retention_failed_buckets"])
                        if full_suite_attempted else 0),
                    "CERT/recover_promoted": float(admission["promoted"]),
                    "CERT/recover_training_envs_synchronized": float(
                        synchronized_envs),
                }
                for bucket, rows in rows_by_bucket.items():
                    successes = sum(r["successes"] for r in rows)
                    episodes = sum(r["episodes"] for r in rows)
                    stem = f"CERT/recover_bucket_{bucket}"
                    payload[f"{stem}_success_fraction"] = (
                        successes / max(episodes, 1))
                    payload[f"{stem}_gate_fraction"] = min(
                        r["success"] for r in rows)
                    payload[f"{stem}_successes"] = successes
                    payload[f"{stem}_episodes"] = episodes
                    for row in rows:
                        kind = row["kind"]
                        payload[f"CERT/recover_{kind}_success_fraction"] = (
                            row["success"])
                        payload[f"CERT/recover_{kind}_successes"] = (
                            row["successes"])
                        payload[f"CERT/recover_{kind}_episodes"] = (
                            row["episodes"])
                        payload[f"CERT/recover_{kind}_time_s"] = (
                            row["time_mean_s"])
                if admission["promoted"]:
                    self._save_promotion_checkpoint(admission)

                if (full_suite_attempted and frontier_before > 0
                        and self._latest_promotion is not None
                        and args.recover_rollback_after_steps > 0):
                    retention_gates = {
                        bucket: min(
                            row["success"]
                            for row in rows_by_bucket[bucket])
                        for bucket in range(frontier_before)
                    }
                    offenders = _recover_update_regression_timers(
                        self._retention_failed_since, retention_gates,
                        int(self.num_timesteps),
                        args.recover_rollback_fraction,
                        args.recover_rollback_after_steps)
                    if offenders and self._pending_rollback is None:
                        trigger = max(
                            offenders,
                            key=lambda bucket: (
                                self.num_timesteps
                                - self._retention_failed_since[bucket],
                                -bucket))
                        self._pending_rollback = {
                            "checkpoint": self._latest_promotion,
                            "trigger_bucket": int(trigger),
                        }

                regression_ages = {
                    bucket: self.num_timesteps - failed_at
                    for bucket, failed_at
                    in self._retention_failed_since.items()
                }
                payload.update({
                    "RECOVER_GUARD/promotion_checkpoint_saved": float(
                        admission["promoted"]),
                    "RECOVER_GUARD/promotion_checkpoint_count": float(
                        self._promotion_checkpoint_count),
                    "RECOVER_GUARD/latest_checkpoint_bucket": float(
                        self._latest_promotion["bucket"]
                        if self._latest_promotion is not None else -1),
                    "RECOVER_GUARD/latest_checkpoint_step": float(
                        self._latest_promotion["step"]
                        if self._latest_promotion is not None else -1),
                    "RECOVER_GUARD/regressed_bucket_count": float(
                        len(regression_ages)),
                    "RECOVER_GUARD/max_regression_age_steps": float(
                        max(regression_ages.values(), default=0)),
                    "RECOVER_GUARD/rollback_pending": float(
                        self._pending_rollback is not None),
                    "RECOVER_GUARD/rollback_count": float(
                        self._rollback_count),
                })
                for bucket, age in regression_ages.items():
                    payload[
                        f"RECOVER_GUARD/bucket_{bucket:02d}_"
                        "regression_age_steps"
                    ] = float(age)
                score_state = venv.env_method(
                    "recover_score_state", indices=0)[0]
                ages = {
                    bucket: self._cert_round - last_round
                    for bucket, last_round in self._last_cert_round.items()
                }
                score_payload, self._best_score = _recover_score_payload(
                    score_state, self._best_score, ages)
                payload.update(score_payload)
                if run is not None:
                    import wandb
                    wandb.log(payload)
                bits = " | ".join(
                    f"B{bucket} " + " ".join(
                        f"{r['kind']}={r['successes']}/{r['episodes']}"
                        for r in rows)
                    for bucket, rows in rows_by_bucket.items())
                score_points = score_payload[
                    "RECOVER_SCORE/overall_points"]
                score_max = score_payload["RECOVER_SCORE/maximum_points"]
                print(f"[recover-cert] step {self.num_timesteps:,} "
                      f"{bits}; frontier "
                      f"B{frontier_before}->B{active_after - 1}; "
                      f"score={score_points:.1f}/{score_max:.0f}")

            def close(self):
                if self._env is not None:
                    self._env.close()
                    self._env = None

            def _on_training_end(self) -> None:
                self.close()

        cert_cb = _MjxRecoverCert()
        callbacks.append(cert_cb)
        print("[recover-cert] armed: deterministic MJX certification "
              f"every {args.recover_cert_every:,} steps, "
              f"{args.recover_cert_envs} episodes/kind, frontier + weakest "
              f"+ {args.recover_retention_buckets} rotating retention "
              "buckets; a passing frontier triggers a fresh full retention "
              "suite before promotion; full retention every "
              f"{args.recover_full_retention_every or 'promotion-only'} "
              "round(s), promotion checkpoints + rollback after "
              f"{args.recover_rollback_after_steps:,} regressed steps")

    walkcurr_cb = None
    if args.walk_curriculum:
        import copy as _copy
        import shutil

        from .walk_task import (WALKCURR_BUCKETS, WALKCURR_BUCKETS_V2,
                                WALKCURR_BUCKETS_V3, WALKCURR_BUCKETS_V4,
                                WALKCURR_BUCKETS_V5)
        from .walkcurr_cert import (WalkCurrController,
                                    aggregate_walk_probe,
                                    failed_probe_row,
                                    walkcurr_bucket_pass)
        wc_table = {1: WALKCURR_BUCKETS, 2: WALKCURR_BUCKETS_V2,
                    3: WALKCURR_BUCKETS_V3, 4: WALKCURR_BUCKETS_V4,
                    5: WALKCURR_BUCKETS_V5}[args.walk_curriculum_version]
        core_venv = _unwrap_vec(venv)
        wc_best_path = POLICY_DIR / f"{out_name}_best.zip"
        wc_promo_dir = POLICY_DIR / "walkcurr_promotions"
        wc_promo_dir.mkdir(parents=True, exist_ok=True)

        class _MjxWalkCurrCert(BaseCallback):
            """Deterministic walk-curriculum authority ON TRAINING
            PHYSICS: assays run on a dedicated MJX/Warp cert env (same
            backend/env class/cfg as training; C evaluation is never an
            admission signal — the recover-cert precedent) with the
            in-env walk probe measuring the eval_task quality metrics.
            Same admission contract as train_ppo_transfer's WalkCurrCb:
            all-env certification broadcast, frontier+retention
            promotion, 2-consecutive-retained-failure rollback, best =
            last retention-clean promotion. Rollbacks apply at the NEXT
            _on_rollout_start (recover-cert pattern) so replacing the
            policy never invalidates collected old-policy log-probs;
            reset pools are flushed on every admission change so stale
            pooled episodes cannot leak a re-locked bucket's
            distribution into training."""

            def __init__(self):
                super().__init__()
                self._next = int(args.walkcurr_cert_every)
                self.ctl = WalkCurrController(args.walkcurr_fail_streak)
                self.cert_round = 0
                self._env = None
                self.promo_path: Path | None = None
                self.promo_state: dict | None = None
                self._pending_rollback = False

            def _build(self):
                cert_kw = dict(vec_kw)
                ek = _copy.deepcopy(env_kw)
                cfg_d = ek.get("cfg")
                if cfg_d is None:
                    raise RuntimeError("walkcurr env kwargs carry no "
                                       "cfg (goal.walk_curriculum "
                                       "injection failed)")
                cfg_d.setdefault("goal", {})["walk_probe"] = 1.0
                cert_kw.update(env_kwargs=ek, seed=args.seed + 616161,
                               pool_per_env=1, desync_episodes=False)
                n_cert = int(args.walkcurr_cert_episodes)
                if args.host_workers > 0:
                    from .mjx_sharded_vec_env import MjxShardedVecEnv
                    env = MjxShardedVecEnv(
                        env_cls, n_cert,
                        host_workers=min(n_cert,
                                         max(1, args.host_workers)),
                        **cert_kw)
                else:
                    from .mjx_vec_env import MjxVecEnv
                    env = MjxVecEnv(env_cls, n_cert, **cert_kw)
                print("[walkcurr-cert] deterministic MJX cert env "
                      f"ready: {n_cert} episodes/bucket, "
                      f"{impl or 'jax(default)'} backend, in-env walk "
                      "probe ON")
                if _prof_ramp_steps > 0:
                    # Mirror the TRAINING profile ramp: certs must
                    # assay the dose the policy is actually training
                    # under, not the cfg target (num_timesteps is 0 at
                    # the pre-PPO init cert — the ramp START, which is
                    # exactly what the transplant guard should see).
                    rv = _profile_ramp_apply(
                        env, getattr(self, "num_timesteps", 0) or 0)
                    print("[walkcurr-cert] profile ramp mirrored: "
                          f"frac={rv['frac']:.3f} write_speed="
                          f"{rv['write_speed_counts_s']:.0f} counts/s")
                return env

            def _assay(self, bucket: int) -> dict:
                """One bucket's deterministic held-out assay: fixed
                seeds, forced bucket, first episode per cert env."""
                env = self._env
                env.flush_reset_pools()   # rebuild pools under the
                # forced bucket + fresh rng (also bounds pool growth
                # across rounds: reset() appends pool entries)
                env.set_attr("force_walk_curr_bucket", int(bucket))
                env.seed(70_000 + 1_000 * bucket)
                obs = env.reset()
                n_envs = int(env.num_envs)
                finished = np.zeros(n_envs, dtype=bool)
                rows: list = [None] * n_envs
                ep_start = np.ones(n_envs, dtype=bool)
                state = None
                horizon = int(env.get_attr("episode_steps",
                                           indices=0)[0]) + 2
                ticks = 0
                while not bool(np.all(finished)):
                    actions, state = self.model.predict(
                        obs, state=state, episode_start=ep_start,
                        deterministic=True)
                    obs, _r, dones, infos = env.step(actions)
                    ticks += 1
                    ep_start = np.asarray(dones, dtype=bool)
                    for i in np.flatnonzero(np.asarray(dones)
                                            & ~finished):
                        info = infos[int(i)]
                        wp = info.get("walk_probe")
                        rows[i] = (dict(wp) if wp is not None
                                   else failed_probe_row())
                        finished[i] = True
                    if ticks > horizon:
                        missing = np.flatnonzero(~finished).tolist()
                        raise RuntimeError(
                            "walkcurr certification exceeded the "
                            f"episode horizon for envs {missing}")
                env.set_attr("force_walk_curr_bucket", None)
                return aggregate_walk_probe(rows)

            def _apply_rollback(self) -> None:
                # After the preceding PPO update, before the next
                # rollout: policy swap cannot invalidate old-policy
                # log-probs here.
                from stable_baselines3.common.save_util import (
                    load_from_zip_file)
                from .update_health import (
                    load_optimizer_state_if_compatible)
                assert self.promo_path is not None
                _, params_, _ = load_from_zip_file(
                    str(self.promo_path), device=self.model.device)
                if args.walkcurr_actor_only_rollback:
                    # Bridge2 spec fb_20260818T112826_9ed832 item 1:
                    # restore the ACTOR only — bridge1-retry1 showed
                    # whole-policy retention rollback repeatedly
                    # undoing the fresh critic's own adaptation
                    # (critic EV +.304 -> -.183 across 3 rollbacks).
                    # Critic/value head, predictive residual, frozen
                    # encoder and the critic optimizer moments are
                    # never reset; actor Adam moments restart fresh.
                    from .update_health import (
                        restore_actor_only_from_state)
                    non_actor = (("value_net", "vf_features_extractor",
                                  "value_gate", "latent_adapter",
                                  "critic_predictor", "obs_to_frames",
                                  "snapshot_version"))
                    logger = getattr(self.model, "logger", None)
                    ev_now = (logger.name_to_value.get(
                        "train/explained_variance")
                        if logger is not None else None)
                    n_rest, critic_ok = restore_actor_only_from_state(
                        self.model.policy, params_["policy"],
                        optimizer=self.model.policy.optimizer,
                        critic_markers=non_actor)
                    saved_actor_version = params_["policy"].get(
                        "actor_snapshot_version")
                    if (saved_actor_version is not None
                            and hasattr(self.model.policy,
                                        "actor_snapshot_version")):
                        with th.no_grad():
                            self.model.policy.actor_snapshot_version.copy_(
                                saved_actor_version.to(
                                    self.model.policy.
                                    actor_snapshot_version.device))
                    if not critic_ok:
                        raise RuntimeError(
                            "actor-only rollback mutated a critic/"
                            "frozen tensor — marker set out of sync "
                            "with the policy architecture")
                    if run is not None:
                        import wandb
                        wandb.log({
                            "global_step": self.num_timesteps,
                            "walkcurr/rollback_actor_only": 1.0,
                            "walkcurr/rollback_actor_tensors":
                                float(n_rest),
                            "walkcurr/rollback_critic_unchanged":
                                float(critic_ok),
                            **({"walkcurr/rollback_critic_ev":
                                float(ev_now)}
                               if ev_now is not None else {})})
                else:
                    self.model.policy.load_state_dict(
                        params_["policy"])
                if (not args.walkcurr_actor_only_rollback
                        and "policy.optimizer" in params_):
                    # BUG FIX 2026-08-18 (bridge1 crash): a
                    # save_stock_optimizer checkpoint (any --actor-lr/
                    # condition-D run) carries a single-group stock
                    # optimizer snapshot on purpose (eval
                    # compatibility) — blindly loading it into the
                    # live 2-group actor/critic optimizer raises
                    # "different number of parameter groups" the first
                    # time a run actually reaches promotion+rollback.
                    # Group-count-gated: policy weights always
                    # restore exactly; optimizer momentum resets fresh
                    # only when the checkpoint's group shape differs
                    # (same contract warm-starts already accept).
                    load_optimizer_state_if_compatible(
                        self.model.policy.optimizer,
                        params_["policy.optimizer"],
                        context="walkcurr rollback")
                venv.env_method("restore_walkcurr_checkpoint_state",
                                self.promo_state)
                flushed = core_venv.flush_reset_pools()
                self.model._last_obs = venv.reset()
                self.model._last_episode_starts = np.ones(
                    venv.num_envs, dtype=bool)
                print(f"  walkcurr ROLLBACK to {self.promo_path.name} "
                      f"@ {self.num_timesteps} (2 consecutive "
                      "retained-failure rounds; "
                      f"{flushed} stale pool entries flushed, "
                      "fresh rollout state)")
                if run is not None:
                    import wandb
                    wandb.log({"global_step": self.num_timesteps,
                               "walkcurr/rollback_applied": 1.0,
                               "walkcurr/pool_flushed": float(flushed)})

            def _on_rollout_start(self) -> None:
                if self._pending_rollback:
                    self._pending_rollback = False
                    self._apply_rollback()

            def _on_step(self) -> bool:
                return True

            def _on_rollout_end(self) -> None:
                if self.num_timesteps < self._next:
                    return
                self._next = ((self.num_timesteps
                               // args.walkcurr_cert_every) + 1
                              ) * args.walkcurr_cert_every
                if self._env is None:
                    self._env = self._build()
                elif _prof_ramp_steps > 0:
                    # keep the cert env's profile at the training frac
                    _profile_ramp_apply(self._env, self.num_timesteps)
                self.cert_round += 1
                active_n = int(venv.env_method(
                    "walkcurr_state", indices=0)[0]["active_n"])
                payload: dict = {"walkcurr/cert_round": self.cert_round}
                t_cert = time.time()
                for b in range(active_n):
                    spec = wc_table[b]
                    m = self._assay(b)
                    passed, checks = walkcurr_bucket_pass(m, spec)
                    score = m.get("cmd_prog_frac")
                    score = (0.0 if score is None or score != score
                             else float(score))
                    # all-env admission broadcast — every training env
                    # must see the same certification results
                    venv.env_method("apply_walkcurr_certification",
                                    b, passed, score, self.cert_round)
                    pfx = f"walkcurr/b{b}_{spec['name']}"
                    payload.update({
                        f"{pfx}/pass": float(passed),
                        f"{pfx}/cmd_prog_frac": m["cmd_prog_frac"],
                        f"{pfx}/wrong_way": m["wrong_way"],
                        f"{pfx}/vx_rmse": m["vx_rmse"],
                        f"{pfx}/vy_rmse": m["vy_rmse"],
                        f"{pfx}/cross_track_frac": m["cross_track_frac"],
                        f"{pfx}/slip_per_m": m["slip_per_m"],
                        f"{pfx}/peak_roll_deg": m["peak_roll_deg"],
                        f"{pfx}/falls": m["early_term_rate"],
                        f"{pfx}/contact_sw_per_s": m["contact_sw_per_s"],
                        f"{pfx}/foot_sw_min_per_s":
                            m["foot_sw_min_per_s"],
                        f"{pfx}/duty_factor": m["duty_factor"],
                        f"{pfx}/slew_sat": m["slew_sat"],
                        f"{pfx}/stop_speed_m_s": m["stop_speed_m_s"],
                        f"{pfx}/return": m["return"],
                        # Body height + income factor vs the reward
                        # gate's anchor, and the DR the bucket REALIZES
                        # in training (_walkcurr_prepare_episode
                        # overrides --dr-scale per bucket) — operator
                        # order fb_20260818T085648_2a0a60.
                        f"{pfx}/mean_h_m": m["mean_h_m"],
                        f"{pfx}/h_err_mm": m["h_err_mm"],
                        f"{pfx}/height_factor": m["height_factor"],
                        f"{pfx}/height_factor_p10":
                            m["height_factor_p10"],
                        f"{pfx}/cmd_prog_frac_p10":
                            m["cmd_prog_frac_p10"],
                        f"{pfx}/survival_s": m["survival_s"],
                        f"{pfx}/survival_s_min": m["survival_s_min"],
                        f"{pfx}/duration_target_s": float(
                            spec.get("duration_s", args.episode_seconds)),
                        f"{pfx}/command_changes": m["command_changes"],
                        f"{pfx}/command_changes_min":
                            m["command_changes_min"],
                        f"{pfx}/command_changes_target": float(
                            spec.get("min_command_changes", 0)),
                        f"{pfx}/dr": float(spec["dr"]),
                    })
                    fails = [k for k, ok in checks.items() if not ok]
                    print(f"  walkcurr cert r{self.cert_round} b{b} "
                          f"{spec['name']}: "
                          f"{'PASS' if passed else 'FAIL ' + ','.join(fails)}"
                          f" prog={m['cmd_prog_frac']:.2f}"
                          f" slip/m={m['slip_per_m']:.2f}"
                          f" roll={m['peak_roll_deg']:.1f}"
                          f" h_err={m['h_err_mm']:+.1f}mm"
                          f" hf={m['height_factor']:.2f}"
                          f" survive={m['survival_s_min']:.1f}s"
                          f" changes={m['command_changes_min']:.0f}")
                statuses = venv.env_method("walkcurr_update_admission",
                                           self.cert_round)
                status = statuses[0]
                actives = {s["active_n"] for s in statuses}
                assert len(actives) == 1, \
                    f"admission diverged across envs: {actives}"
                action = self.ctl.record_round(status)
                flushed = 0
                if action == "promote":
                    new_frontier = status["active_n"] - 1
                    self.promo_path = wc_promo_dir / (
                        f"{out_name}_promo_b{new_frontier}.zip")
                    self.model.save(str(self.promo_path))
                    self.promo_state = venv.env_method(
                        "walkcurr_checkpoint_state", indices=0)[0]
                    self.promo_path.with_suffix(".json").write_text(
                        json.dumps({"step": self.num_timesteps,
                                    "cert_round": self.cert_round,
                                    "state": self.promo_state},
                                   indent=2) + "\n")
                    shutil.copyfile(self.promo_path, wc_best_path)
                    flushed = core_venv.flush_reset_pools()
                    print("  walkcurr PROMOTION -> frontier "
                          f"b{new_frontier} @ {self.num_timesteps} "
                          f"(saved {self.promo_path.name}, now the "
                          f"best ckpt; {flushed} stale pool entries "
                          "flushed)")
                    if run is not None:
                        try:
                            run.save(str(self.promo_path),
                                     base_path=str(POLICY_DIR),
                                     policy="now")
                            run.summary[
                                "walkcurr_latest_promotion"] = (
                                self.promo_path.name)
                        except Exception as exc:
                            print("[walkcurr] W&B promotion upload "
                                  f"failed (non-fatal): {exc}")
                    # Frontier-gated update-schedule handover on the
                    # FIRST promotion only (default OFF; operator
                    # order fb_20260818T085648_2a0a60).
                    if self.ctl.promotions == 1 and (
                            args.walkcurr_post_promo_epochs > 0
                            or args.walkcurr_post_promo_actor_lr
                            > 0.0):
                        chg = apply_walkcurr_post_promo_schedule(
                            self.model,
                            args.walkcurr_post_promo_epochs,
                            args.walkcurr_post_promo_actor_lr,
                            args.walkcurr_post_promo_actor_lr_final)
                        payload[
                            "walkcurr/post_promo_schedule_applied"
                        ] = 1.0
                        print("  walkcurr POST-PROMO SCHEDULE "
                              f"applied @ {self.num_timesteps}: "
                              + ", ".join(
                                  f"{k} {v[0]:g}->{v[1]:g}"
                                  for k, v in chg.items()))
                elif action == "rollback":
                    self._pending_rollback = True
                state = venv.env_method("walkcurr_state", indices=0)[0]
                payload.update({
                    "walkcurr/frontier": state["frontier_bucket"],
                    "walkcurr/active_n": state["active_n"],
                    "walkcurr/weakest_mastered":
                        state["weakest_mastered"],
                    "walkcurr/promotions": self.ctl.promotions,
                    "walkcurr/rollbacks": self.ctl.rollbacks,
                    "walkcurr/retention_pass":
                        float(status["retention_passed"]),
                    "walkcurr/frontier_pass":
                        float(status["frontier_passed"]),
                    "walkcurr/fail_streak": self.ctl.fail_streak,
                    "walkcurr/pool_flushed": float(flushed),
                    "walkcurr/rollback_pending":
                        float(self._pending_rollback),
                    "walkcurr/cert_wall_s":
                        round(time.time() - t_cert, 1),
                    "walkcurr/base_dr_scale": float(args.dr_scale),
                })
                if run is not None:
                    import wandb
                    wandb.log({"global_step": self.num_timesteps,
                               **payload})

            def close(self):
                if self._env is not None:
                    self._env.close()
                    self._env = None

            def _on_training_end(self) -> None:
                self.close()

        walkcurr_cb = _MjxWalkCurrCert()
        callbacks.append(walkcurr_cb)
        print("[walkcurr] armed: deterministic MJX certification every "
              f"{args.walkcurr_cert_every:,} steps, "
              f"{args.walkcurr_cert_episodes} episodes/bucket, V"
              f"{args.walk_curriculum_version} buckets, promotion "
              "checkpoints (best = last retention-clean promotion), "
              f"rollback after {args.walkcurr_fail_streak} consecutive "
              "retained-failure rounds")
        if args.walkcurr_cert_at_init:
            # Exact pre-PPO deterministic certification of the INITIAL
            # policy (operator order fb_20260818T102844_116d4c item 6):
            # the frontier bucket's own held-out assay — same cert env,
            # same fixed seeds, same probe — runs BEFORE any PPO update
            # and is logged, so a broken transplant/obs mapping is
            # caught at step 0 instead of trained over. Measurement
            # only: results are never broadcast to admission.
            walkcurr_cb.model = model
            if walkcurr_cb._env is None:
                walkcurr_cb._env = walkcurr_cb._build()

            def _f(v, bad):
                v = float(v) if v is not None else float("nan")
                return bad if v != v else v
            n_pre = min(int(args.walkcurr_precert_buckets),
                        len(wc_table))
            prog0 = falls0 = None
            for bi in range(n_pre):
                spec_b = wc_table[bi]
                m_b = walkcurr_cb._assay(bi)
                passed_b, checks_b = walkcurr_bucket_pass(m_b, spec_b)
                fails_b = [k for k, ok in checks_b.items() if not ok]
                prog_b = _f(m_b.get("cmd_prog_frac"), 0.0)
                falls_b = _f(m_b.get("early_term_rate"), 1.0)
                print(f"[walkcurr-precert] b{bi} {spec_b['name']} @ "
                      "step 0 (pre-PPO, det, "
                      f"n={args.walkcurr_cert_episodes}): "
                      f"{'PASS' if passed_b else 'FAIL ' + ','.join(fails_b)}"
                      f" prog={prog_b:.3f} falls={falls_b:.2f}"
                      f" slip/m={_f(m_b.get('slip_per_m'), -1):.2f}"
                      f" roll={_f(m_b.get('peak_roll_deg'), -1):.1f}"
                      f" h_err={_f(m_b.get('h_err_mm'), 0):+.1f}mm"
                      f" hf={_f(m_b.get('height_factor'), 0):.2f}"
                      f" slew={_f(m_b.get('slew_sat'), -1):.2f}"
                      + ("" if bi == 0 else "  [informational]"))
                if run is not None:
                    import wandb
                    pre = {f"walkcurr/pre_b{bi}_{k}": float(m_b[k])
                           for k in ("cmd_prog_frac", "slip_per_m",
                                     "peak_roll_deg",
                                     "early_term_rate",
                                     "height_factor", "h_err_mm",
                                     "mean_h_m", "contact_sw_per_s",
                                     "foot_sw_min_per_s", "slew_sat",
                                     "cross_track_frac", "wrong_way",
                                     "return", "survival_s",
                                     "survival_s_min", "command_changes",
                                     "command_changes_min",
                                     "cmd_prog_frac_p10",
                                     "height_factor_p10")
                           if m_b.get(k) is not None
                           and float(m_b[k]) == float(m_b[k])}
                    wandb.log({"global_step": 0,
                               f"walkcurr/pre_b{bi}_pass":
                                   float(passed_b),
                               **pre})
                    run.summary[f"walkcurr_precert_b{bi}"] = {
                        "pass": bool(passed_b), "prog": prog_b,
                        "falls": falls_b, "fails": fails_b}
                if bi == 0:
                    prog0, falls0 = prog_b, falls_b
            ok0 = (falls0 == 0.0
                   and prog0 >= args.walkcurr_precert_min_prog)
            if args.walkcurr_precert_only:
                walkcurr_cb.close()
                venv.close()
                if run is not None:
                    run.finish()
                print("[walkcurr-precert] precert-only mode: exiting "
                      f"{'PASS (rc 0)' if ok0 else 'FAIL (rc 3)'} "
                      "before any PPO update")
                return 0 if ok0 else 3
            if not ok0:
                raise SystemExit(
                    "[walkcurr-precert] the INITIAL policy fails the "
                    f"survive/walk bar under exact b0 (falls={falls0:.2f},"
                    f" prog={prog0:.3f} < "
                    f"{args.walkcurr_precert_min_prog:.2f}) — fix the "
                    "transplant/obs mapping first rather than training "
                    "over it (fb_20260818T102844_116d4c item 6)")
    bg = None
    if run is not None and (args.eval_every > 0 or args.video_every > 0):
        # The campaign's background eval/video worker, reused verbatim:
        # a spawn process holding C-MuJoCo envs. Every periodic eval is
        # therefore an MJX-vs-C behavioral A/B on the live checkpoint.
        from .train_ppo_sim import (
            _ActFn, _BgEval, _build_env, _make_canary_stop_callback,
            _make_periodic_eval_callback, _make_video_callback,
            _protected_groups, _run_canaries)
        canary_protected: list[str] = []
        if args.canary:
            # Baseline on the PARENT policy (C env, fixed seeds): groups
            # the parent passes 2/2 become the protected set.
            cenv = _build_env(env_cls, params, args, seed=args.seed + 77777)
            act0 = _ActFn(model.policy)
            baseline = _run_canaries(cenv, act0)
            cenv.close()
            canary_protected = _protected_groups(baseline)
            print("[canary] parent baseline: "
                  + " ".join(f"{c}={int(v)}" for c, v in baseline.items()))
            print(f"[canary] protected groups (parent passed 2/2): "
                  f"{canary_protected or 'none'}")
            run.config.update({
                "canary_baseline": {k: int(v) for k, v in baseline.items()},
                "canary_protected": canary_protected,
                "canary_stop_after": args.canary_stop_after,
            }, allow_val_change=True)
        bg = _BgEval(args.task, args)
        if args.eval_every > 0:
            callbacks.append(_make_periodic_eval_callback(
                bg, every=args.eval_every))
            if args.canary and canary_protected:
                callbacks.append(_make_canary_stop_callback(
                    bg, canary_protected,
                    stop_after=args.canary_stop_after))
                print("[canary] regression auto-stop armed "
                      f"(stop after {args.canary_stop_after} consecutive "
                      "full-group failures)")
        if args.video_every > 0:
            callbacks.append(_make_video_callback(bg, args.video_every,
                                                  args))
    if args.best_ckpt or args.ev_stop_min > 0.0:
        from .update_health import EVTracker, HealthTracker

        best_path = POLICY_DIR / f"{out_name}_best.zip"

        class _Health(BaseCallback):
            """Best-checkpoint retention + joint-regression stop +
            critic explained-variance hard gate (fb_20260817T005114
            items 2 and 8). Assays ride the periodic C-env eval; the
            best checkpoint is the LIVE policy at drain time (the
            eval snapshot is a few updates older — close enough for
            retention, and the harness re-scores _best.zip anyway)."""

            def __init__(self):
                super().__init__()
                self._tracker = HealthTracker(
                    regress_n=args.regress_stop_n)
                self._ev = (EVTracker(args.ev_stop_min,
                                      args.ev_stop_after)
                            if args.ev_stop_min > 0.0 else None)
                self._stop_reason: str | None = None

            def _on_step(self) -> bool:
                if self._stop_reason is not None:
                    print(f"[health-stop] {self._stop_reason}")
                    return False
                return True

            def _on_rollout_end(self) -> None:
                payload = {"global_step": self.num_timesteps}
                if self._ev is not None:
                    ev = self.model.logger.name_to_value.get(
                        "train/explained_variance")
                    if ev is not None:
                        ema = self._ev.feed(float(ev))
                        payload["health/explained_variance_ema"] = ema
                        if self._ev.failed(self.num_timesteps):
                            self._stop_reason = (
                                f"critic explained variance EMA "
                                f"{ema:.3f} < {self._ev.ev_min} after "
                                f"{self.num_timesteps:,} steps — value "
                                "learning hard failure (canary gate)")
                if args.best_ckpt and bg is not None:
                    for ep in bg.pop_evals():
                        surv = ep.get("eval/walk/survived_frac")
                        if surv is None:
                            continue
                        dir_err = float(ep.get(
                            "eval/walk/dir_err_deg_mean", 180.0))
                        vel_err = float(ep.get(
                            "eval/walk/vel_err_m_s", 1.0))
                        v_along = max(1.0 - vel_err / 0.05, -1.0)
                        buf = self.model.ep_info_buffer
                        rew = (float(np.mean([e["r"] / max(e["l"], 1)
                                              for e in buf]))
                               if buf else 0.0)
                        res = self._tracker.feed(
                            float(surv), dir_err, v_along, rew)
                        payload["health/composite_score"] = res["score"]
                        payload["health/regress_streak"] = (
                            self._tracker.streak)
                        if res["is_best"]:
                            self.model.save(best_path)
                            payload["health/best_step"] = (
                                self.num_timesteps)
                            print(f"[health] new best composite "
                                  f"{res['score']:.3f} @ "
                                  f"{self.num_timesteps:,} -> "
                                  f"{best_path.name}")
                        if res["should_stop"]:
                            self._stop_reason = (
                                f"reward+survival+direction regressed "
                                f"together for {self._tracker.streak} "
                                f"consecutive assays (best composite "
                                f"{self._tracker.best_score:.3f}) — "
                                "stopping; best checkpoint retained "
                                f"at {best_path.name}")
                if run is not None and len(payload) > 1:
                    import wandb
                    wandb.log(payload)

        callbacks.append(_Health())
        print("[update-health] health callback armed "
              f"(best-ckpt={bool(args.best_ckpt)}, "
              f"ev-gate={args.ev_stop_min or 'off'})")
    try:
        model.learn(total_timesteps=args.steps, callback=callbacks,
                    progress_bar=False)
    finally:
        if cert_cb is not None:
            cert_cb.close()
        if walkcurr_cb is not None:
            walkcurr_cb.close()
        if bg is not None:
            bg.shutdown()
    model.save(out_path)
    if amp_wrap is not None:
        disc_path = out_path.with_suffix(".amp_disc.pt")
        amp_wrap.save(disc_path)
        print(f"[amp-style] discriminator saved -> {disc_path} "
              f"({amp_wrap.disc_updates} updates); continue with "
              f"--amp-disc-init {disc_path.name}")
    dt = time.monotonic() - t0
    print(f"[mjx-train] done: {args.steps:,} steps in {dt:.0f}s "
          f"({args.steps / dt:,.0f} env-steps/s incl. setup) -> {out_path}")
    print("[mjx-train] evaluate with the C-env harness before trusting "
          "anything (MJX_PORT.md phase-2 item 4).")
    if run is not None:
        # Publish the final checkpoint as a W&B artifact so every future
        # warm start (use_artifact above) links into the lineage DAG.
        try:
            import hashlib
            import wandb
            md5 = hashlib.md5(out_path.read_bytes()).hexdigest()[:8]
            art = wandb.Artifact(
                f"ckpt-{out_name}", type="policy-checkpoint",
                metadata={"run": args.run_name, "md5": md5,
                          "steps": args.steps, "task": args.task,
                          "parent_ckpt": (args.init_from.stem
                                          if args.init_from else None)})
            art.add_file(str(out_path))
            run.log_artifact(art, aliases=["latest", args.run_name])
            print(f"[wandb] checkpoint artifact ckpt-{out_name} (md5 {md5})")
        except Exception as ex:
            print(f"[wandb] artifact publish failed (non-fatal): {ex}")
        run.finish()
    venv.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
