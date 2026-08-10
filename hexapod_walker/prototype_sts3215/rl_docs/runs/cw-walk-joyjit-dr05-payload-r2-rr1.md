# cw-walk-joyjit-dr05-payload-r2-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T03:17:14+00:00

**pod**: hexapod-mjx-train-7

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05-payload-r1

**hypothesis**: 3rd launch attempt: base + r1 both died 0-steps to launch-collision/stale-code-marker infra (gotcha 13b + a stale train-pod .code_sha this window, now synced). Same spec unchanged: command-jitter driving (DR0.5) x payload compose.

**gate**: JOYSTICK GATE 0 in-envelope falls; own-cfg (jitter-DR0.5+payload) det+sto gv 6/6, 0 term, prog med>=0.75; DR0 no-payload retention gv 6/6 prog>=0.85; frames watched det

**refused_reason**: hexapod-mjx-train-7 code marker 36153cd70fd4b03481ba029392cbf06335ff5ef0 != local HEAD e840c7735aca9a856ea4461d5efba571a8441126. Sync first: snapshot.sh --sync hexapod-mjx-train-7 (and snapshot/commit before that if the tree is dirty).

