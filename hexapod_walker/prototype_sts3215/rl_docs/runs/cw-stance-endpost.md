# cw-stance-endpost

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T00:23:14+00:00

**pod**: hexapod-sweep-s5

**steps**: 4000000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**hypothesis**: Terminal end-posture pricing: the flag ending survives because no term has gradient on an airborne leg during the terminal phase. k_end_posture=5.0 (schedule-gated clearance charge, last ~1.5s, transients untaxed, allowances mirror eval gate 20/60mm) supplies that gradient, so the planted ending is reachable at INHERITED std (no basin-escape — posture2 just showed flat ent 0.01 warm-start = std runaway). One variable vs cw-stance-posture (same parent cw_stance_dr10 md5 da1d912a, same cfg/seed): +k_end_posture. If-true: lower end-posture >=4/6 det or sto (baseline 0/6 everywhere, worst_clear 256-264mm), heights retained. If-false: worst_clear stays ~250+mm with -46/episode absorbed -> refutes dense-terminal-gradient-suffices; remaining option = belly-rest reference states. Strongest alt: k=5 distorts descent to dodge the window — impossible by construction (window is time-based), which is why this distinguishes. Probe: 150k integration clean + local band check (ledger). Snapshot 5589bc4.

**gate**: posture-strict harness @ DR 1.0, 6 eps/mode det+sto: lower end-posture >=5/6 sto AND >=4/6 det AND rise/lower height-only >=5/6 both AND hold sto 6/6

**verdict**: LAUNCH FAILED at init: parent ppo_goal_cw_stance_dr10.zip missing on the NEW s5 pod (code sync does not carry policies/ - gitignored runtime state). Trainer crashed in seconds (FileNotFoundError .zip.zip), W&B run wwnhar8o exists crashed and blocks the name. Fixed: ckpt kubectl-cp to s5, md5 verified da1d912a; relaunched as cw-stance-endpost-r1 (RUNNING, wandb no0ihywt).

**failed_reason**: log not growing (3093 -> 3093 bytes)

